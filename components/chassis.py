import math
import time

import rev
from commands2 import Subsystem
from wpilib import DriverStation, Field2d, SmartDashboard, DutyCycleEncoder
import wpimath.units
from phoenix6.hardware import Pigeon2
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase, SparkLowLevel, SparkClosedLoopController, \
    ClosedLoopConfig, SparkMaxSim, AbsoluteEncoderConfig, RelativeEncoder
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition, \
    SwerveModuleState

from helpers import Component
from collections import namedtuple
from constants import *

MotorConfig = namedtuple("MotorConfig", ["kP", "kI", "kD"])

kMaxSpeed = 3 # Meters per second

# Class to create a single swerve module
class SwerveModule(Subsystem):
    # Initialize all motors and subsystems
    def __init__(self, swerve_can_id, drive_can_id, encoder_type, angle_offset):
        super().__init__()
        self.angle_offset = angle_offset
        self.swerve_motor = SparkMax(swerve_can_id, SparkLowLevel.MotorType.kBrushless)
        self.drive_motor = SparkMax(drive_can_id, SparkLowLevel.MotorType.kBrushless)
        self._swerve_controller = self.swerve_motor.getClosedLoopController()
        self._drive_controller = self.drive_motor.getClosedLoopController()
        self.swerve_motor_config = SparkMaxConfig()

        self.encoder_type = encoder_type
        self.coasted = False
        self.drive_ref = 0

        if self.encoder_type == rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder:
            self.swerve_encoder = self.swerve_motor.getAbsoluteEncoder()
        else:
            self.swerve_encoder = self.swerve_motor.getEncoder()
        self.setup()

    def execute(self):
        SmartDashboard.putNumber("Module Speed", self.drive_motor.getEncoder().getVelocity())

    # Setup code for all functional components
    def setup(self):
        self.swerve_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50).inverted(False)
        self.swerve_motor_config.closedLoop.pid(SwervePID.kP, SwervePID.kI, SwervePID.kD,
                                           rev.ClosedLoopSlot.kSlot0).setFeedbackSensor(self.encoder_type).positionWrappingEnabled(True).positionWrappingMaxInput(1).positionWrappingMinInput(0)
        self.swerve_motor_config.absoluteEncoder.inverted(True)
        self.swerve_motor.configure(self.swerve_motor_config, SparkBase.ResetMode.kResetSafeParameters,
                                    SparkBase.PersistMode.kPersistParameters)

        drive_motor_config = SparkMaxConfig()
        drive_motor_config.apply(self.swerve_motor_config).setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        drive_motor_config.closedLoop.setFeedbackSensor(drive_motor_config.closedLoop.FeedbackSensor.kPrimaryEncoder)
        # closedLoop is the accessor for the closed loop controller configuration
        drive_motor_config.closedLoop.pid(DrivePID.kP, DrivePID.kI, DrivePID.kD, rev.ClosedLoopSlot.kSlot0)
        self.drive_motor.configure(drive_motor_config, SparkBase.ResetMode.kNoResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

    def toggle_coast(self):
        config = SparkMaxConfig()
        if self.coasted:
            config.setIdleMode(config.IdleMode.kBrake)
        else:
            config.setIdleMode(config.IdleMode.kCoast)
        self.coasted ^= 1
        self.swerve_motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)

    @staticmethod
    def motor_to_meters(position_rotations, circumference, gear_ratio):
        return position_rotations * (circumference / gear_ratio)

    # Field relative
    def get_position(self) -> SwerveModulePosition:
        # Get the total distance travelled by the drive motor and the current angle of the swerve encoder.
        # This information is used for odometry, which is in turn used for all automatic control.
        rotations = self.drive_motor.getEncoder().getPosition() # How many revolutions has the drive motor done
        angle_rotations = self.get_swerve_rotations()
        angle = Rotation2d.fromRotations(angle_rotations - 0.25)
        SmartDashboard.putNumber(f"ID {self.swerve_motor.getDeviceId()} Pos", angle.degrees())
        return SwerveModulePosition(math.pi*wpimath.units.inchesToMeters(3)*rotations/DrivePID.Ratio, angle)

    def get_state(self) -> SwerveModuleState:
        drive_motor_rps = self.drive_motor.getEncoder().getVelocity()*60
        angle_rotations = self.get_swerve_rotations()
        angle = Rotation2d.fromRotations(angle_rotations)
        return SwerveModuleState(SwerveModule.motor_to_meters(drive_motor_rps, DrivePID.Circumference, DrivePID.Ratio), angle)

    def set_state(self, state: SwerveModuleState):
        # Get the current rotation and optimize the minimum rotation required to get the desired rotation
        # This also automatically inverts the speed of the motor if required in order to achieve the shortest swerve angle delta
        current_rotation = wpimath.geometry.Rotation2d.fromRotations(self.get_swerve_rotations())
        state.optimize(current_rotation)
        # Slow the drive speed relative to the angle error. This eliminates the jerking motion that we saw during week 1
        state.cosineScale(current_rotation)

        # Convert the desired state data into data for our hardware
        speed = state.speed
        self.set_swerve_rotations(state.angle)
        self.set_drive_speed(speed)

    def set_swerve_rotations(self, rot: Rotation2d):
        target = units.radiansToRotations(rot.radians()) # Convert a rotation2d into rotations
        target += self.angle_offset
        self._swerve_controller.setReference(target, SparkLowLevel.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)

    def set_drive_speed(self, speed: wpimath.units.meters_per_second):
        # Convert from feet per second to RPM
        # speed = speed * 76.39 * DrivePID.Ratio
        # speed = speed * DrivePID.Ratio  #convert wheel RPM to motor RPM
        # self._drive_controller.setReference(speed, SparkLowLevel.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0, arbFeedforward=DrivePID.kFF)
        speed_scalar = wpimath.units.feetToMeters(20.86)
        self.drive_motor.set(speed/speed_scalar)

    def on_disable(self):
        self.swerve_motor.disable()
        self.drive_motor.disable()

    # Robot Relative
    def get_swerve_rotations(self):
        real_rotation = self.swerve_encoder.getPosition() - self.angle_offset
        real_rotation = wpimath.inputModulus(real_rotation, 0, 1)
        return real_rotation

# Class to combine all swerve modules and drive the robot.
#TODO, Chassis needs "getPose" "resetPose" "getRobotRelativeSpeeds" "getCurrentSpeeds" "driveRobotRelative"
# See https://pathplanner.dev/pplib-getting-started.html#install-pathplannerlib for details
class DriveTrain(Subsystem):

    swerve: dict[str: SwerveModule]
    def __init__(self, field: Field2d):
        super().__init__()
        self.field = field

        self.gear = 1
        self.max_rot = 0

        self.pose = Pose2d(0,0,0)

        self.swerve = {"NW": SwerveModule(CanIDs.SwerveSparkNW, CanIDs.DriveSparkNW, rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder, 0.5),
                       "NE": SwerveModule(CanIDs.SwerveSparkNE, CanIDs.DriveSparkNE, rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder, 0.75),
                       "SW": SwerveModule(CanIDs.SwerveSparkSW, CanIDs.DriveSparkSW, rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder, 0.25),
                       "SE": SwerveModule(CanIDs.SwerveSparkSE, CanIDs.DriveSparkSE, rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder, 0)}

        self.swervepositions = {"NW": Translation2d(-centerPoint[0], centerPoint[1]),
                                "NE": Translation2d(centerPoint[0], centerPoint[1]),
                                "SW": Translation2d(-centerPoint[0], -centerPoint[1]),
                                "SE": Translation2d(centerPoint[0], -centerPoint[1])}
        self.kinematics = SwerveDrive4Kinematics(*self.swervepositions.values())
        self.speed = ChassisSpeeds(0,0,0)
        self.stopdrive = ChassisSpeeds(0,0,0)
        self.running = False

        self.gyro = Pigeon2(CanIDs.Gyro, "rio")

        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.gyro.getRotation2d(),
            (
                self.swerve["NW"].get_position(),
                self.swerve["NE"].get_position(),
                self.swerve["SW"].get_position(),
                self.swerve["SE"].get_position()
            ))

    def update_odometry(self):
        self.odometry.update(self.gyro.getRotation2d(), self.get_module_positions())

    @staticmethod
    def shouldFlipPath() -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def drive_all_percent(self, percent):
        for mod in self.swerve:
            self.swerve[mod].drive_motor.set(percent)

    def getPose(self):
        return self.odometry.getPose()

    def resetPose(self, pose: Pose2d):
        self.odometry.resetPose(pose)

    def getRobotRelativeSpeeds(self):
        return self.kinematics.toChassisSpeeds(self.get_module_states())

    def driveRobot(self, vx, vy, vrot, period, field_relative = False, driver_relative = False):

        # Handle all of the kinematics to drive the robot. Our coordinate system is messed up, which is why the vx and
        # vy are mixed up. This works for now, but might be worth fixing later for completeness' sake
        states = self.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(-vy, vx, vrot, self.gyro.getRotation2d())
                if field_relative
                else ChassisSpeeds(-vy,vx,vrot), period
            )
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, kMaxSpeed) # Limit speeds without changing the trajectory
        self.swerve["NW"].set_state(states[0])
        self.swerve["NE"].set_state(states[1])
        self.swerve["SW"].set_state(states[2])
        self.swerve["SE"].set_state(states[3])


    def on_disable(self):
        self.set_angle(0)

    def get_module_states(self):
        return tuple([module.get_state() for module in self.swerve.values()])

    def get_module_positions(self) -> tuple[SwerveModulePosition]:
        return tuple([module.get_position() for module in self.swerve.values()])

    def on_enable(self):
        self.running = True

    def periodic(self):
        self.pose = self.odometry.update(self.gyro.getRotation2d(), self.get_module_positions())
        data = [self.pose.X(), self.pose.Y(), self.pose.rotation().degrees()]
        SmartDashboard.putNumberArray("Module Distances", data)
        SmartDashboard.putNumber("Gyro Heading", self.gyro.getRotation2d().degrees())
        vel = self.gyro.get_angular_velocity_z_world().value
        if vel > self.max_rot:
            self.max_rot = vel
        SmartDashboard.putNumber("Gyro Z Vel", self.max_rot)
        self.field.setRobotPose(self.pose)

    def set_angle(self, angle):
        if self.running:
            for drive in self.swerve.values():
                drive.set_swerve_rotations(Rotation2d.fromDegrees(angle))

    def get_average_distance(self):
        positions = self.get_module_positions()
        avg = 0
        for pos in positions:
            avg += pos.distance
        avg /= 4
        return avg

    def represent_pose(self):
        return f"{self.odometry.getPose().X()}, {self.odometry.getPose().Y()}, {self.odometry.getPose().Y()}"