import math

import rev
from wpimath import units
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase, SparkLowLevel, SparkClosedLoopController, \
    ClosedLoopConfig
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds

from helpers import Component
from collections import namedtuple
from constants import *

MotorConfig = namedtuple("MotorConfig", ["kP", "kI", "kD"])

class SwerveModule(Component):
    swerve_motor: SparkMax
    drive_motor: SparkMax

    _swerve_controller: SparkClosedLoopController
    _drive_controller: SparkClosedLoopController

    def __init__(self, swerve_can_id, drive_can_id):
        self.swerve_motor = SparkMax(swerve_can_id, SparkLowLevel.MotorType.kBrushless)
        self.drive_motor = SparkMax(drive_can_id, SparkLowLevel.MotorType.kBrushless)
        self._swerve_controller = self.swerve_motor.getClosedLoopController()
        self._drive_controller = self.drive_motor.getClosedLoopController()

        self.setup()

    def execute(self):
        pass

    def setup(self):
        swerve_motor_config = SparkMaxConfig()
        swerve_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50).inverted(True)
        swerve_motor_config.closedLoop.pid(SwervePID.kP, SwervePID.kI, SwervePID.kD,
                                           rev.ClosedLoopSlot.kSlot0)
        self.swerve_motor.configure(swerve_motor_config, SparkBase.ResetMode.kResetSafeParameters,
                                    SparkBase.PersistMode.kPersistParameters)

        drive_motor_config = SparkMaxConfig()
        drive_motor_config.apply(swerve_motor_config).setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        # closedLoop is the accessor for the closed loop controller configuration
        drive_motor_config.closedLoop.pid(DrivePID.kP, DrivePID.kI, DrivePID.kD, rev.ClosedLoopSlot.kSlot0)
        self.drive_motor.configure(drive_motor_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)


    def set_swerve_rotations(self, rot):
        self._swerve_controller.setReference(rot*SwervePID.Ratio, SparkLowLevel.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)

    def set_drive_speed(self, speed):
        # Convert from feet per second to RPM
        speed = speed * 76.39 * DrivePID.Ratio
        speed = speed * DrivePID.Ratio  #convert wheel RPM to motor RPM
        self._drive_controller.setReference(speed, SparkLowLevel.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def on_disable(self):
        self.swerve_motor.disable()
        self.drive_motor.disable()

class DriveTrain(Component):
    def __init__(self):
        self.gear = 1
        self.swerve = {"NW": SwerveModule(CanIDs.SwerveSparkNW, CanIDs.DriveSparkNW),
                       "NE": SwerveModule(CanIDs.SwerveSparkNE, CanIDs.DriveSparkNE),
                       "SW": SwerveModule(CanIDs.SwerveSparkSW, CanIDs.DriveSparkSW),
                       "SE": SwerveModule(CanIDs.SwerveSparkSE, CanIDs.DriveSparkSE)}

        self.swervepositions = {"NW": Translation2d(-centerPoint[0], centerPoint[1]),
                                "NE": Translation2d(centerPoint[0], centerPoint[1]),
                                "SW": Translation2d(-centerPoint[0], -centerPoint[1]),
                                "SE": Translation2d(centerPoint[0], -centerPoint[1])}
        self.kinematics = SwerveDrive4Kinematics(*self.swervepositions.values())
        self.speed = ChassisSpeeds(0,0,0)
        self.running = False

    def setup(self):
        pass

    def on_disable(self):
        self.stop()

    def on_enable(self):
        self.running = True

    def execute(self):
        pass

    def teleop(self):
        states = self.kinematics.toSwerveModuleStates(self.speed)
        items = list(self.swerve.values())
        for i, state in enumerate(states):
            items[i].set_swerve_rotations(state.angle.degrees() / 360)
            items[i].set_drive_speed(state.speed_fps)

    def set_velocity(self, vx, vy):
        self.speed.vx_fps = vx
        self.speed.vy_fps = vy


    def set_rotation(self, rotation):
        rotation = rotation * 2 * math.pi
        self.speed.omega = rotation

    def drive_straight(self, speed):
        if self.running:
            for drive in self.swerve.values():
                drive.set_drive_speed(speed)
        else:
            print("You can't drive straight right now. The robot is off")

    def set_angle(self, angle):
        if self.running:
            for drive in self.swerve.values():
                drive.set_swerve_rotations(angle)

    def stop(self):
        for drive in self.swerve.values():
            drive.set_swerve_rotations(0)
            drive.set_drive_speed(0)
        self.speed.vy_fps = 0
        self.speed.vx_fps = 0
        self.speed.omega = 0
        self.running = False