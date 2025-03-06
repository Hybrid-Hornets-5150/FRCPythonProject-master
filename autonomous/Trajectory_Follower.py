import wpimath.trajectory
import wpimath.geometry
import wpimath.units
from choreo import SwerveTrajectory
from magicbot import AutonomousStateMachine, state
from wpilib import SmartDashboard, SendableChooser, Field2d, Timer, DriverStation
from wpimath._controls._controls.controller import LTVUnicycleController, HolonomicDriveController, PIDController, \
    ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import Trajectory, TrajectoryGenerator, TrapezoidProfileRadians
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import DriveTrain
from components.lift import Lift
from constants import autonSpeedScaling


class TrajectoryFollower(AutonomousStateMachine):
    MODE_NAME = "Trajectory"
    DEFAULT = True
    DISABLED = False

    driveTrain: DriveTrain
    field: Field2d
    automaticController: HolonomicDriveController

    # trajectory = Trajectory()
    trajectory: SwerveTrajectory
    timer = Timer()
    splits = []
    events = []

    def is_red_alliance(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    @state(first=True)
    def initialize(self):
        # self.trajectory = TrajectoryGenerator.generateTrajectory(
        #     wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d.fromDegrees(0)),
        #     [
        #         wpimath.geometry.Translation2d(1.5, 0),
        #         wpimath.geometry.Translation2d(1.5, -1),
        #         wpimath.geometry.Translation2d(0.75, -0.5),
        #     ],
        #     wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d.fromDegrees(0)),
        #     wpimath.trajectory.TrajectoryConfig(
        #         wpimath.units.feetToMeters(3.0), wpimath.units.feetToMeters(3.0)
        #     ),
        # )
        self.timer = Timer()
        self.timer.start()
        self.splits = self.trajectory.splits
        self.events = self.trajectory.events
        self.driveTrain.resetPose(self.trajectory.get_initial_pose(self.is_red_alliance()))
        self.next_state("follow_path")

    @state()
    def follow_path(self):
        if self.trajectory:
            next_sample = self.trajectory.sample_at(self.timer.get())
            next_pose = next_sample.get_pose()
            next_heading = next_pose.rotation()

            calculated_robot_speed = self.automaticController.calculate(self.driveTrain.getPose(), next_pose,
                                                                        next_sample.vx + next_sample.vy, next_heading)
            vx = calculated_robot_speed.vx
            vy = calculated_robot_speed.vy
            vrot = calculated_robot_speed.omega
            self.driveTrain.driveRobot(vx, vy, vrot, 0.02, field_relative=True)


    @state()
    def at_target(self):
        self.driveTrain.driveRobot(0, 0, 0, 0.02)
