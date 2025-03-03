import wpimath.trajectory
import wpimath.geometry
import wpimath.units
from magicbot import AutonomousStateMachine, state
from wpilib import SmartDashboard, SendableChooser, Field2d, Timer
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
    DEFAULT = False
    DISABLED = False

    driveTrain: DriveTrain
    field: Field2d

    trajectory = Trajectory()
    timer = Timer()
    controller = HolonomicDriveController(
        PIDController(2,0,0.1),
        PIDController(2,0,0.1),
        ProfiledPIDControllerRadians(2,0,0.1, TrapezoidProfileRadians.Constraints(3,3))
    )

    @state(first = True)
    def initialize(self):
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d.fromDegrees(0)),
            [
                wpimath.geometry.Translation2d(1.5, 0),
                wpimath.geometry.Translation2d(1.5, -1),
                wpimath.geometry.Translation2d(0.75, -0.5),
            ],
            wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d.fromDegrees(0)),
            wpimath.trajectory.TrajectoryConfig(
                wpimath.units.feetToMeters(3.0), wpimath.units.feetToMeters(3.0)
            ),
        )

        self.timer = Timer()
        self.timer.start()
        self.field.setRobotPose(self.trajectory.initialPose())
        self.field.getObject("traj").setTrajectory(self.trajectory)
        self.driveTrain.resetPose(self.trajectory.initialPose())
        self.next_state("follow_path")

    @state()
    def follow_path(self):
        self.driveTrain.update_odometry()
        self.field.setRobotPose(self.driveTrain.getPose())
        if self.timer.get() < self.trajectory.totalTime():
            next_pose = self.trajectory.sample(self.timer.get())

            calculated_robot_speed = self.controller.calculate(self.driveTrain.getPose(), next_pose, Rotation2d(0))
            vx = calculated_robot_speed.vx
            vy = calculated_robot_speed.vy
            vrot = calculated_robot_speed.omega
            self.driveTrain.driveRobot(vx, vy, vrot, 0.02, field_relative=True)

        else:
            self.next_state("at_target")

    @state()
    def at_target(self):
        self.driveTrain.driveRobot(0,0,0,0.02)




