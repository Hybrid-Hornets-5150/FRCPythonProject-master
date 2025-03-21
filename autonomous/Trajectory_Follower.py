import math
import time

from choreo import SwerveTrajectory
from magicbot import AutonomousStateMachine, state
from wpilib import Field2d, Timer, DriverStation, DataLogManager
from wpimath._controls._controls.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians
from wpiutil._wpiutil.log import StringLogEntry

from components.chassis import DriveTrain
from components.lift import Lift, Arm
from robot import MyRobot
from state_machines import ScoreCoralRight


class TrajectoryFollower(AutonomousStateMachine):
    MODE_NAME = "Trajectory"
    DEFAULT = True
    DISABLED = False

    # Required components. Will be automatically injected by magicrobot
    driveTrain: DriveTrain
    lift: Lift
    arm: Arm
    field: Field2d
    coral_autoscore_right_fsm: ScoreCoralRight

    x_controller = PIDController(4.0, 0.0, 0.0)
    y_controller = PIDController(4.0, 0.0, 0.0)
    heading_controller = PIDController(2.75, 0.0, 0.0)

    heading_controller.enableContinuousInput(-math.pi, math.pi)

    # trajectory = Trajectory()
    trajectory: SwerveTrajectory

    # Define class variables
    timer = Timer()
    events = []
    go_to_pose = False

    def is_red_alliance(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def process_events(self, timestamp):
        for event in self.events:
            if timestamp >= event.timestamp:
                self.events.remove(event)
                print("Handling event: ", event.event)
                # Run the event if it is defined
                if event.event in self.state_names:
                    self.next_state(event.event)
                else:
                    print("Invalid event: ", event.event)

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
        self.go_to_pose = False
        self.timer = Timer()
        self.timer.start()
        self.events = self.trajectory.events.copy()
        self.driveTrain.resetPose(self.trajectory.get_initial_pose(self.is_red_alliance()))
        self.lift.set_height(0)
        self.next_state("follow_path")

    @state()
    def follow_path(self):
        timestamp = self.timer.get()
        self.process_events(timestamp)
        if self.trajectory:
            sample = self.trajectory.sample_at(timestamp, self.is_red_alliance())
            pose = self.driveTrain.getPose()

            vx = sample.vx + self.x_controller.calculate(pose.X() , sample.x)
            vy = sample.vy + self.y_controller.calculate(pose.Y(), sample.y)
            vrot = sample.omega + self.heading_controller.calculate(pose.rotation().radians(), sample.heading)

            self.driveTrain.driveRobot(vx, vy, vrot, 0.02, field_relative=True)
            if self.go_to_pose:
                self.next_state("score_coral")
            if timestamp > self.trajectory.get_total_time() + 10:
                self.next_state("at_target")

    @state()
    def start_lift(self):
        """This is an event. To run automatically, have an event with the name start_lift in choreo

        :return:
        """

        self.lift.set_height(26)
        self.arm.arm_angle = 55
        self.next_state("follow_path")

    @state()
    def score_coral(self):
        self.timer.stop()
        self.go_to_pose = True
        if False and (not self.x_controller.atSetpoint() or not self.y_controller.atSetpoint()):
            self.next_state("follow_path")
        else:
            self.driveTrain.driveRobot(0,0,0,0.02)
            self.coral_autoscore_right_fsm.run()
            if self.coral_autoscore_right_fsm.current_state == "finished":
                self.timer.start()
                self.go_to_pose = False
                self.next_state("follow_path")

    @state()
    def lower_lift(self):
        self.lift.set_height(0)
        self.arm.arm_angle = -75
        self.next_state("follow_path")

    @state()
    def at_target(self):
        self.driveTrain.driveRobot(0, 0, 0, 0.02)
