from commands2 import Command, CommandScheduler
from magicbot import AutonomousStateMachine, state
from pathplannerlib.auto import AutoBuilder, CommandUtil, PathPlannerAuto
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.trajectory import PathPlannerTrajectory
from wpilib import SmartDashboard, SendableChooser, Field2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import DriveTrain
from components.lift import Lift


class basicAuto(AutonomousStateMachine):
    MODE_NAME = "Basic Autonomous"
    DEFAULT = False
    DISABLED = False

    autoChooser: SendableChooser
    lift: Lift
    driveTrain: DriveTrain

    distance_offset = 0
    distance_target = 0
    position_deadband = 0.25

    def get_distance(self):
        return self.driveTrain.get_average_distance() - self.distance_offset

    def zero_distance(self):
        self.distance_offset = self.driveTrain.get_average_distance()

    @state(first = True)
    def initialize(self):
        self.zero_distance()
        self.position_deadband = 0.25
        self.distance_target = 2
        self.next_state("drive")

    @state()
    def drive(self):
        speed = ChassisSpeeds(vx=0, vy=2, omega=0)
        self.driveTrain.driveRobotRelative(speed)
        if self.distance_target - self.get_distance() < self.position_deadband:
            self.next_state("second_state")

    @state()
    def second_state(self):
        speed = ChassisSpeeds(0,0,0)
        self.driveTrain.driveRobotRelative(speed)
