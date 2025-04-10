from commands2 import Command, CommandScheduler
from magicbot import AutonomousStateMachine, state
from wpilib import SmartDashboard, SendableChooser, Field2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import DriveTrain
from components.lift import Lift
from constants import autonSpeedScaling


class BasicAuto(AutonomousStateMachine):
    MODE_NAME = "Basic Autonomous"
    DEFAULT = False
    DISABLED = False

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
        self.driveTrain.driveRobot(autonSpeedScaling, 0, 0,0.02, field_relative=False)
        if self.distance_target - self.get_distance() < self.position_deadband:
            self.next_state("second_state")

    @state()
    def second_state(self):
        self.driveTrain.driveRobot(0, 0, 0, 0.02)
