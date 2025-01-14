from magicbot import AutonomousStateMachine, timed_state, state
import wpilib
from helpers import Auton


class ExampleAuton(Auton, AutonomousStateMachine):

    MODE_NAME = "Example Auton"
    DEFAULT = False
    DISABLED = True

    @timed_state(duration=3, first=True)
    def drive_forward(self):
        """Drive forward for 3 seconds"""
        pass
