from magicbot import AutonomousStateMachine, state

from components.chassis import DriveTrain
from components.lift import Lift
from state_machines import ScoreCoralRight


class VisionMiddle(AutonomousStateMachine):
    MODE_NAME = "Vision Middle"
    DEFAULT = False
    DISABLED = False

    lift: Lift
    driveTrain: DriveTrain
    coral_autoscore_right_fsm: ScoreCoralRight

    @state(first=True)
    def initialize(self):
        self.coral_autoscore_right_fsm.run()
        if self.coral_autoscore_right_fsm.current_state == "finished":
            self.done()
