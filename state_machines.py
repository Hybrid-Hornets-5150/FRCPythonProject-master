from magicbot import StateMachine, state, timed_state

from components.chassis import DriveTrain
from components.lift import Grabber, Arm


class IntakeCoral(StateMachine):
    grabber: Grabber
    arm: Arm

    percent_target = 0
    def intake(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.percent_target = 0.5
        self.next_state("begin_intake")

    @timed_state(duration=2.0, next_state="begin_hold")
    def begin_intake(self):
        self.grabber.kicker_percent = -0.1
        self.arm.extension = 2.5
        self.grabber.intake_percent = self.percent_target

    @state()
    def begin_hold(self):
        self.percent_target = 0.05
        self.arm.extension = 0
        self.next_state("hold")

    @state()
    def hold(self):
        self.grabber.intake_percent = self.percent_target

    #Currently unused
    @state()
    def full_speed(self):
        self.grabber.intake_percent = self.percent_target
        self.grabber.kicker_percent = 0.05

class CheckAlignment(StateMachine):
    arm: Arm

    original_angle = 0

    def run(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.original_angle = self.arm.arm_angle
        self.next_state("go_down")

    @state()
    def go_down(self):
        self.arm.arm_angle = self.original_angle - 10

    def done(self):
        super().done()
        self.arm.arm_angle = self.original_angle


class ClearReef(StateMachine):
    arm: Arm

    original_angle = 0

    def run(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.original_angle = self.arm.arm_angle
        self.next_state("go_up")

    @state()
    def go_up(self):
        self.arm.arm_angle = self.original_angle + 10

    def done(self):
        super().done()
        self.arm.arm_angle = self.original_angle

class ScoreCoral(StateMachine):
    grabber: Grabber
    arm: Arm
    driveTrain: DriveTrain

    angle_target = 0
    initial_angle = 0

    def run(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.initial_angle = self.arm.arm_angle
        if self.arm.arm_angle < - 10:
            self.next_state("just_feed_out")
        else:
            self.angle_target = self.arm.arm_angle - 10
            self.next_state("begin_scoring")

    @timed_state(duration=2.0)
    def just_feed_out(self):
        self.grabber.kicker_percent = 0.5
        self.grabber.intake_percent = -0.2

    @timed_state(duration=1.0, next_state="output_coral")
    def begin_scoring(self):
        self.arm.arm_angle = self.angle_target

    @timed_state(duration=0.5, next_state="run_away")
    def output_coral(self):
        self.grabber.kicker_percent = 0.5
        self.grabber.intake_percent = -0.2

    @timed_state(duration=1, next_state="final")
    def run_away(self): #Like the brave Sir Robin
        self.grabber.intake_percent = -0.2
        self.driveTrain.set_angle(90)
        self.driveTrain.drive_all_percent(-0.1)

    @state()
    def final(self):
        self.arm.arm_angle = self.initial_angle

    def done(self):
        super().done()
        self.arm.arm_angle = self.initial_angle
        self.grabber.kicker_percent = 0
        self.grabber.intake_percent = 0
