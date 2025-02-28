from typing import Union, Callable

import wpimath.units
from commands2 import Command, SequentialCommandGroup, TrapezoidProfileCommand, CommandScheduler
from wpimath._controls._controls.trajectory import TrapezoidProfile

from components.chassis import DriveTrain, SwerveModule
from components.lift import Lift, Grabber, Arm


class ToggleSwerveRotateCoast(Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.addRequirements(drivetrain)
        self.driveTrain = drivetrain
        self.done = False

    def initialize(self):
        for module in self.driveTrain.swerve:
            self.driveTrain.swerve[module].toggle_coast()
        self.done = True
        print("Executed")

    def isFinished(self) -> bool:
        return self.done

    def runsWhenDisabled(self) -> bool:
        return True

class LiftVelocity(Command):
    lift: Lift

    def __init__(self, velocity, lift):
        super().__init__()
        self.lift = lift
        self.addRequirements(self.lift)
        self.velocity = velocity

    def initialize(self):
        self.lift.set_voltage(self.velocity)

    def isFinished(self) -> bool:
        return True

class LiftTo(Command):
    lift: Lift

    def __init__(self, setpoint, lift):
        super().__init__()
        self.lift = lift
        self.addRequirements(self.lift)
        self.setpoint = setpoint

    def execute(self):
        self.lift.set_height(self.setpoint)

    def isFinished(self) -> bool:
        if abs(self.lift.get_height() - self.setpoint) < 2:
            return True
        else:
            return False

class RunIntake(Command):
    grabber: Grabber
    def __init__(self, percent, grabber):
        self.grabber = grabber
        self.addRequirements(self.grabber)
        self.percent = percent
        super().__init__()

    def initialize(self):
        self.grabber.intake_percent = self.percent

    def isFinished(self) -> bool:
        return True

class SetExtension(Command):
    arm: Arm

    def __init__(self, extension, arm):
        self.arm = arm
        self.extension_target = extension
        super().__init__()

    def initialize(self):
        self.arm.extension = self.extension_target

    def isFinished(self) -> bool:
        if abs(self.arm.extension - self.extension_target) < 1:
            print("At Target")
            return True
        else:
            return False

class SetArmAngle(Command):
    def __init__(self, degrees, arm):
        self.target = degrees
        self.arm = arm
        self.addRequirements(self.arm)
        super().__init__()

    def initialize(self):
        self.arm.arm_angle = self.target

    def isFinished(self) -> bool:
        if abs(self.arm.arm_angle - self.target) < 5:
            return True
        else:
            return False

class SetArmVoltage(Command):
    def __init__(self, volts, arm):
        self.volts = volts
        self.arm = arm
        self.addRequirements(self.arm)
        super().__init__()
    def initialize(self):
        self.arm.arm_voltage = self.volts

    def isFinished(self) -> bool:
        return True

class SetKickerPercent(Command):
    def __init__(self, percent, grabber):
        self.target = percent
        self.grabber = grabber
        self.addRequirements(self.grabber)
        super().__init__()

    def initialize(self):
        self.grabber.kicker_percent = self.target

    def isFinished(self) -> bool:
        return True