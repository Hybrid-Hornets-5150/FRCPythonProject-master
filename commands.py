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


class AddArmAngle(Command):
    """Continuous command, doesn't work like it"""

    def __init__(self, degrees, arm):
        super().__init__()
        self.degrees = degrees
        self.arm = arm
        self.addRequirements(self.arm)
        self.running = True

    def initialize(self):
        self.running = True

    def execute(self):
        self.arm.angle_setpoint += self.degrees
        self.arm.arm_angle = self.arm.angle_setpoint

    def isFinished(self) -> bool:
        return self.running

    def cancel(self) -> None:
        self.running = False

class SetGyroRed(Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.addRequirements(drivetrain)
        self.drivetrain = drivetrain

    def initialize(self):
        self.drivetrain.gyro.set_yaw(0)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True


class SetGyroBlue(Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.addRequirements(drivetrain)
        self.drivetrain = drivetrain

    def initialize(self):
        self.drivetrain.gyro.set_yaw(180)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True