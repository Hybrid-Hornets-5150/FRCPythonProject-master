import commands2
import rev
import wpilib
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import PathPlannerAuto
from phoenix6 import SignalLogger
from wpilib import drive, SmartDashboard, SendableChooser, Field2d
import magicbot
from commands2 import *
from commands2.button import CommandXboxController

import commands
from commands import LiftVelocity, SetArmVoltage, RunIntake, SetKickerPercent
from components.chassis import *
from rev import SparkMax, SparkLowLevel

from components.climber import Climber
from components.lift import Lift, Arm, Grabber


class MyRobot(magicbot.MagicRobot):
    # Controllers
    driveController: CommandXboxController
    operatorController: CommandXboxController

    #Other
    autoChooser: SendableChooser
    field: Field2d

    # Subsystems
    driveTrain: DriveTrain
    lift: Lift
    arm: Arm
    climber: Climber
    grabber: Grabber

    def __init__(self):
        super().__init__()
        self.estop: bool = False
        self.lift_setpoints = [1, 24]
        self.lift_setpoint_index = 0
        self.arm_setpoints = [-98, 0, 49.2]
        self.arm_setpoint_index = 0
        self.lift_manual = False
        self.arm_manual = False
        self.extension_setpoint = 0

    def createObjects(self):
        # Create motors and stuff here
        self.driveController = CommandXboxController(0)
        self.operatorController = CommandXboxController(1)

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        self.driveTrain = DriveTrain(self.field)

        self.lift = Lift()

        self.arm = Arm()

        self.climber = Climber()

        self.grabber = Grabber()

        self.autoChooser = AutoBuilder.buildAutoChooser("Test Auton")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # self.robotController.povUp().onTrue(commands.SetArmAngle(0, self.arm))
        # self.robotController.povDown().onTrue(commands.SetArmVoltage(0, self.arm))
        # self.robotController.povLeft().onTrue(commands.LiftTo(10, self.lift))
        # self.robotController.povRight().onTrue(commands.LiftTo(0, self.lift))
        # self.robotController.x().onTrue(commands.RunIntake(0, self.grabber))
        # self.robotController.b().onTrue(commands.RunIntake(50, self.grabber))
        # self.robotController.a().onTrue(commands.SetExtension(0, self.arm))
        # self.robotController.y().onTrue(commands.SetExtension(5, self.arm))


        #region operatorControls
        self.operatorController.povUp().onTrue(
            ConditionalCommand(
                LiftVelocity(5, self.lift),
                InstantCommand(self.increase_lift_setpoint),
                self.is_lift_manual
            )
        ).onFalse(
            LiftVelocity(0, self.lift)
            .onlyIf(self.is_lift_manual)
        )

        self.operatorController.povDown().onTrue(
            ConditionalCommand(
                LiftVelocity(-2, self.lift),
                InstantCommand(self.decrease_lift_setpoint),
                self.is_lift_manual
            )
        ).onFalse(
            LiftVelocity(0, self.lift)
            .onlyIf(self.is_lift_manual)
        )

        self.operatorController.back().onTrue(
            InstantCommand(self.toggle_lift_manual)
        )

        self.operatorController.povRight().onTrue(
            InstantCommand(self.goto_zero_extension).andThen(WaitUntilCommand(self.at_zero_extension))
            .andThen(
                ConditionalCommand(
                    SetArmVoltage(2, self.arm),
                    InstantCommand(self.increase_arm_setpoint),
                    self.is_arm_manual
                )
            )
        ).onFalse(
            SetArmVoltage(0, self.arm)
            .onlyIf(self.is_arm_manual)
        )

        self.operatorController.povLeft().onTrue(
            InstantCommand(self.goto_zero_extension).andThen(WaitUntilCommand(self.at_zero_extension))
            .andThen(
                ConditionalCommand(
                    SetArmVoltage(-0.5, self.arm),
                    InstantCommand(self.decrease_arm_setpoint),
                    self.is_arm_manual
                )
            )
        ).onFalse(
            SetArmVoltage(0, self.arm)
            .onlyIf(self.is_arm_manual)
        )

        self.operatorController.start().onTrue(
            InstantCommand(self.toggle_arm_manual)
        )

        self.operatorController.b().onTrue(
            RunIntake(0.5, self.grabber)
        ).onFalse(
            RunIntake(0, self.grabber)
        )

        self.operatorController.y().onTrue(
            SetKickerPercent(0.5, self.grabber)
        ).onFalse(
            SetKickerPercent(0, self.grabber)
        )

        self.operatorController.a().onTrue(
            SetKickerPercent(-0.5, self.grabber)
        ).onFalse(
            SetKickerPercent(0, self.grabber)
        )

        self.operatorController.x().onTrue(
            InstantCommand(print(self.driveTrain.represent_pose()))
        )

        #endregion

    def at_zero_extension(self):
        return self.arm.extension < 1

    def goto_zero_extension(self):
        self.extension_setpoint = 0

    def toggle_arm_manual(self):
        self.arm_manual = not self.arm_manual

    def is_arm_manual(self):
        return self.arm_manual

    def is_lift_manual(self):
        return self.lift_manual

    def increase_arm_setpoint(self):
        self.arm_setpoint_index += 1
        if self.arm_setpoint_index >= len(self.arm_setpoints):
            self.arm_setpoint_index = len(self.arm_setpoints) - 1

        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]


    def decrease_arm_setpoint(self):
        self.arm_setpoint_index -= 1
        if self.arm_setpoint_index <= 0:
            self.arm_setpoint_index = 0

        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def toggle_lift_manual(self):
        self.lift_manual = not self.lift_manual

    def increase_lift_setpoint(self):
        self.lift_setpoint_index += 1
        if self.lift_setpoint_index >= len(self.lift_setpoints):
            self.lift_setpoint_index = len(self.lift_setpoints) - 1

        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])

    def decrease_lift_setpoint(self):
        self.lift_setpoint_index -= 1
        if self.lift_setpoint_index <= 0:
            self.lift_setpoint_index = 0

        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])

    def e_stop(self):
        self.estop = True

    def autonomousInit(self):
        super().autonomousInit()
        # Runs when auton starts
        pass

    def teleopInit(self):
        # Called when teleop starts; optional
        self.lift.set_height(1)
        self.driveTrain.on_enable()
        self.lift.on_enable()

    def teleopPeriodic(self):
        # Called every 20ms when teleop runs

        #region operatorControl

        lt = self.operatorController.getLeftTriggerAxis()
        rt = self.operatorController.getRightTriggerAxis()
        if lt < 0.1:
            lt = 0

        if rt < 0.1:
            rt = 0

        travel = 0.4*8*(rt - lt)*0.01
        self.extension_setpoint += travel
        if self.extension_setpoint < 0:
            self.extension_setpoint = 0
        if self.extension_setpoint > 8:
            self.extension_setpoint = 8

        self.arm.extension = self.extension_setpoint

        #endregion

        if self.estop:
            return
        lx = self.driveController.getLeftX()
        ly = self.driveController.getLeftY() * (-1)

        rotation = self.driveController.getRightX()
        climb = self.driveController.getRightY()

        if abs(lx) < 0.2:
            lx = 0
        if abs(ly) < 0.2:
            ly = 0
        if abs(rotation) < 0.2:
            rotation = 0
        if abs(climb) < 0.2:
            climb = 0

        rotation = rotation*2*math.pi/4 * (-1)

        if lx != 0:
            angle = math.atan(abs(ly)/abs(lx))
        else:
            if ly >= 0:
                angle = math.pi/2
            else:
                angle = math.pi/2*3

        angle = angle / math.pi * 180

        if lx < 0 < ly:
            angle = 180 - angle
        elif lx < 0 and ly < 0:
            angle = angle + 180
        elif lx > 0 > ly:
            angle = 360 - angle

        SmartDashboard.putNumber("angle", angle)

        SmartDashboard.putNumberArray("Controller L", [lx, ly])
        speed = ChassisSpeeds(vx=lx, vy=ly, omega=rotation)
        self.driveTrain.driveRobotRelative(speed)
        #self.climber.percent_output = climb

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def robotPeriodic(self):
        # Called every 20ms in each run mode.
        super().robotPeriodic()
        SmartDashboard.putBoolean("ESTOP", self.estop)
        if self.estop:
            return

        lx = self.driveController.getLeftX()
        ly = self.driveController.getLeftY() * (-1)

        SmartDashboard.putNumberArray("Controller L", [lx, ly])
        CommandScheduler.getInstance().run()

    def disabledInit(self):
        self.driveTrain.on_disable()

