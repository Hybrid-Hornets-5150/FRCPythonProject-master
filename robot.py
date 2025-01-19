import wpilib
from wpilib import drive, SmartDashboard
import magicbot
from commands2 import *
from commands2.button import CommandXboxController
from components import *
from rev import SparkMax, SparkLowLevel


class MyRobot(magicbot.MagicRobot):
    robotController: CommandXboxController

    swerve_mod_fl: SwerveModule

    def createObjects(self):
        # Create motors and stuff here
        self.robotController = CommandXboxController(0)
        self.bindButtons()

        self.swerve_mod_fl_swerve_motor = SparkMax(5, SparkLowLevel.MotorType.kBrushless)
        self.swerve_mod_fl_drive_motor = SparkMax(3, SparkLowLevel.MotorType.kBrushless)
        self.swerve_mod_fl_swerve_pid = MotorConfig(0.025, 1e-7, 0.0001)
        self.swerve_mod_fl_drive_pid = MotorConfig(3e-7, 6e-7, 1e-6)

    def autonomousInit(self):
        # Runs when auton starts
        pass

    def teleopInit(self):
        # Called when teleop starts; optional
        pass

    def teleopPeriodic(self):
        # Called every 20ms when teleop runs
        pass

    def robotPeriodic(self):
        # Called every 20ms in each run mode.
        super().robotPeriodic()
        CommandScheduler.getInstance().run()
        SmartDashboard.putNumber("Left Joystick", self.robotController.getLeftX())
        SmartDashboard.putNumber("Right Joystick", self.robotController.getRightX())
        self.swerve_mod_fl.set_swerve_rotations(self.robotController.getLeftY() * 10)

    def bindButtons(self):
        self.robotController.a().onTrue(
            InstantCommand(lambda: SmartDashboard.putBoolean("A button", True)).ignoringDisable(True))
        self.robotController.a().onFalse(
            InstantCommand(lambda: SmartDashboard.putBoolean("A button", False)).ignoringDisable(True))
