import rev
import wpilib
from wpilib import drive, SmartDashboard
import magicbot
from commands2 import *
from commands2.button import CommandXboxController
from components.chassis import *
from rev import SparkMax, SparkLowLevel


class MyRobot(magicbot.MagicRobot):
    # Controllers
    robotController: CommandXboxController

    # Components

    def createObjects(self):
        # Create motors and stuff here
        self.robotController = CommandXboxController(0)
        self.swerve = SwerveModule(5, 3)

    def autonomousInit(self):
        # Runs when auton starts
        pass

    def teleopInit(self):
        # Called when teleop starts; optional
        pass

    def teleopPeriodic(self):
        # Called every 20ms when teleop runs
        self.swerve.set_swerve_rotations(self.robotController.getLeftX()*10)
        self.swerve.set_drive_speed(self.robotController.getRightY()*2000)

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def robotPeriodic(self):
        # Called every 20ms in each run mode.
        super().robotPeriodic()
        CommandScheduler.getInstance().run()
        SmartDashboard.putNumber("Left Joystick", self.robotController.getLeftX())
        SmartDashboard.putNumber("Right Joystick", self.robotController.getRightX())


