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
        self.driveTrain = DriveTrain()

    def autonomousInit(self):
        # Runs when auton starts
        pass

    def teleopInit(self):
        # Called when teleop starts; optional
        self.driveTrain.on_enable()

    def teleopPeriodic(self):
        # Called every 20ms when teleop runs
        lx = self.robotController.getLeftX()
        ly = self.robotController.getLeftY()
        if abs(lx) < 0.1:
            lx = 0
        if abs(ly) < 0.1:
            ly = 0
        self.driveTrain.set_velocity(lx, ly)
        rotateSpeed = 0
        rotateSpeed -= self.robotController.getLeftTriggerAxis()
        rotateSpeed += self.robotController.getRightTriggerAxis()
        rotateSpeed /= 2
        self.driveTrain.set_rotation(rotateSpeed)
        if lx == 0 and ly == 0 and rotateSpeed == 0:
            self.driveTrain.stop()
        self.driveTrain.teleop()

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

    def disabledInit(self):
        self.driveTrain.on_disable()

