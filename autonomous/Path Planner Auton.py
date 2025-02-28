from commands2 import Command, CommandScheduler
from magicbot import AutonomousStateMachine
from pathplannerlib.auto import AutoBuilder, CommandUtil, PathPlannerAuto
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.trajectory import PathPlannerTrajectory
from wpilib import SmartDashboard, SendableChooser, Field2d


class PathAuto:
    MODE_NAME = "Path Finder Auton"
    DEFAULT = True
    DISABLED = False

    autoChooser: SendableChooser
    field: Field2d

    def __init__(self):
        self.pathCommand = None

    def setup(self):
        self.pathCommand: PathPlannerAuto = self.autoChooser.getSelected()

        PathPlannerLogging.setLogCurrentPoseCallback(lambda pose: self.field.setRobotPose(pose))
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self.field.getObject("target pose").setPose(pose))
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self.field.getObject("path").setPoses(poses))

    def on_enable(self, *args):
        self.pathCommand: PathPlannerAuto =  self.autoChooser.getSelected()
        CommandScheduler.getInstance().schedule(self.pathCommand)

    def on_disable(self, *args):
        if not self.pathCommand.isFinished():
            CommandScheduler.getInstance().cancel(self.pathCommand)

    def on_iteration(self, *args):
        pass