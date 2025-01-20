import rev
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase, SparkLowLevel, SparkClosedLoopController, \
    ClosedLoopConfig

from helpers import Component
from collections import namedtuple
from constants import *

MotorConfig = namedtuple("MotorConfig", ["kP", "kI", "kD"])

class SwerveModule(Component):
    swerve_motor: SparkMax
    drive_motor: SparkMax

    _swerve_controller: SparkClosedLoopController
    _drive_controller: SparkClosedLoopController

    def __init__(self, swerve_can_id, drive_can_id):
        self.swerve_motor = SparkMax(swerve_can_id, SparkLowLevel.MotorType.kBrushless)
        self.drive_motor = SparkMax(drive_can_id, SparkLowLevel.MotorType.kBrushless)

        self._swerve_controller = self.swerve_motor.getClosedLoopController()
        self._drive_controller = self.drive_motor.getClosedLoopController()

        self.setup()

    def execute(self):
        pass

    def setup(self):
        swerve_motor_config = SparkMaxConfig()
        swerve_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
        swerve_motor_config.closedLoop.pid(SwervePID.kP, SwervePID.kI, SwervePID.kD,
                                           rev.ClosedLoopSlot.kSlot0)
        self.swerve_motor.configure(swerve_motor_config, SparkBase.ResetMode.kResetSafeParameters,
                                    SparkBase.PersistMode.kPersistParameters)

        drive_motor_config = SparkMaxConfig()
        drive_motor_config.apply(swerve_motor_config).setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        # closedLoop is the accessor for the closed loop controller configuration
        drive_motor_config.closedLoop.pid(DrivePID.kP, DrivePID.kI, DrivePID.kD, rev.ClosedLoopSlot.kSlot0)
        self.drive_motor.configure(drive_motor_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)


    def set_swerve_rotations(self, rot):
        self._swerve_controller.setReference(rot, SparkLowLevel.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)

    def set_drive_speed(self, speed):
        self._drive_controller.setReference(speed, SparkLowLevel.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def on_disable(self):
        self.swerve_motor.disable()
        self.drive_motor.disable()
