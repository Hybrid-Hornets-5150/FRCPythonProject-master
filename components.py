import rev
from rev import SparkMax, SparkClosedLoopController, SparkLowLevel, SparkMaxConfig, SparkBase, MAXMotionConfig, \
    SparkBaseConfig
from helpers import Component
from collections import namedtuple

PIDConfig = namedtuple("PIDConfig", ["kP", "kI", "kD"])

class SwerveModule(Component):
    swerve_motor: SparkMax
    drive_motor: SparkMax

    swerve_controller: SparkClosedLoopController
    drive_controller: SparkClosedLoopController

    def __init__(self, swerve_kv: int, drive_kv: int):
        super().__init__()
        self.swerve_kV = swerve_kv
        self.drive_kV = drive_kv
        self.tuning_mode = False

    def setup(self):
        swerve_motor_config = SparkMaxConfig()
        swerve_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
        self.swerve_motor.configure(swerve_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        drive_motor_config = SparkMaxConfig()
        drive_motor_config.apply(swerve_motor_config).setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        self.drive_motor.configure(drive_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self.swerve_controller = self.swerve_motor.getClosedLoopController()
        self.drive_controller = self.drive_motor.getClosedLoopController()

    def set_swerve_rotations(self, rot):
        self.swerve_controller.setReference(rot, SparkLowLevel.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0, 1 / self.swerve_kV)

    def set_drive_speed(self, speed):
        self.drive_controller.setReference(speed, SparkLowLevel.ControlType.kMAXMotionVelocityControl, rev.ClosedLoopSlot.kSlot0, 1 / self.drive_kV)

    def on_disable(self):
        self.swerve_motor.disable()
        self.drive_motor.disable()
