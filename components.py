import rev
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase, SparkLowLevel, SparkClosedLoopController

from helpers import Component
from collections import namedtuple

MotorConfig = namedtuple("MotorConfig", ["kP", "kI", "kD"])

class SwerveModule(Component):
    swerve_motor: SparkMax
    drive_motor: SparkMax

    _swerve_controller: SparkClosedLoopController
    _drive_controller: SparkClosedLoopController
    def execute(self):
        pass
    def setup(self):
        swerve_motor_config = SparkMaxConfig()
        swerve_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
        self.swerve_motor.configure(swerve_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        drive_motor_config = SparkMaxConfig()
        drive_motor_config.apply(swerve_motor_config).setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        self.drive_motor.configure(drive_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self._swerve_controller = self.swerve_motor.getClosedLoopController()
        self._drive_controller = self.drive_motor.getClosedLoopController()

    def set_swerve_rotations(self, rot):
        self._swerve_controller.setReference(rot, SparkLowLevel.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0, 1 / self.swerve_kV)
    def set_drive_speed(self, speed):
        self._drive_controller.setReference(speed, SparkLowLevel.ControlType.kMAXMotionVelocityControl, rev.ClosedLoopSlot.kSlot0, 1 / self.drive_kV)
    def on_disable(self):
        self.swerve_motor.disable()
        self.drive_motor.disable()
