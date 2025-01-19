import rev
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase, SparkLowLevel, SparkClosedLoopController, \
    ClosedLoopConfig

from helpers import Component
from collections import namedtuple

MotorConfig = namedtuple("MotorConfig", ["kP", "kI", "kD"])

class SwerveModule(Component):
    swerve_motor: SparkMax
    drive_motor: SparkMax
    swerve_pid: MotorConfig
    drive_pid: MotorConfig

    _swerve_controller: SparkClosedLoopController
    _drive_controller: SparkClosedLoopController

    def execute(self):
        pass

    def setup(self):
        swerve_motor_config = SparkMaxConfig()
        swerve_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
        swerve_motor_config.closedLoop.pid(self.swerve_pid.kP, self.swerve_pid.kI, self.swerve_pid.kD,
                                           rev.ClosedLoopSlot.kSlot0)
        self.swerve_motor.configure(swerve_motor_config, SparkBase.ResetMode.kResetSafeParameters,
                                    SparkBase.PersistMode.kPersistParameters)

        drive_motor_config = SparkMaxConfig()
        drive_motor_config.apply(swerve_motor_config).setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        # closedLoop is the accessor for the closed loop controller configuration
        drive_motor_config.closedLoop.pid(self.drive_pid.kP, self.drive_pid.kI,
                                          self.drive_pid.kD, rev.ClosedLoopSlot.kSlot0)
        self.drive_motor.configure(drive_motor_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        self._swerve_controller = self.swerve_motor.getClosedLoopController()
        self._drive_controller = self.drive_motor.getClosedLoopController()


    def set_swerve_rotations(self, rot):
        self._swerve_controller.setReference(rot, SparkLowLevel.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)
    def set_drive_speed(self, speed):
        self._drive_controller.setReference(speed, SparkLowLevel.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
    def on_disable(self):
        self.swerve_motor.disable()
        self.drive_motor.disable()
