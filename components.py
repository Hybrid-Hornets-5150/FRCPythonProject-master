import rev
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase, SparkLowLevel, SparkClosedLoopController
from wpilib import SmartDashboard

from helpers import Component
from collections import namedtuple

MotorConfig = namedtuple("MotorConfig", ["kP", "kI", "kD"])
swerveRotationsPerRev = 5.5

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


class PIDDemo(Component):
    position_spark: SparkMax

    _position_controller: SparkClosedLoopController
    _setpoint: int

    position_target = 0
    PID = [0, 0, 0]
    # Current error, error rate of change, total error
    errors = {"e": 0, "e_roc": 0, "total_e": 0}
    actions = [0, 0, 0]

    def setup(self):
        self._position_controller = self.position_spark.getClosedLoopController()

    def step(self):
        # Calculate error to be acted on by individual PID elements.
        e = self._setpoint - self.position_target
        self.errors["e_roc"] = e - self.errors["e"]
        self.errors["e"] = self._setpoint - self.position_target
        self.errors["total_e"] += e

        self.actions = [0, 0, 0]

        # Proportional term
        self.actions[0] += self.PID[0] * self.errors["e"]

        # Integral term
        self.actions[1] += self.PID[1] * self.errors["total_e"]

        # Derivative term
        self.actions[2] += self.PID[2] * self.errors["e_roc"]

        self._position_controller.setReference(self.position_target,
                                               SparkLowLevel.ControlType.kPosition,
                                               rev.ClosedLoopSlot.kSlot0)
    def set_setpoint(self, sp):
        self._setpoint = sp

    def execute(self):
        pass
