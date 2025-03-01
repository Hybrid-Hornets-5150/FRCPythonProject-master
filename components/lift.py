import wpimath.units
from commands2 import Subsystem
from phoenix6 import hardware, controls, configs
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBase, SparkBaseConfig
from wpilib import SmartDashboard

from constants import *
from helpers import Component


class Lift(Component):
    max_height:wpimath.units.inches = 25+(5/8)

    def __init__(self):
        self.motor = hardware.TalonFX(CanIDs.LiftTalon)

        # region Controller
        # Controller configs
        talon_configs = configs.TalonFXConfiguration()
        slot0_configs = talon_configs.slot0
        slot0_configs.k_s = 0.105
        slot0_configs.k_v = 0.35
        slot0_configs.k_a = 0.03
        slot0_configs.k_g = 0.125

        slot0_configs.k_p = 5
        slot0_configs.k_i = 0
        slot0_configs.k_d = 0

        # Controller magic motion configs
        motion_magic_configs = talon_configs.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = 90
        motion_magic_configs.motion_magic_acceleration = 50
        motion_magic_configs.motion_magic_jerk = 200

        output_config = talon_configs.motor_output
        output_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        output_config.neutral_mode = NeutralModeValue.BRAKE

        limit_config = talon_configs.software_limit_switch
        limit_config.forward_soft_limit_enable = True
        limit_config.reverse_soft_limit_enable = True
        limit_config.forward_soft_limit_threshold = 140
        limit_config.reverse_soft_limit_threshold = 5

        # Configure controller
        self.motor.configurator.apply(talon_configs)
        self.request = controls.MotionMagicVoltage(0)
        self.vel_request = controls.VoltageOut(0)
        # endregion

    def set_rotation(self, rotations):
        self.motor.set_control(self.request.with_position(rotations * liftGearRatio))

    def set_height(self, height:wpimath.units.inches):
        scalar = 5.05
        self.motor.set_control(self.request.with_position(height*scalar))

    def set_voltage(self, volts):
        self.motor.set_control(self.vel_request.with_output(volts))

    def get_height(self):
        return self.motor.get_position().value/5.05

    def drive_percent(self, percent):
        self.motor.set(percent)

    def on_disable(self):
        self.set_rotation(0)

    def on_enable(self):
        #self.set_height(5)
        pass

    def periodic(self):
        pass


class Grabber(Subsystem):
    def __init__(self):
        super().__init__()
        # Sparks
        self.intake_motor = SparkMax(CanIDs.IntakeLeaderSpark, SparkLowLevel.MotorType.kBrushless)
        self.intake_follower_motor = SparkMax(CanIDs.IntakeFollowerSpark, SparkLowLevel.MotorType.kBrushless)
        self.kicker_motor = SparkMax(CanIDs.KickerSpark, SparkLowLevel.MotorType.kBrushless)

        # region SparkConfigs
        leader_config = SparkMaxConfig().inverted(True)
        self.intake_motor.configure(leader_config, SparkBase.ResetMode.kResetSafeParameters,
                                    SparkBase.PersistMode.kPersistParameters)

        follower_config = SparkMaxConfig().follow(CanIDs.IntakeLeaderSpark, True)
        self.intake_follower_motor.configure(follower_config, SparkBase.ResetMode.kResetSafeParameters,
                                             SparkBase.PersistMode.kPersistParameters)

        kicker_config = SparkMaxConfig().setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.kicker_motor.configure(kicker_config, SparkBase.ResetMode.kResetSafeParameters,
                                             SparkBase.PersistMode.kPersistParameters)
        # endregion

    @property
    def intake_percent(self):
        return self.intake_motor.get()

    @intake_percent.setter
    def intake_percent(self, percent: wpimath.units.percent):
        if abs(percent) > 1:
            percent /= 100
        self.intake_motor.set(percent)

    @property
    def kicker_percent(self):
        return self.kicker_motor.get()

    @kicker_percent.setter
    def kicker_percent(self, percent):
        if abs(percent) > 1:
            percent /= 100
        self.kicker_motor.set(percent)


class Arm(Subsystem):
    def __init__(self):
        super().__init__()
        # Talons
        self.extension_motor = hardware.TalonFX(CanIDs.ExtenderTalon)
        self.rotation_motor = hardware.TalonFX(CanIDs.ArmTalon)

        #region ArmController
        # Arm controller configs
        talon_configs = configs.TalonFXConfiguration()
        slot0_configs = talon_configs.slot0
        slot0_configs.k_s = 0.25
        slot0_configs.k_v = 0.3
        slot0_configs.k_a = 0.01

        slot0_configs.k_p = 15
        slot0_configs.k_i = 0
        slot0_configs.k_d = 0.001

        # Arm controller magic motion configs
        motion_magic_configs = talon_configs.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = 30
        motion_magic_configs.motion_magic_acceleration = 50
        motion_magic_configs.motion_magic_jerk = 120

        output_config = talon_configs.motor_output
        output_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        output_config.neutral_mode = NeutralModeValue.BRAKE

        limit_config = talon_configs.software_limit_switch
        limit_config.forward_soft_limit_enable = True
        limit_config.reverse_soft_limit_enable = True
        limit_config.forward_soft_limit_threshold = 26
        limit_config.reverse_soft_limit_threshold = -0.5
        self.angle_limits = [-0.5, 26]

        # Configure arm controller
        self.rotation_motor.configurator.apply(talon_configs)
        self.rotation_request = controls.MotionMagicVoltage(0)
        self.rotate_volts_request = controls.VoltageOut(0)
        #endregion

        #region ExtensionController
        talon_configs = configs.TalonFXConfiguration()
        slot0_configs = talon_configs.slot0
        slot0_configs.k_s = 0.16
        slot0_configs.k_v = 0.1
        slot0_configs.k_a = 0.01

        slot0_configs.k_p = 5
        slot0_configs.k_i = 0
        slot0_configs.k_d = 0

        # Extension controller magic motion configs
        motion_magic_configs = talon_configs.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = 400
        motion_magic_configs.motion_magic_acceleration = 150
        motion_magic_configs.motion_magic_jerk = 1000

        output_config = talon_configs.motor_output
        output_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        output_config.neutral_mode = NeutralModeValue.COAST

        limit_config = talon_configs.software_limit_switch
        limit_config.forward_soft_limit_enable = True
        limit_config.reverse_soft_limit_enable = True
        limit_config.forward_soft_limit_threshold = 280
        limit_config.reverse_soft_limit_threshold = 0.1

        # Configure extension controller
        self.extension_motor.configurator.apply(talon_configs)
        self.extension_request = controls.MotionMagicVoltage(0)
        #endregion

        self.angle_setpoint = self.arm_angle
        self.extension_setpoint = self.extension
        self.zero_angle()

    def zero_angle(self):
        self.rotation_motor.set_position(0)

    @property
    def extension(self):
        return (self.extension_motor.get_position().value * 8 / 200)

    @extension.setter
    def extension(self, inches: wpimath.units.inches):
        self.extension_setpoint = inches*200/8
        self.extension_motor.set_control(self.extension_request.with_position(self.extension_setpoint))

    @property
    def arm_angle(self) -> wpimath.units.degrees:
        return (self.rotation_motor.get_position().value - 16.1) / .163

    @arm_angle.setter
    def arm_angle(self, degrees: wpimath.units.degrees):
        self.extension = 0
        self.angle_setpoint = degrees
        rotations = 0.163 * degrees + 16.1

        if rotations < self.angle_limits[0]:
            rotations = self.angle_limits[0]
            self.angle_setpoint = (rotations - 16.1) / 0.163
        elif rotations > self.angle_limits[1]:
            rotations = self.angle_limits[1]
            self.angle_setpoint = (rotations - 16.1) / 0.163

        feedforward = 0.5
        self.rotation_motor.set_control(self.rotation_request.with_position(rotations).with_feed_forward(feedforward))

    @property
    def arm_voltage(self) -> wpimath.units.degrees:
        return (self.rotation_motor.get_position().value - 16.1) / .163

    @arm_voltage.setter
    def arm_voltage(self, volts: wpimath.units.volts):
        self.rotation_motor.set_control(self.rotate_volts_request.with_output(volts))

    def increment_up(self):
        self.arm_angle = self.angle_setpoint + 5

    def increment_down(self):
        self.arm_angle = self.angle_setpoint - 5

    def set_arm_extension(self, inches):
        self.extension_setpoint = inches
        self.extension_motor.set_control(self.extension_request.with_position(self.extension_setpoint * armExtensionInchesPerRev))

    def at_extension(self):
        return abs(self.extension - self.extension_setpoint) < 1

