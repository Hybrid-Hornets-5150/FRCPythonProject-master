from wpilib import PWMTalonFX
from constants import *
from helpers import Component
from phoenix6 import hardware, controls, configs

class Lift(Component):
    def __init__(self):
        self.motor = hardware.TalonFX(CanIDs.LiftTalon)
        slot0_configs = configs.Slot0Configs()
        slot0_configs.k_p = 2.4
        slot0_configs.k_i = 0.01
        slot0_configs.k_d = 0
        self.motor.configurator.apply(slot0_configs)
        self.request = controls.PositionVoltage(0).with_slot(0)

    def set_rotation(self, rotations):
        self.motor.set_control(self.request.with_position(rotations * liftGearRatio))

    def on_disable(self):
        self.set_rotation(0)

    def on_enable(self):
        pass


class Arm(Component):
    def __init__(self):
        self.extension_motor = hardware.TalonFX(CanIDs.ExtenderTalon)
        self.rotation_motor = hardware.TalonFX(CanIDs.ArmTalon)
        self.intake_motor = hardware.TalonFX(CanIDs.IntakeLeaderTalon)
        self.intake_follower_motor = hardware.TalonFX(CanIDs.IntakeFollowerTalon)

        self.intake_follower_motor.set_control(controls.Follower(self.intake_motor.device_id, True))

        slot0_configs = configs.Slot0Configs()
        slot0_configs.k_p = 2.4
        slot0_configs.k_i = 0.01
        slot0_configs.k_d = 0
        self.extension_motor.configurator.apply(slot0_configs)
        self.extension_request = controls.PositionVoltage(0).with_slot(0)

        slot0_configs = configs.Slot0Configs()
        slot0_configs.k_p = 2.4
        slot0_configs.k_i = 0.01
        slot0_configs.k_d = 0
        self.rotation_motor.configurator.apply(slot0_configs)
        self.rotation_request = controls.PositionVoltage(0).with_slot(0)

    def set_intake_voltage(self, volts):
        self.intake_motor.setVoltage(volts)

    def set_arm_rotation(self, rotations):
        self.rotation_motor.set_control(self.rotation_request.with_position(rotations * armRotationGearRatio))

    def set_arm_extension(self, inches):
        self.extension_motor.set_control(self.extension_request.with_position(inches * armExtensionInchesPerRev))