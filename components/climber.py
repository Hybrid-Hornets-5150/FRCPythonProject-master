import wpimath.units
from commands2 import Subsystem
from phoenix6 import hardware, configs
from phoenix6.signals import InvertedValue, NeutralModeValue

from constants import CanIDs


class Climber(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = hardware.TalonFX(CanIDs.ClimberTalon)

        talon_configs = configs.TalonFXConfiguration()
        motor_config = talon_configs.motor_output
        motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        motor_config.neutral_mode = NeutralModeValue.BRAKE

        self.motor.configurator.apply(motor_config)

    @property
    def percent_output(self):
        return self.motor.get()

    @percent_output.setter
    def percent_output(self, percent: wpimath.units.percent):
        if percent > 1:
            percent /= 100

        self.motor.set(percent)


