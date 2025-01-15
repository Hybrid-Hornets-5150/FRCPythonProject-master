from rev import SparkMax
from helpers import Component


class SwerveModule(Component):
    swerve_motor: SparkMax
    drive_motor: SparkMax

    def on_disable(self):
        self.swerve_motor.disable()
        self.drive_motor.disable()
