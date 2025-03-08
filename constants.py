import math

from wpimath import units

#diameter of the wheel is 3 inches
class DrivePID:
    kP = 3e-3
    kI = 0
    kD = 0
    Ratio = 3.56
    kFF = 0
    Circumference = 3 * math.pi


class SwervePID:
    kP = 3
    kI = 0.00001
    kD = 3
    Ratio = 46.42


class IntakePID:
    kP = 0.00001
    kI = 0
    kD = 0
    kF = 0.0001

class CanIDs:
    DriveSparkNE = 1
    DriveSparkNW = 2
    DriveSparkSE = 3

    DriveSparkSW = 4
    SwerveSparkNE = 5
    SwerveSparkNW = 6
    SwerveSparkSE = 7
    SwerveSparkSW = 8
    LiftTalon = 9
    ExtenderTalon = 10
    IntakeLeaderSpark = 11
    IntakeFollowerSpark = 12
    ArmTalon = 13
    Gyro = 14
    ClimberTalon = 15
    KickerSpark = 16
    PDP = 17

class DigitalPins:
    SwerveAbsSW = 0
    SwerveAbsNW = 1
    SwerveAbsSE = 8
    SwerveAbsNE = 9

#TODO: determine constants
centerPoint = (units.feetToMeters(22.5 / 24), units.feetToMeters(30.25 / 24))
liftGearRatio = 1
armRotationGearRatio = 1
armExtensionInchesPerRev = 1
teleopSpeedScaling = 1.75
autonSpeedScaling = 1