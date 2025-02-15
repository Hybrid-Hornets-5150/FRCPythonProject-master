from wpimath import units

class DrivePID:
    kP = 3e-7
    kI = 6e-7
    kD = 1e-6
    Ratio = 3.56

class SwervePID:
    kP = 0.025
    kI = 1e-7
    kD = 0.0001
    Ratio = 46.42

class CanIDs:
    DriveSparkNE = 4
    DriveSparkNW = 3
    DriveSparkSE = 9
    DriveSparkSW = 1
    SwerveSparkNE = 5
    SwerveSparkNW = 7
    SwerveSparkSE = 8
    SwerveSparkSW = 6
    LiftTalon = 0
    ExtenderTalon = 0
    IntakeLeaderTalon = 0
    IntakeFollowerTalon = 0
    ArmTalon = 0

centerPoint = (units.feetToMeters(22.5 / 24), units.feetToMeters(30.25 / 24))
liftGearRatio = 1
armRotationGearRatio = 1
armExtensionInchesPerRev = 1