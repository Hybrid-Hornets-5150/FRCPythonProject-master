import math
from pathlib import Path

import choreo
import magicbot
import wpimath.filter
from choreo import SwerveTrajectory
from commands2 import *
from commands2.button import CommandXboxController
from wpilib import SendableChooser, DataLogManager
from wpilib.shuffleboard import Shuffleboard
from wpimath._controls._controls.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Transform2d

from commands import LiftVelocity, RunIntake, SetKickerPercent, AddArmAngle, SetGyroRed, SetGyroBlue
from components.chassis import *
from components.climber import Climber
from components.lift import Lift, Arm, Grabber
from state_machines import IntakeCoral, ScoreCoral, ScoreCoralRight, IntakeHome, ScoreCoralLeft


class MyRobot(magicbot.MagicRobot):
    # Controllers
    driveController: CommandXboxController
    operatorController: CommandXboxController

    # Automatic robot controllers
    automaticController: HolonomicDriveController

    # Other
    field: Field2d
    camera: PhotonCamera
    trajectory: SwerveTrajectory
    traj_chooser: SendableChooser

    # Subsystems
    driveTrain: DriveTrain
    lift: Lift
    arm: Arm
    climber: Climber
    grabber: Grabber

    # State machines
    coral_intake_fsm: IntakeCoral
    coral_score_fsm: ScoreCoral
    coral_autoscore_right_fsm: ScoreCoralRight
    coral_autoscore_left_fsm: ScoreCoralLeft
    intake_home: IntakeHome

    def __init__(self):
        super().__init__()
        # Define simple variables here.
        self.estop: bool = False
        self.lift_setpoints = [2, 24]
        self.lift_setpoint_index = 0
        self.arm_setpoints = [-107, -75, -65, -33, 8, 55]
        self.arm_setpoint_index = 0
        self.lift_manual = False
        self.arm_manual = False
        self.manual_intake = False
        self.changing_angle_manual = False
        self.extension_setpoint = 0
        self.drivexlimiter = wpimath.filter.SlewRateLimiter(3)
        self.driveylimiter = wpimath.filter.SlewRateLimiter(3)
        self.driveanglelimiter = wpimath.filter.SlewRateLimiter(3)
        self.drive_mode = "field_oriented"
        self.speed_scalar = 1
        self.auto_aligning = False
        DriverStation.silenceJoystickConnectionWarning(True)

    def createObjects(self):
        SmartDashboard.putBoolean("Arm Angle In Manual", self.arm_manual)
        SmartDashboard.putBoolean("Lift Height In Manual", self.lift_manual)
        # SmartDashboard.putNumber("Desired X Velocity", 0)
        # SmartDashboard.putNumber("Desired Y Velocity", 0)
        # SmartDashboard.putNumber("Desired Rotational Velocity", 0)
        SmartDashboard.putBoolean("Drive Train in Slow Mode", False)


        self.automaticController = HolonomicDriveController(
            PIDController(2, 0, 0.1),
            PIDController(2, 0, 0.1),
            ProfiledPIDControllerRadians(2, 0, 0.1, TrapezoidProfileRadians.Constraints(3, 3))
        )

        self.traj_chooser = SendableChooser()
        self.traj_chooser.onChange(self.load_auton_trajectory)
        # Load choreo trajectory
        choreo_path = Path(choreo.getDeployDirectory()) / "choreo"
        for trajectory_name in choreo_path.iterdir():
            trajectory_name: Path
            if trajectory_name.suffix == ".traj":
                self.traj_chooser.addOption(trajectory_name.name, trajectory_name.stem)

        SmartDashboard.putData("Auton Trajectory", self.traj_chooser)

        try:
            self.trajectory = choreo.load_swerve_trajectory("test_auton_lower (one score)")
        except ValueError:
            pass

        # Create motors and stuff here
        # Create command based XBox controllers
        self.driveController = CommandXboxController(0)
        self.operatorController = CommandXboxController(1)

        # Used purely for visualization
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Initialize the robot vision system
        self.camera = PhotonCamera("Primary_Vision_Camera")

        # Initialize the robot subsystems
        self.driveTrain = DriveTrain(self.field, self.camera)
        self.lift = Lift()
        self.arm = Arm()
        self.climber = Climber()
        self.grabber = Grabber()

        SmartDashboard.putData("Set Gyro RED", SetGyroRed(self.driveTrain))
        SmartDashboard.putData("Set Gyro BLUE", SetGyroBlue(self.driveTrain))

        # region operatorControls
        self.operatorController.povUp().onTrue(
            ConditionalCommand(
                LiftVelocity(5, self.lift),
                InstantCommand(self.increase_lift_setpoint),
                self.is_lift_manual
            )
        ).onFalse(
            LiftVelocity(0, self.lift)
            .onlyIf(self.is_lift_manual)
        )

        self.operatorController.povDown().onTrue(
            ConditionalCommand(
                LiftVelocity(-2, self.lift),
                InstantCommand(self.decrease_lift_setpoint),
                self.is_lift_manual
            )
        ).onFalse(
            LiftVelocity(0, self.lift)
            .onlyIf(self.is_lift_manual)
        )

        self.operatorController.back().onTrue(
            InstantCommand(self.toggle_lift_manual)
        )

        add_angle_cmd = InstantCommand(self.set_manual_arm).andThen(
            AddArmAngle(1, self.arm).repeatedly().until(self.not_manual_arm))
        sub_angle_cmd = InstantCommand(self.set_manual_arm).andThen(
            AddArmAngle(-1, self.arm).repeatedly().until(self.not_manual_arm))

        self.operatorController.povRight().onTrue(

            ConditionalCommand(
                add_angle_cmd
                ,
                InstantCommand(self.increase_arm_setpoint),
                self.is_arm_manual
            )

        ).onFalse(
            InstantCommand(self.unset_manual_arm)
        )

        self.operatorController.povLeft().onTrue(
            ConditionalCommand(
                sub_angle_cmd
                ,
                InstantCommand(self.decrease_arm_setpoint),
                self.is_arm_manual
            )
        ).onFalse(
            InstantCommand(self.unset_manual_arm)
        )

        self.operatorController.start().onTrue(
            InstantCommand(self.toggle_arm_manual)
        )

        self.operatorController.a().onTrue(
            InstantCommand(
                self.l1_scoring_position
            )
        )

        self.operatorController.x().onTrue(
            InstantCommand(
                self.l2_scoring_position
            )
        )

        self.operatorController.y().onTrue(
            InstantCommand(
                self.l3_scoring_position
            )
        )

        self.operatorController.b().onTrue(
            InstantCommand(
                self.l4_scoring_position

            )
        )

        self.operatorController.leftBumper().onTrue(
            InstantCommand(
                lambda: self.arm.HAMMAR(-8)
            )
        ).onFalse(
            InstantCommand(
                lambda: self.arm.HAMMAR(0)
            )
        )

        self.operatorController.rightBumper().onTrue(
            InstantCommand(
                lambda: self.arm.HAMMAR(8)
            )
        ).onFalse(
            InstantCommand(
                lambda: self.arm.HAMMAR(0)
            )
        )

        # endregion

        # region driverControls

        self.driveController.y().onTrue(InstantCommand(self.toggle_driver_control_mode))
        self.driveController.a().onTrue(
            InstantCommand(self.set_fast_mode)
        ).onFalse(
            InstantCommand(self.unset_fast_mode)
        )

        self.driveController.rightBumper().onTrue(
            RunIntake(0.5, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", True)))
        ).onFalse(
            RunIntake(0, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", False)))
        )

        self.driveController.leftBumper().onTrue(
            RunIntake(-1.0, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", True)))
        ).onFalse(
            RunIntake(0, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", False)))
        )

        # endregion

    def l1_scoring_position(self):
        self.lift_setpoint_index = 0
        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])
        self.arm_setpoint_index = 3
        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def l2_scoring_position(self):
        self.lift_setpoint_index = 0
        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])
        self.arm_setpoint_index = 4
        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def l3_scoring_position(self):
        self.lift_setpoint_index = 0
        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])
        self.arm_setpoint_index = 5
        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def l4_scoring_position(self):
        self.lift_setpoint_index = 1
        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])
        self.arm_setpoint_index = 5
        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def load_auton_trajectory(self, new_trajectory):
        try:
            self.trajectory = choreo.load_swerve_trajectory(new_trajectory)
            pose = self.trajectory.get_initial_pose(DriverStation.getAlliance() == DriverStation.Alliance.kRed)
            self.field.setRobotPose(pose)
            self.driveTrain.resetPose(pose)
            self._automodes.modes["Trajectory"].trajectory = self.trajectory
        except ValueError:
            pass

    def set_fast_mode(self):
        self.speed_scalar = 1.75

    def unset_fast_mode(self):
        self.speed_scalar = 1

    def toggle_driver_control_mode(self):
        if self.drive_mode == "field_oriented":
            self.drive_mode = "robot_oriented"
        else:
            self.drive_mode = "field_oriented"

    def set_manual_arm(self):
        self.changing_angle_manual = True

    def unset_manual_arm(self):
        self.changing_angle_manual = False

    def not_manual_arm(self):
        return not self.changing_angle_manual

    def toggle_arm_manual(self):
        self.arm_manual = not self.arm_manual
        SmartDashboard.putBoolean("Arm Angle In Manual", self.arm_manual)

    def is_arm_manual(self):
        return self.arm_manual

    def is_lift_manual(self):
        return self.lift_manual

    def increase_arm_setpoint(self):
        self.arm_setpoint_index += 1
        if self.arm_setpoint_index >= len(self.arm_setpoints):
            self.arm_setpoint_index = len(self.arm_setpoints) - 1

        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def decrease_arm_setpoint(self):
        self.arm_setpoint_index -= 1
        if self.arm_setpoint_index <= 0:
            self.arm_setpoint_index = 0

        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def toggle_lift_manual(self):
        self.lift_manual = not self.lift_manual
        SmartDashboard.putBoolean("Lift Height In Manual", self.lift_manual)

    def increase_lift_setpoint(self):
        self.lift_setpoint_index += 1
        if self.lift_setpoint_index >= len(self.lift_setpoints):
            self.lift_setpoint_index = len(self.lift_setpoints) - 1

        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])

    def decrease_lift_setpoint(self):
        self.lift_setpoint_index -= 1
        if self.lift_setpoint_index <= 0:
            self.lift_setpoint_index = 0

        self.lift.set_height(self.lift_setpoints[self.lift_setpoint_index])

    def autonomousInit(self):
        super().autonomousInit()
        # Runs when auton starts
        self.arm_setpoint_index = 0
        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]

    def teleopInit(self):
        # Called when teleop starts; optional
        # CameraServer.startAutomaticCapture()
        self.lift.set_height(2)
        self.arm_setpoint_index = 1
        self.arm.arm_angle = self.arm_setpoints[self.arm_setpoint_index]
        self.driveTrain.on_enable()
        self.lift.on_enable()

        self.driveController.b().onTrue(
            InstantCommand(lambda: setattr(self, "auto_aligning", True))
        ).onFalse(
            InstantCommand(lambda: setattr(self, "auto_aligning", False))
        )

        self.driveController.x().whileTrue(
            RunCommand(self.intake_home.run)
        )

    def teleopPeriodic(self):
        # Called every 20ms when teleop runs

        #TODO: Testing code
        result = self.camera.getLatestResult()
        best_target = result.getBestTarget()
        if best_target:
            SmartDashboard.putNumber("Best Target", float(best_target.fiducialId))
            tx = best_target.getBestCameraToTarget()
            rotation = tx.rotation()
            SmartDashboard.putNumber("Camera Delta X", rotation.x_degrees)
            SmartDashboard.putNumber("Camera Delta Y", rotation.y_degrees)
            SmartDashboard.putNumber("Camera Delta Z", rotation.z_degrees)

        # region operatorControl

        lt = self.operatorController.getLeftTriggerAxis()
        rt = self.operatorController.getRightTriggerAxis()
        lt = MyRobot.deadband(lt, 0.1)
        rt = MyRobot.deadband(rt, 0.1)

        self.climber.percent_output = rt - lt

        # endregion

        # Currently unused
        if self.estop:
            return

        # region driverControl

        # Get the x speed. This needs to be inverted xbox controllers give a negative value when pushed forward.
        # We are also flipping the axes around since the +x coordinate is pointing away from the driver station.
        # region drive
        x_speed = (
                -self.drivexlimiter.calculate(
                    wpimath.applyDeadband(self.driveController.getLeftY(), 0.1)
                ) * kMaxSpeed * self.speed_scalar
        )

        y_speed = (
                -self.driveylimiter.calculate(
                    wpimath.applyDeadband(self.driveController.getLeftX(), 0.1)
                ) * kMaxSpeed * self.speed_scalar
        )

        rot_speed = (
                -self.driveanglelimiter.calculate(
                    wpimath.applyDeadband(self.driveController.getRightX(), 0.1)
                ) * kMaxSpeed * self.speed_scalar
        )
        if self.drive_mode == "field_oriented":
            rel = True
        else:
            rel = False

        # Override driver commands if the auto align button is being pressed
        if self.auto_aligning:
            self.coral_autoscore_right_fsm.run()
        else:
            SmartDashboard.putBoolean("Field Oriented Control", rel)
            self.driveTrain.driveRobot(x_speed, y_speed, rot_speed, self.control_loop_wait_time, field_relative=rel, driver_relative=rel)

            l_trigger = self.driveController.getLeftTriggerAxis()
            r_trigger = self.driveController.getRightTriggerAxis()

            l_trigger = MyRobot.deadband(l_trigger, 0.2)
            r_trigger = MyRobot.deadband(r_trigger, 0.2)

            if r_trigger > 0:
                self.coral_intake_fsm.intake()
            elif l_trigger > 0:
                self.coral_score_fsm.run()
        # endregion

            # Old scoring
            # self.grabber.intake_percent = -0.2
            # self.grabber.kicker_percent = 0.5

            if l_trigger <= 0.1 and r_trigger <= 0.1 and not self.manual_intake:
                self.grabber.intake_percent = 0.02
                self.grabber.kicker_percent = 0

            if self.lift_setpoint_index > 0 or self.arm_setpoint_index > 2:
                speed_scalar = 0.5
                slow = True
            else:
                speed_scalar = 1
                slow = False

        # endregion


    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def robotPeriodic(self):
        # Called every 20ms in each run mode.
        super().robotPeriodic()
        if self.estop:
            return

        SmartDashboard.putNumber("Current Lift Setpoint", self.lift_setpoints[self.lift_setpoint_index])
        SmartDashboard.putNumber("Current Angle Setpoint", self.arm_setpoints[self.arm_setpoint_index])

        SmartDashboard.putNumber("Current Lift Height", self.lift.get_height())
        SmartDashboard.putNumber("Current Arm Angle", self.arm.arm_angle)
        try:
            CommandScheduler.getInstance().run()
        except:
            pass
    def disabledInit(self):
        self.driveTrain.on_disable()

    @staticmethod
    def deadband(signal, db=0.2):
        if abs(signal) < db:
            return 0
        else:
            return signal
