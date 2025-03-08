import choreo
import magicbot
import wpimath.filter
from commands2 import *
from commands2.button import CommandXboxController
from wpimath._controls._controls.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians

from commands import LiftVelocity, RunIntake, SetKickerPercent, AddArmAngle
from components.chassis import *
from components.climber import Climber
from components.lift import Lift, Arm, Grabber
from state_machines import IntakeCoral, ScoreCoral


class MyRobot(magicbot.MagicRobot):
    # Controllers
    driveController: CommandXboxController
    operatorController: CommandXboxController

    # Automatic robot controllers
    automaticController: HolonomicDriveController

    # Other
    field: Field2d
    camera: PhotonCamera

    # Subsystems
    driveTrain: DriveTrain
    lift: Lift
    arm: Arm
    climber: Climber
    grabber: Grabber

    # State machines
    coral_intake_fsm: IntakeCoral
    coral_score_fsm: ScoreCoral

    def __init__(self):
        super().__init__()
        # Define simple variables here.
        self.estop: bool = False
        self.lift_setpoints = [2, 26]
        self.lift_setpoint_index = 0
        self.arm_setpoints = [-107, -75, -60, 5, 60]
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

    def createObjects(self):
        SmartDashboard.putBoolean("Arm Angle In Manual", self.arm_manual)
        SmartDashboard.putBoolean("Lift Height In Manual", self.lift_manual)
        SmartDashboard.putNumber("Desired X Velocity", 0)
        SmartDashboard.putNumber("Desired Y Velocity", 0)
        SmartDashboard.putNumber("Desired Rotational Velocity", 0)
        SmartDashboard.putBoolean("Drive Train in Slow Mode", False)

        self.automaticController = HolonomicDriveController(
            PIDController(2, 0, 0.1),
            PIDController(2, 0, 0.1),
            ProfiledPIDControllerRadians(2, 0, 0.1, TrapezoidProfileRadians.Constraints(3, 3))
        )

        # Load choreo trajectory
        try:
            self.trajectory = choreo.load_swerve_trajectory("test_auton_lower (three score)")
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
            InstantCommand(self.goto_zero_extension).andThen(WaitUntilCommand(self.at_zero_extension))
            .andThen(
                ConditionalCommand(
                    add_angle_cmd
                    ,
                    InstantCommand(self.increase_arm_setpoint),
                    self.is_arm_manual
                )
            )
        ).onFalse(
            InstantCommand(self.unset_manual_arm)
        )

        self.operatorController.povLeft().onTrue(
            InstantCommand(self.goto_zero_extension).andThen(WaitUntilCommand(self.at_zero_extension))
            .andThen(
                ConditionalCommand(
                    sub_angle_cmd
                    ,
                    InstantCommand(self.decrease_arm_setpoint),
                    self.is_arm_manual
                )
            )
        ).onFalse(
            InstantCommand(self.unset_manual_arm)
        )

        self.operatorController.start().onTrue(
            InstantCommand(self.toggle_arm_manual)
        )

        self.operatorController.x().onTrue(
            InstantCommand(print(self.driveTrain.represent_pose()))
        )

        # endregion

        # region driverControls

        self.driveController.start().onTrue(InstantCommand(self.toggle_driver_control_mode))

        self.driveController.rightBumper().onTrue(
            RunIntake(0.05, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", True)))
        ).onFalse(
            RunIntake(0, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", False)))
        )

        self.driveController.leftBumper().onTrue(
            RunIntake(-0.05, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", True)))
        ).onFalse(
            RunIntake(0, self.grabber)
            .andThen(InstantCommand(lambda: setattr(self, "manual_intake", False)))
        )

        self.driveController.a().onTrue(
            SetKickerPercent(-0.5, self.grabber)
        ).onFalse(
            SetKickerPercent(0, self.grabber)
        )

        self.driveController.y().onTrue(
            SetKickerPercent(0.05, self.grabber)
        ).onFalse(
            SetKickerPercent(0, self.grabber)
        )

        # endregion

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

    def at_zero_extension(self):
        return self.arm.extension < 1

    def goto_zero_extension(self):
        self.extension_setpoint = 0

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

    def teleopPeriodic(self):
        # Called every 20ms when teleop runs

        # region operatorControl

        lt = self.operatorController.getLeftTriggerAxis()
        rt = self.operatorController.getRightTriggerAxis()
        lt = MyRobot.deadband(lt, 0.1)
        rt = MyRobot.deadband(rt, 0.1)

        travel = 0.4 * 8 * (rt - lt) * 0.02
        self.extension_setpoint += travel
        if self.extension_setpoint < 0:
            self.extension_setpoint = 0
        if self.extension_setpoint > 8:
            self.extension_setpoint = 8

        self.arm.extension = self.extension_setpoint

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
                    wpimath.applyDeadband(self.driveController.getLeftY(), 0.02)
                ) * kMaxSpeed
        )

        y_speed = (
                -self.driveylimiter.calculate(
                    wpimath.applyDeadband(self.driveController.getLeftX(), 0.02)
                ) * kMaxSpeed
        )

        rot_speed = (
                -self.driveanglelimiter.calculate(
                    wpimath.applyDeadband(self.driveController.getRightX(), 0.02)
                ) * kMaxSpeed
        )

        SmartDashboard.putNumber("x", x_speed)
        SmartDashboard.putNumber("y", y_speed)
        SmartDashboard.putNumber("z", rot_speed)
        if self.drive_mode == "field_oriented":
            rel = True
        else:
            rel = False
        SmartDashboard.putBoolean("Field Oriented Control", rel)
        self.driveTrain.driveRobot(x_speed, y_speed, rot_speed, self.control_loop_wait_time, field_relative=rel)
        # endregion

        climb = self.operatorController.getRightY()

        l_trigger = self.driveController.getLeftTriggerAxis()
        r_trigger = self.driveController.getRightTriggerAxis()

        l_trigger = MyRobot.deadband(l_trigger, 0.2)
        r_trigger = MyRobot.deadband(r_trigger, 0.2)

        climb = MyRobot.deadband(climb)

        if r_trigger > 0:
            self.coral_intake_fsm.intake()
        elif l_trigger > 0:
            self.coral_score_fsm.run()
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

        self.climber.percent_output = climb
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
        SmartDashboard.putNumber("Current Extension Setpoint", self.arm.extension_setpoint)

        CommandScheduler.getInstance().run()

    def disabledInit(self):
        self.driveTrain.on_disable()

    @staticmethod
    def deadband(signal, db=0.2):
        if abs(signal) < db:
            return 0
        else:
            return signal
