import time

import wpimath
from magicbot import StateMachine, state, timed_state
from photonlibpy import PhotonCamera
from wpilib import Timer, Field2d, SmartDashboard, DriverStation
from wpimath._controls._controls.controller import HolonomicDriveController, ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import Trajectory, TrajectoryGenerator, TrajectoryConfig, \
    TrapezoidProfileRadians
from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from wpimath.units import feetToMeters

from components.chassis import DriveTrain
from components.lift import Grabber, Arm, Lift


class IntakeCoral(StateMachine):
    grabber: Grabber
    arm: Arm
    lift: Lift

    starting_position = 0
    percent_target = 0
    starting_angle = 0

    def intake(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.percent_target = 0.625
        self.starting_position = self.lift.get_height()
        self.starting_angle = self.arm.arm_angle
        if self.arm.arm_angle <= -40:
            self.next_state("lower_arm")
        else:
            self.next_state("just_intake")

    @state()
    def just_intake(self):
        self.grabber.intake_percent = self.percent_target

    @state()
    def lower_arm(self):
        self.arm.arm_angle = -107
        if self.arm.arm_angle <= -100:
            self.next_state("begin_intake")

    @timed_state(duration=2.0, next_state="begin_hold")
    def begin_intake(self):
        self.grabber.kicker_percent = -0.1
        self.lift.set_height(0)
        self.grabber.intake_percent = self.percent_target

    @state()
    def begin_hold(self):
        self.percent_target = 0.05
        self.lift.set_height(2)
        self.next_state("hold")

    @state()
    def hold(self):
        self.grabber.intake_percent = self.percent_target

    def done(self):
        super().done()
        self.arm.arm_angle = self.starting_angle
        self.lift.set_height(self.starting_position)


class IntakeHome(StateMachine):
    arm: Arm
    lift: Lift

    def run(self):
        self.engage()

    @state(first=True, must_finish=True)
    def initialize(self):
        self.lift.set_height(2)
        self.arm.arm_angle = -75
        if self.lift.get_height() <= 4:
            # self.next_state("arm_down")
            self.done()


# @state(must_finish=True)
# def arm_down(self):
# self.arm.arm_angle = -75
# self.done()

class CheckAlignment(StateMachine):
    arm: Arm

    original_angle = 0

    def run(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.original_angle = self.arm.arm_angle
        self.next_state("go_down")

    @state()
    def go_down(self):
        self.arm.arm_angle = self.original_angle - 10

    def done(self):
        super().done()
        self.arm.arm_angle = self.original_angle


class ClearReef(StateMachine):
    arm: Arm

    original_angle = 0

    def run(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.original_angle = self.arm.arm_angle
        self.next_state("go_up")

    @state()
    def go_up(self):
        self.arm.arm_angle = self.original_angle + 10

    def done(self):
        super().done()
        self.arm.arm_angle = self.original_angle


class ScoreCoral(StateMachine):
    grabber: Grabber
    arm: Arm
    driveTrain: DriveTrain

    angle_target = 0
    initial_angle = 0

    def run(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.initial_angle = self.arm.arm_angle
        if self.arm.arm_angle < - 10:
            self.next_state("just_feed_out")
        else:
            self.angle_target = self.arm.arm_angle - 10
            self.next_state("begin_scoring")

    @timed_state(duration=2.0)
    def just_feed_out(self):
        self.grabber.kicker_percent = 0.5
        self.grabber.intake_percent = -0.2

    @timed_state(duration=1.0, next_state="output_coral")
    def begin_scoring(self):
        self.arm.arm_angle = self.angle_target

    @timed_state(duration=1, next_state="run_away")
    def output_coral(self):
        self.grabber.kicker_percent = 0.5
        self.grabber.intake_percent = -0.2

    @timed_state(duration=1, next_state="final")
    def run_away(self):  # Like the brave Sir Robin
        self.grabber.intake_percent = -0.2
        self.driveTrain.set_angle(90)
        self.driveTrain.drive_all_percent(-0.2)

    @state()
    def final(self):
        self.arm.arm_angle = self.initial_angle

    def done(self):
        super().done()
        self.arm.arm_angle = self.initial_angle
        self.grabber.kicker_percent = 0
        self.grabber.intake_percent = 0


# class DriveRobotToPoint(StateMachine):
#     driveTrain: DriveTrain
#     controller: HolonomicDriveController
#
#

def clamp(val, magnitude):
    if val > magnitude:
        val = magnitude
    if val < -magnitude:
        val = -magnitude
    return val


class ScoreCoralRight(StateMachine):
    """Make the robot drive by a certain translation. This is used extensively by the vision tracking so that the robot
    can reliably auto-align after a target leaves the FOV.
    """
    driveTrain: DriveTrain
    field: Field2d
    camera: PhotonCamera
    lift: Lift
    arm: Arm
    grabber: Grabber

    target_pose = Pose2d()
    complete = False
    starting_lift_height = 0
    starting_arm_angle = 0
    y_offset = 0.05
    seen_target = False

    red_target = 10
    blue_target = 21

    def run(self):
        self.engage()

    def set_transform(self, transform):
        start = self.driveTrain.getPose()
        blank_pose = Pose2d(start.translation(), Rotation2d(0))
        blank_pose = blank_pose.transformBy(transform)
        blank_pose = blank_pose.rotateAround(start.translation(), start.rotation())
        self.target_pose = blank_pose

    def calculate_offset_pose(self):
        start = self.target_pose
        blank_pose = Pose2d(start.translation(), Rotation2d(0))
        blank_pose = blank_pose.transformBy(Transform2d(0.5, 0, 0))
        blank_pose = blank_pose.rotateAround(start.translation(), start.rotation())
        return blank_pose

    def target_right(self):
        result = self.camera.getLatestResult()
        best_target = None
        scorable_id = self.red_target
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            scorable_id = self.blue_target

        for target in result.getTargets():
            if target.getFiducialId() == scorable_id:
                best_target = target

        if best_target:
            target_offset = best_target.getBestCameraToTarget()
            SmartDashboard.putNumber("Target X Offset", target_offset.x)
            SmartDashboard.putNumber("Target Y Offset", target_offset.y)
            SmartDashboard.putNumber("Vision Target ID", float(best_target.getFiducialId()))
            # Subtract 0.5 from the target position since the alignment function will go 0.5m forward after ligning up
            relative_x = target_offset.x - 0.29 - 0.5
            relative_y = target_offset.y + self.y_offset
            angle_difference = target_offset.rotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(180))
            relative_rot = angle_difference.radians()
            # Update our target vector the entire time the camera can see the target
            self.set_transform(Transform2d(relative_x, relative_y, relative_rot))
            self.seen_target = True

    @state(first=True)
    def initialize(self):
        self.seen_target = False
        self.next_state("drive")

    @state()
    def drive(self):
        self.starting_lift_height = self.lift.get_height()
        self.starting_arm_angle = self.arm.arm_angle
        self.target_right()
        self.complete = False
        pose_err = self.target_pose.relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        if self.seen_target:
            self.driveTrain.driveRobot(clamp(x_err * 3, 3), clamp(y_err * 3, 3), clamp(angle_err * 2, 3), 0.02,
                                       field_relative=False)
            if abs(x_err) < 0.2 and abs(y_err) < 0.2 and abs(angle_err) < 0.09:
                self.next_state("stabilize")
        else:
            self.driveTrain.driveRobot(1, 0, 0, 0.02, field_relative=False)

    @timed_state(duration=1.5, next_state="approach")
    def stabilize(self):
        print("stabilizing")
        self.lift.set_height(24)
        self.arm.arm_angle = 60
        self.target_right()
        pose_err = self.target_pose.relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 2, 3), clamp(y_err * 2, 3), clamp(angle_err * 2, 3), 0.02,
                                   field_relative=False)
        # if abs(x_err) > 0.05 or abs(y_err) > 0.05 or abs(angle_err) > 0.09:
        #     self.next_state_now("initialize")

    @state()
    def approach(self):
        pose_err = self.calculate_offset_pose().relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 2, 1), clamp(y_err * 3, 1), clamp(angle_err * 2, 1), 0.02,
                                   field_relative=False)
        if abs(x_err) < 0.05 and abs(y_err) < 0.05:
            self.next_state("hold")

    @timed_state(duration=1.0, next_state="score")
    def hold(self):
        pose_err = self.calculate_offset_pose().relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 3, 1), clamp(y_err * 3, 1), clamp(angle_err * 5, 2), 0.02,
                                   field_relative=False)

    @timed_state(duration=0.75, next_state="run_away")
    def score(self):
        self.arm.arm_angle = 25

    @timed_state(duration=1.0, next_state="finished")
    def run_away(self):
        self.grabber.intake_percent = -0.5
        pose_err = self.target_pose.relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 2, 1), clamp(y_err * 2, 1), clamp(angle_err * 2, 1), 0.02,
                                   field_relative=False)

    @state()
    def finished(self):
        self.driveTrain.driveRobot(0, 0, 0, 0.02)

    def done(self):
        super().done()
        self.grabber.intake_percent = 0.02
        self.lift.set_height(self.starting_lift_height)
        self.arm.arm_angle = self.starting_arm_angle
        self.complete = True


class ScoreCoralLeft(StateMachine):
    """Make the robot drive by a certain translation. This is used extensively by the vision tracking so that the robot
    can reliably auto-align after a target leaves the FOV.
    """
    driveTrain: DriveTrain
    field: Field2d
    camera: PhotonCamera
    lift: Lift
    arm: Arm
    grabber: Grabber

    target_pose = Pose2d()
    complete = False
    starting_lift_height = 0
    starting_arm_angle = 0
    y_offset = -0.37

    def run(self):
        self.engage()

    def set_transform(self, transform):
        start = self.driveTrain.getPose()
        blank_pose = Pose2d(start.translation(), Rotation2d(0))
        blank_pose = blank_pose.transformBy(transform)
        blank_pose = blank_pose.rotateAround(start.translation(), start.rotation())
        self.target_pose = blank_pose

    def calculate_offset_pose(self):
        start = self.target_pose
        blank_pose = Pose2d(start.translation(), Rotation2d(0))
        blank_pose = blank_pose.transformBy(Transform2d(0.5, 0, 0))
        blank_pose = blank_pose.rotateAround(start.translation(), start.rotation())
        return blank_pose

    def target_right(self):
        result = self.camera.getLatestResult()
        best_target = result.getBestTarget()
        if best_target:
            target_offset = best_target.getBestCameraToTarget()
            SmartDashboard.putNumber("Target X Offset", target_offset.x)
            SmartDashboard.putNumber("Target Y Offset", target_offset.y)
            # Subtract 0.5 from the target position since the alignment function will go 0.5m forward after ligning up
            relative_x = target_offset.x - 0.29 - 0.5
            relative_y = target_offset.y + self.y_offset
            angle_difference = target_offset.rotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(180))
            relative_rot = angle_difference.radians()
            # Update our target vector the entire time the camera can see the target
            self.set_transform(Transform2d(relative_x, relative_y, relative_rot))

    @state(first=True)
    def initialize(self):
        self.starting_lift_height = self.lift.get_height()
        self.starting_arm_angle = self.arm.arm_angle
        self.target_right()
        self.complete = False
        pose_err = self.target_pose.relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 3, 3), clamp(y_err * 3, 3), clamp(angle_err * 2, 3), 0.02,
                                   field_relative=False)
        if abs(x_err) < 0.05 and abs(y_err) < 0.05 and abs(angle_err) < 0.09:
            self.next_state("stabilize")

    @timed_state(duration=1.0, next_state="approach")
    def stabilize(self):
        print("stabilizing")
        self.lift.set_height(24)
        self.arm.arm_angle = 60
        self.target_right()
        pose_err = self.target_pose.relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 2, 3), clamp(y_err * 2, 3), clamp(angle_err * 2, 3), 0.02,
                                   field_relative=False)
        if abs(x_err) > 0.05 or abs(y_err) > 0.05 or abs(angle_err) > 0.09:
            self.next_state_now("initialize")

    @state()
    def approach(self):
        pose_err = self.calculate_offset_pose().relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 3, 1), clamp(y_err * 3, 1), clamp(angle_err * 3, 1), 0.02,
                                   field_relative=False)
        if abs(x_err) < 0.05 and abs(y_err) < 0.05 and abs(angle_err) < 0.09:
            self.next_state("hold")

    @timed_state(duration=1.0, next_state="score")
    def hold(self):
        pose_err = self.calculate_offset_pose().relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 3, 1), clamp(y_err * 3, 1), clamp(angle_err * 5, 2), 0.02,
                                   field_relative=False)

    @timed_state(duration=0.75, next_state="run_away")
    def score(self):
        self.arm.arm_angle = 25

    @timed_state(duration=1.0, next_state="finished")
    def run_away(self):
        self.grabber.intake_percent = -0.5
        pose_err = self.target_pose.relativeTo(self.driveTrain.getPose())
        x_err = pose_err.x
        y_err = pose_err.y
        angle_err = pose_err.rotation().radians()
        self.driveTrain.driveRobot(clamp(x_err * 2, 1), clamp(y_err * 2, 1), clamp(angle_err * 2, 1), 0.02,
                                   field_relative=False)

    @state()
    def finished(self):
        self.driveTrain.driveRobot(0, 0, 0, 0.02)

    def done(self):
        super().done()
        self.grabber.intake_percent = 0.02
        self.lift.set_height(self.starting_lift_height)
        self.arm.arm_angle = self.starting_arm_angle
        self.complete = True


class ScoreLevelFour(StateMachine):
    grabber: Grabber
    arm: Arm
    driveTrain: DriveTrain
    lift: Lift

    starting_lift_position = 0
    starting_arm_angle = 0

    def run(self):
        self.engage()

    @state(first=True)
    def initialize(self):
        self.starting_lift_position = self.lift.get_height()
        self.starting_arm_angle = self.arm.arm_angle
        self.lift.set_height(24)
        self.arm.arm_angle = 60
        self.next_state("align_vision_target")

    @state()
    def align_vision_target(self):

        self.next_state("wait_for_pos")

    @state()
    def wait_for_pos(self):

        if self.arm.arm_angle >= 55:
            if self.lift.get_height() >= 22:
                self.next_state("goto_scoring_pos")

    @state()
    def goto_scoring_pos(self):
        # todo: drive into position
        # Wait for position
        self.next_state("start_score")

    @state()
    def start_score(self):
        self.arm.arm_angle = 55

        if abs(self.arm.arm_angle - 55) <= 1:
            self.next_state("score")

    @state()
    def score(self):
        pass

    def done(self) -> None:
        super().done()
        self.lift.set_height(self.starting_lift_position)
        self.arm.arm_angle = self.starting_arm_angle
