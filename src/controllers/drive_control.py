import math

from choreo.trajectory import SwerveTrajectory
from magicbot import StateMachine, state, will_reset_to
from phoenix6 import swerve, units
from wpilib import Alliance, DriverStationBackend
from wpimath import ChassisVelocities, Pose2d, Rotation2d

from components.swerve_drive import SwerveDrive
from lemonlib.smart import SmartProfile


class DriveControl(StateMachine):
    drivetrain: SwerveDrive

    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    vel_x = will_reset_to(0.0)
    vel_y = will_reset_to(0.0)
    omega = will_reset_to(0.0)
    field_relative = will_reset_to(True)
    point_req = will_reset_to(False)
    point_blue_perspective = will_reset_to(False)
    x_brake = will_reset_to(False)
    following_traj = will_reset_to(False)
    current_sample = will_reset_to(None)

    DISTANCE_TOLERANCE = 0.05
    ANGLE_TOLERANCE = math.radians(1)  # radians
    TRANSLATIONAL_SPEED_TOLERANCE = 0.2  # m/s
    ROTATIONAL_SPEED_TOLERANCE = 0.1  # rad/s

    def setup(self):
        self.requested_angle = Rotation2d()

        self.brake_control = swerve.requests.SwerveDriveBrake()
        self.field_control = swerve.requests.FieldCentric()
        self.robot_control = swerve.requests.RobotCentric()

        heading_gains = self.rotation_profile.gains
        heading_gains["kI"] = (
            1e-9  # theres a weird bug where it cant be zero but kI gain sucks sooooo
        )
        self.point_control = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            heading_gains["kP"], heading_gains["kI"], heading_gains["kP"]
        )

    def on_enable(self) -> None:
        self.trans_controller = self.translation_profile.create_pid_controller(
            "Translation"
        )
        self.rot_contoller = self.rotation_profile.create_pid_controller("Rotation")

    def _try_engage(self):
        if not (self.vel_x == 0.0 and self.vel_y == 0.0 and self.omega == 0.0):
            self.engage()

    def request_drive_field(
        self,
        velocity_x: units.meters_per_second,
        velocity_y: units.meters_per_second,
        omega: units.radians_per_second,
    ):
        self.vel_x = velocity_x
        self.vel_y = velocity_y
        self.omega = omega
        self.field_relative = True
        self._try_engage()

    def request_drive_robot(
        self,
        velocity_x: units.meters_per_second,
        velocity_y: units.meters_per_second,
        omega: units.radians_per_second,
    ):
        self.vel_x = velocity_x
        self.vel_y = velocity_y
        self.omega = omega
        self.field_relative = False
        self._try_engage()

    def request_drive_point(
        self,
        velocity_x: units.meters_per_second,
        velocity_y: units.meters_per_second,
        angle: Rotation2d,
    ):
        self.vel_x = velocity_x
        self.vel_y = velocity_y
        self.requested_angle = angle
        self.field_relative = True
        self.point_req = True
        self._try_engage()

    def request_angle_blue_perspective(self, angle: Rotation2d):
        self.requested_angle = angle
        self.point_blue_perspective = True
        self.point_req = True
        self.engage()

    def request_x_brake(self):
        self.x_brake = True
        self._try_engage()

    def request_follow_trajectory(self, trajectory: SwerveTrajectory, state_tm: float):
        current_pose = self.drivetrain.get_pose()
        final_pose = trajectory.get_final_pose(
            DriverStationBackend.get_alliance() == Alliance.RED
        )
        if final_pose is None:
            final_pose = Pose2d()

        swerve_sample = trajectory.sample_at(
            state_tm, DriverStationBackend.get_alliance() == Alliance.RED
        )

        distance = current_pose.translation().distance(
            final_pose.translation()
        )  # Calculate distance to final pose
        angle_error = (
            final_pose.rotation() - current_pose.rotation()
        ).radians()  # Calculate angle error
        velocity = self.drivetrain.get_chassis()
        speed = math.hypot(velocity.vx, velocity.vy)  # Calculate speed

        in_distance_tolerance = distance < self.DISTANCE_TOLERANCE
        in_angle_tolerance = abs(angle_error) < self.ANGLE_TOLERANCE
        in_translational_speed_tolerance = speed < self.TRANSLATIONAL_SPEED_TOLERANCE
        in_rotational_speed_tolerance = (
            abs(velocity.omega) < self.ROTATIONAL_SPEED_TOLERANCE
        )
        is_in_second_half_of_trajectory = state_tm > trajectory.get_total_time() / 2.0

        if not (
            in_distance_tolerance
            and in_angle_tolerance
            and in_translational_speed_tolerance
            and in_rotational_speed_tolerance
            and is_in_second_half_of_trajectory
        ):
            self.current_sample = (swerve_sample, current_pose)
            self.following_traj = True
            self.engage()
            self.next_state_now("tracking_trajectory")

    def get_following_trajectory(self) -> bool:
        return self.following_traj

    @state(first=True)
    def idle(self):
        if (self.current_sample is not None) and (DriverStationBackend.is_autonomous()):
            self.next_state("tracking_trajectory")
        elif self.x_brake:
            self.next_state("braking")
        elif self.point_req:
            self.next_state("driving_point_field_centric")
        elif self.field_relative:
            self.next_state("driving_field_centric")
        else:
            self.next_state("driving_robot_centric")

    @state
    def driving_field_centric(self):
        self.drivetrain.apply_control(
            self.field_control.with_velocity_x(self.vel_x)
            .with_velocity_y(self.vel_y)
            .with_rotational_rate(self.omega)
        )

    @state
    def driving_robot_centric(self):
        self.drivetrain.apply_control(
            self.robot_control.with_velocity_x(self.vel_x)
            .with_velocity_y(self.vel_y)
            .with_rotational_rate(self.omega)
        )

    @state
    def driving_point_field_centric(self):
        if self.point_blue_perspective:
            self.drivetrain.apply_control(
                self.point_control.with_velocity_x(self.vel_x)
                .with_velocity_y(self.vel_y)
                .with_target_direction(self.requested_angle)
                .with_forward_perspective(
                    swerve.requests.ForwardPerspectiveValue.BLUE_ALLIANCE
                )
            )
        else:
            self.drivetrain.apply_control(
                self.point_control.with_velocity_x(self.vel_x)
                .with_velocity_y(self.vel_y)
                .with_target_direction(self.requested_angle)
            )

    @state
    def braking(self):
        self.drivetrain.apply_control(self.brake_control)

    @state
    def tracking_trajectory(self):
        if self.current_sample is not None:
            sample, pose = self.current_sample

            speeds = ChassisVelocities(
                sample.vx + self.trans_controller.calculate(pose.x, sample.x),
                sample.vy + self.trans_controller.calculate(pose.y, sample.y),
                sample.omega
                + self.rot_contoller.calculate(
                    pose.rotation().radians(), sample.heading
                ),
            ).to_robot_relative(pose.rotation())

            self.drivetrain.apply_control(
                self.robot_control.with_velocity_x(speeds.vx)
                .with_velocity_y(speeds.vy)
                .with_rotational_rate(speeds.omega)
            )
