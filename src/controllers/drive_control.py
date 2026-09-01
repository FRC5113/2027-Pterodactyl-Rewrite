from phoenix6 import swerve, units
from wpimath import Rotation2d

from components.swerve_drive import SwerveDrive
from modified_libs.magicbot import StateMachine, state, will_reset_to


class DriveControl(StateMachine):
    drivetrain: SwerveDrive

    vel_x = will_reset_to(0.0)
    vel_y = will_reset_to(0.0)
    omega = will_reset_to(0.0)
    field_relative = will_reset_to(True)
    point_req = will_reset_to(False)
    point_blue_perspective = will_reset_to(False)
    x_brake = will_reset_to(False)

    def setup(self):
        self.requested_angle = Rotation2d()

        self.brake_control = swerve.requests.SwerveDriveBrake()
        self.field_control = swerve.requests.FieldCentric()
        self.robot_control = swerve.requests.RobotCentric()
        self.point_control = swerve.requests.FieldCentricFacingAngle()

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
        print(angle)
        self.engage()

    def request_x_brake(self):
        self.x_brake = True
        self._try_engage()

    @state(first=True)
    def idle(self):
        if self.x_brake:
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
                .with_heading_pid(3.0, 0.0, 0.0)
            )
        else:
            self.drivetrain.apply_control(
                self.point_control.with_velocity_x(self.vel_x)
                .with_velocity_y(self.vel_y)
                .with_target_direction(self.requested_angle)
                .with_heading_pid(3.0, 0.0, 0.0)
            )

    @state
    def braking(self):
        self.drivetrain.apply_control(self.brake_control)
