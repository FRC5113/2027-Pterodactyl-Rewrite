import math

from wpimath import Rotation2d

from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from controllers.ballistics import Ballistics
from controllers.drive_control import DriveControl
from controllers.shooter_controller import ShooterController
from modified_libs.magicbot import StateMachine, state, will_reset_to


class ScoreController(StateMachine):
    # i would suggest looking at this for shoot anywhere https://github.com/thedropbears/pyrebuilt/blob/main/components/ballistics.py

    shooter_controller: ShooterController
    drive_control: DriveControl
    ballistics: Ballistics

    shooter: Shooter
    drivetrain: SwerveDrive

    target_angle = will_reset_to(0.0)
    target_rps = will_reset_to(0.0)

    def setup(self):
        self.angle_tol = 0.035  # rads (~2 deg)

    def request_score(self):
        self.engage()

    def _is_aimed(self):
        heading = self.drivetrain.get_pose().rotation().radians()
        diff = self.target_angle - heading
        error = math.atan2(math.sin(diff), math.cos(diff))
        return abs(error) <= self.angle_tol

    def _update_values(self):
        self.target_angle = self.ballistics.get_target_angle()
        self.target_rps = self.ballistics.get_speed()

    @state(first=True)
    def readying_shot(self):
        """
        sends angle to drivetrain and velocity to shooter(non controller so it does not shoot) then once at angle go to shoot
        """
        self._update_values()
        self.drive_control.request_angle_blue_perspective(Rotation2d(self.target_angle))

        self.shooter_controller.request_only_spin_up()
        self.shooter_controller.request_shot(self.target_rps)

        if self._is_aimed() and self.shooter_controller.at_speed():
            self.next_state("scoring")

    @state
    def scoring(self):
        """
        sends velocity to shooter controller
        """
        self._update_values()
        self.drive_control.request_angle_blue_perspective(Rotation2d(self.target_angle))
        self.shooter_controller.request_shot(self.target_rps)
