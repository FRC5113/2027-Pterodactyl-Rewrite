import math

from magicbot import feedback, will_reset_to

from components.swerve_drive import SwerveDrive
from game import alliance_hub_pos, is_red


class Ballistics:
    drivetrain: SwerveDrive

    shooter_speed = will_reset_to(0.0)
    target_angle = 0.0

    def setup(self):
        # Meters
        self.distance_lookup = [1.597, 2.597, 3.597, 4.597]

        # RPS
        self.speed_lookup = [41.95, 45.8, 48.9, 53.0]

        self.distance_hub = 0.0

    def _linear_interp(self, x: float, xp: list[float], fp: list[float]):
        """linear interpolation"""

        if x <= xp[0]:
            return fp[0]
        if x >= xp[-1]:
            return fp[-1]

        for i in range(len(xp) - 1):
            if xp[i] <= x <= xp[i + 1]:
                # Linear interpolation formula
                t = (x - xp[i]) / (xp[i + 1] - xp[i])
                return fp[i] + t * (fp[i + 1] - fp[i])

        return fp[-1]

    @feedback
    def get_distance_hub(self):
        """
        Returns distance to hub
        """
        return self.distance_hub

    @feedback
    def get_speed(self):
        """
        returns shooter speed
        """
        return self.shooter_speed

    @feedback
    def get_target_angle(self):
        """
        just read the func name bro
        """
        return self.target_angle

    def execute(self):
        pose = self.drivetrain.get_pose().translation()
        hub_pose = alliance_hub_pos(
            is_red()
        )  # gets what the hub pose is given if we are in red alliance

        self.distance_hub = hub_pose.distance(pose)

        self.target_angle = math.atan2(
            (hub_pose.y - pose.y), (hub_pose.x - pose.x)
        )  # gets angle to turn to from robot to hub

        self.shooter_speed = self._linear_interp(
            self.distance_hub, self.distance_lookup, self.speed_lookup
        )  # gets what speed to use to score based off distance with linear interp
        # between known values cause more accruate and safe than equation through all points
