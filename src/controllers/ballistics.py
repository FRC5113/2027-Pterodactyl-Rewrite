import math

from components.swerve_drive import SwerveDrive
from game import alliance_hub_pos, is_red
from modified_libs.magicbot import feedback, will_reset_to


class Ballistics:

    drivetrain: SwerveDrive

    shooter_speed = will_reset_to(0.0)
    target_angle = 0.0

    def setup(self):
        # Meters
        self.distance_lookup = [1.597, 2.597, 3.597, 4.597]  # TODO Tune these values

        # RPS
        self.speed_lookup = [41.95, 45.8, 48.9, 53.0]  # TODO Tune these values

        # Seconds — measured flight times at each distance
        self.time_lookup = [0.97, 1.21, 1.2, 1.2]  # TODO Tune these values

        self.distance_hub = 0.0
        self.distance_left = 0.0
        self.distance_right = 0.0

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
        return self.distance_hub

    def get_distance_left(self):
        return self.distance_left

    def get_distance_right(self):
        return self.distance_right

    @feedback
    def get_speed(self):
        return self.shooter_speed

    @feedback
    def get_target_angle(self):
        return self.target_angle

    def execute(self):
        pose = self.drivetrain.get_pose().translation()
        hub_pose = alliance_hub_pos(is_red())

        self.distance_hub = hub_pose.distance(pose)

        self.target_angle = math.atan2(
            (hub_pose.Y() - pose.Y()), (hub_pose.X() - pose.X())
        )

        self.shooter_speed = self._linear_interp(
            self.distance_hub, self.distance_lookup, self.speed_lookup
        )
