import math
import time
HOOP_RADIUS = 1.0922  # meters = 42 inches
HOOP_HEIGHT = 1.8288  # meters = 72 inches
ACCURACY_TOLERANCE = 0.0762  # meters = 3 inches
NET_HEIGHT = 3.048  # meters = 120 inches
BALL_EXIT_HEIGHT = 0.254  # meters = 10 inches
GRAVITY = 9.81  # m/s^2
THETA = 23

# </Algorithms_etc>
from .indexer import Indexer
from .shooter import Shooter
from smartunits import Distance
from smartunits.distance import meters
class Controller:
    indexer: Indexer
    shooter: Shooter
    MAX_VELOCITY = 60 #rotations/s
    should_shoot = False
    distance_from_target: Distance
    self.force_shoot = False
    def __init__(self):
        self.indexer = Indexer()
        self.shooter = Shooter()
        self.distance_lookup = [1.597, 2.597, 3.597, 4.597]  # TODO Tune these values
        self.speed_lookup = [41.95, 45.8, 48.9, 53.0]  # TODO Tune these values
        self.hard_velocity = 47.5
    def stop_shoot(self):
        self.should_shoot = False
        self.force_shoot = False
    def start_shoot(self, distance: Distance):
        self.distance_from_target = distance
        self.should_shoot = True
    def force_shoot(self):
        self.force_shoot = True
    def execute(self):
        if self.force_shoot:
            self.shooter.set_velocity(self.hard_velocity)
        elif not self.should_shoot:
            self.shooter.set_velocity(
                0.8 * self.MAX_VELOCITY
            )  # 0.8 is a made up number, tune
            self.indexer.conveyor_off()
            self.indexer.kicker_off()
        else:
            velocity = 0
            if self.distance_from_target.in_unit(meters) > self.distance_lookup[-1]:
                velocity = self.speed_lookup[-1]
            else:

                #x = (real - high) / (low - high)
                #y = (low - real) / (low - high)
                for i, d in enumerate(self.distance_lookup):
                    if self.distance_from_target.in_unit(meters) > d:
                        real = self.distance_from_target.in_unit(meters)
                        low = d
                        high = self.distance_lookup[i + 1]
                        x = (real - high) / (low - high)
                        y = (low - real) / (low - high)
                        velocity = self.speed_lookup[i] * x + self.speed_lookup[i + 1] * y

            self.shooter.set_velocity(velocity)
            self.indexer.kicker_on()
            self.indexer.conveyor_on()

        self.indexer.execute()
        self.shooter.execute()
