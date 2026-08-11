from modified_libs.magicbot import state


class ScoreController:
    # i would suggest looking at this for shoot anywhere https://github.com/thedropbears/pyrebuilt/blob/main/components/ballistics.py

    def setup(self):
        # Meters
        self.distance_lookup = [1.597, 2.597, 3.597, 4.597]  # TODO Tune these values

        # RPS
        self.speed_lookup = [41.95, 45.8, 48.9, 53.0]  # TODO Tune these values

        # Seconds — measured flight times at each distance
        self.time_lookup = [0.97, 1.21, 1.2, 1.2]  # TODO Tune these values

    def request_score(self): ...

    @state
    def readying_shot(self):
        """
        sends angle to drivetrain and velocity to shooter(non controller so it does not shoot) then once at angle go to shoot
        """

    @state
    def scoring(self):
        """
        sends velocity to shooter controller
        """
