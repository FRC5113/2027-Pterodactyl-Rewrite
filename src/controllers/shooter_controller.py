from components.indexer import Indexer
from components.kicker import Kicker
from components.shooter import Shooter
from lemonlib.smart import SmartPreference
from modified_libs.magicbot import StateMachine, state, will_reset_to


class ShooterController(StateMachine):
    shooter: Shooter
    indexer: Indexer
    kicker: Kicker

    shooter_velocity = will_reset_to(0.0)

    requested_shoot = will_reset_to(False)
    only_spin_up = will_reset_to(False)

    shooter_tolerance = SmartPreference(3.0)  # In Rotations per second

    indexer_throttle = 0.8
    kicker_throttle = 0.8

    def request_shot(self, velocity):
        self.requested_shoot = True
        self.shooter_velocity = velocity
        self.engage()

    def request_only_spin_up(self):
        self.only_spin_up = True

    def at_speed(self) -> bool:
        return (
            abs(self.shooter.get_velocity() - self.shooter_velocity)
            < self.shooter_tolerance
        )

    @state(first=True)
    def idle(self):
        if self.requested_shoot:
            self.next_state("spin_up")

    @state
    def spin_up(self):
        in_tolerance = (
            abs(self.shooter.get_velocity() - self.shooter.get_requested_velocity())
            < self.shooter_tolerance
        )
        if in_tolerance and (not self.only_spin_up):
            self.next_state("shooting")

    @state
    def shooting(self):
        self.shooter.set_velocity(self.shooter_velocity)
        self.indexer.set_throttle(self.indexer_throttle)
        self.kicker.set_throttle(self.kicker_throttle)
