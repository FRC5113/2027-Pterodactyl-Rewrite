import phoenix6
from wpilib import Notifier, RobotController

from components.swerve_drive import SwerveDrive


class SwerveSim:
    _SIM_LOOP_PERIOD: phoenix6.units.second = 0.004  # 4 ms temp

    def __init__(self, swerve: SwerveDrive) -> None:
        self.drivetrain = swerve

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: phoenix6.units.second = 0.0

        # Run simulation at a faster rate so PID gains behave more reasonably
        self._last_sim_time = phoenix6.utils.get_current_time_seconds()
        self._sim_notifier = Notifier(self.simulation_periodic)
        self._sim_notifier.start_periodic(self._SIM_LOOP_PERIOD)

    def simulation_periodic(self):
        current_time = phoenix6.utils.get_current_time_seconds()
        delta_time = current_time - self._last_sim_time
        self._last_sim_time = current_time

        # Use the measured time delta, get battery voltage from WPILib
        self.drivetrain.drivetrain.update_sim_state(
            delta_time, RobotController.get_battery_voltage()
        )
