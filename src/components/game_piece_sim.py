from wpilib import RobotBase, RobotController
from wpimath import Translation3d, units
from wpimath.units import meters, meters_per_second, radians, radians_per_second

from components.intake import Intake
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from fuel_sim.fuel_sim import FuelSim


class GamePieceSim:
    """
    Simulates fuel, hubs, collisions, and alliance points within `AdvantageScope`.
    """

    drivetrain: SwerveDrive
    intake: Intake
    shooter: Shooter

    robot_width: meters = units.inchesToMeters(27.5)
    robot_length: meters = units.inchesToMeters(27.5)
    bumper_height: meters = units.inchesToMeters(3.0)

    intake_length: meters = units.inchesToMeters(12)
    intake_width: meters = units.inchesToMeters(27.5)

    shooter_max_velocity: radians_per_second = (
        units.rotationsPerMinuteToRadiansPerSecond(60.0)
    )
    shooter_flywheel_radius: meters = units.inchesToMeters(2.0)
    shooter_hood_angle: radians = units.degreesToRadians(78.0)
    shooter_height: meters = units.inchesToMeters(21.0)

    def setup(self) -> None:
        """
        Registers a `FuelSim`, robot, intake, & shooter from injected constants.
        """
        self._fuel_sim = FuelSim()

        self._fuel_sim.register_robot(
            width=self.robot_width,
            length=self.robot_length,
            bumper_height=self.bumper_height,
            pose_supplier=lambda: self.drivetrain.get_pose(),
            field_speeds_supplier=lambda: self.drivetrain.get_chassis(),
        )

        self._fuel_sim.enable_air_resistance()

        intake_length_half = self.intake_length / 2
        intake_front = self.robot_length / 2

        self._fuel_sim.register_intake(
            x_min=intake_front,
            x_max=intake_front + self.intake_width,
            y_min=-intake_length_half,
            y_max=intake_length_half,
            able_to_intake=lambda: None,  # There are no encoders and its just run off voltage and looking,
        )

        self._fuel_sim.log_fuels()

        self._fuel_sim.set_subticks(5)
        self._fuel_sim.set_logging_frequency(300)

        self._fuel_sim.start()

    def spawn_fuel_line(self) -> None:
        """
        Spawns a single line of fuel in the middle of the field.
        """
        self._fuel_sim.spawn_fuel(Translation3d(8.0, 4.0, 0.05), Translation3d(0, 0, 0))
        self._fuel_sim.spawn_fuel(Translation3d(8.2, 4.0, 0.05), Translation3d(0, 0, 0))
        self._fuel_sim.spawn_fuel(Translation3d(8.4, 4.0, 0.05), Translation3d(0, 0, 0))
        self._fuel_sim.spawn_fuel(Translation3d(8.6, 4.0, 0.05), Translation3d(0, 0, 0))
        self._fuel_sim.spawn_fuel(Translation3d(8.8, 4.0, 0.05), Translation3d(0, 0, 0))
        self._fuel_sim.spawn_fuel(Translation3d(9.0, 4.0, 0.05), Translation3d(0, 0, 0))

    def clear_all_fuel(self) -> None:
        """
        Clears all fuel currently in the field.
        """
        self._fuel_sim.clear_fuel()

    def _get_ball_velocity(self) -> meters_per_second:
        """
        Returns the ball velocity in meters per second based on current shooter voltage.
        """
        percent = (
            self.shooter.get_applied_voltage() / RobotController.getBatteryVoltage()
        )
        motor_velocity = percent * self.shooter_max_velocity
        ball_velocity = motor_velocity * self.shooter_flywheel_radius

        return ball_velocity

    def shoot_fuel(self) -> None:
        """
        Launches a fuel object using the current shooter velocity.
        """
        self._fuel_sim.launch_fuel(
            launch_velocity=self._get_ball_velocity(),
            hood_angle=self.shooter_hood_angle,
            shooter_yaw=0.0,
            launch_height=self.shooter_height,
        )

    def execute(self) -> None:
        """
        Updates the state of the fuel sim each iteration in simulation mode.
        This should only be executed in simulation mode to prevent performance bottlenecks.
        """

        if RobotBase.isSimulation():
            self._fuel_sim.update_sim()
