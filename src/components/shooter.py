from phoenix6 import (
    BaseStatusSignal,
    configs,
    controls,
    signals,
    units,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import MotorAlignmentValue
from wpilib import RobotController

from lemonlib.ctre import tryUntilOk
from lemonlib.smart import SmartProfile
from modified_libs.magicbot import feedback, will_reset_to


class Shooter:

    right_motor: TalonFX
    left_motor: TalonFX

    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_stator_amps: units.ampere
    shooter_supply_amps: units.ampere
    shooter_peak_amps: units.ampere

    _NUM_CONFIG_ATTEMPTS = 2

    control = will_reset_to(controls.CoastOut())
    req_velocity = will_reset_to(0.0)

    def setup(self) -> None:
        # Configs common across all TalonFX motors.
        self.talon_fx_initial_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs().with_neutral_mode(
                    signals.NeutralModeValue.COAST
                )
            )
            .with_current_limits(
                configs.CurrentLimitsConfigs()
                .with_stator_current_limit(self.shooter_stator_amps)
                .with_stator_current_limit_enable(True)
                .with_supply_current_limit(self.shooter_supply_amps)
                .with_supply_current_limit_enable(True)
            )
        )

        # Configs for leader_motor.
        self.leader_motor_configs = (
            self.talon_fx_initial_configs.with_motor_output(
                self.talon_fx_initial_configs.motor_output.with_inverted(
                    signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                )
            )
            .with_feedback(
                self.talon_fx_initial_configs.feedback.with_sensor_to_mechanism_ratio(
                    self.shooter_gear_ratio
                )
            )
            .with_slot0(self.shooter_profile.create_ctre_flywheel_controller())
        )

        # Configs for follower_motor.
        self.follower_motor_configs = self.talon_fx_initial_configs.with_motor_output(
            self.talon_fx_initial_configs.motor_output.with_inverted(
                signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
        )

        # device status signals
        self.leader_motor_velocity = self.right_motor.get_velocity(False)
        self.leader_motor_torque_current = self.right_motor.get_torque_current(False)
        self.leader_motor_supply_amps = self.right_motor.get_supply_current(False)

        # controls used by the leader motors
        self.velocity_request = controls.VelocityVoltage(0.0)
        self.coast_request = controls.CoastOut()
        self.voltage_request = controls.VoltageOut(0.0)

        # apply device configs
        tryUntilOk(
            self._NUM_CONFIG_ATTEMPTS,
            lambda: self.right_motor.configurator.apply(self.leader_motor_configs),
        )
        tryUntilOk(
            self._NUM_CONFIG_ATTEMPTS,
            lambda: self.left_motor.configurator.apply(self.follower_motor_configs),
        )

        self.left_motor.set_control(
            controls.Follower(self.right_motor.device_id, MotorAlignmentValue.OPPOSED)
        )

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_velocity(self) -> units.rotations_per_second:
        """
        :returns: The velocity of the leader_motor motor
        :rtype: float
        """
        return self.leader_motor_velocity.value

    @feedback
    def get_requested_velocity(self) -> units.rotations_per_second:
        """
        :returns: The requested velocity of the leader_motor motor
        :rtype: float
        """
        return self.req_velocity

    @feedback
    def get_torque_current(self) -> units.ampere:
        """
        :returns: The torque_current of the leader_motor motor
        :rtype: float
        """
        return self.leader_motor_torque_current.value

    def get_applied_voltage(self) -> units.volt:
        """
        :returns: The applied voltage to the shooter.
        :rtype: volts
        """
        return self.left_motor.getThrottle() * RobotController.getBatteryVoltage()

    @feedback
    def get_supply_amps(self):
        return self.leader_motor_supply_amps.value

    """
    CONTROL METHODS
    """

    def set_velocity(self, velocity: units.rotations_per_second):
        """
        Drives the flywheel to the provided velocity setpoint.
        """
        self.control = self.velocity_request.with_velocity(velocity)
        self.req_velocity = velocity

    def coast(self) -> None:
        """
        Coasts the shooter.
        """
        self.control = self.coast_request

    def set_voltage(self, volts: units.volt):
        self.control = self.voltage_request(volts)

    def execute(self) -> None:
        """
        Sets the control of the right (leader) motor every robot iteration.
        """
        self.right_motor.set_control(self.control)

        # refresh all status signals
        BaseStatusSignal.refresh_all(
            self.leader_motor_velocity,
            self.leader_motor_torque_current,
            self.leader_motor_supply_amps,
        )
