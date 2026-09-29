from enum import Enum

from phoenix6 import BaseStatusSignal, configs, controls, signals, units
from phoenix6.hardware import TalonFX, TalonFXS
from wpilib import DutyCycleEncoder

from lemonlib.ctre import tryUntilOk
from lemonlib.smart import SmartProfile, SmartPreference
from lemonlib.util import clamp
from modified_libs.magicbot import feedback, will_reset_to


class Intake_Angle(float, Enum):
    STOWED = 2.0
    DOWN = 90.0


class Intake:

    spin_motor: TalonFX
    right_motor: TalonFXS
    left_motor: TalonFXS

    encoder: DutyCycleEncoder

    profile: SmartProfile

    spin_amps: units.ampere
    arm_amps: units.ampere
    tolerance: float

    target_angle = will_reset_to(Intake_Angle.DOWN)
    spin_control = will_reset_to(controls.StaticBrake())
    arm_voltage = will_reset_to(0.0)
    arm_manual = False

    up_kP = SmartPreference(12.0)
    down_kP = SmartPreference(4.0)

    def setup(self) -> None:
        self._config_arm_motors()
        self._config_spin_motor()

        self.spin_motor_supply_amps = self.spin_motor.get_supply_current(False)

        self.volt_control = controls.VoltageOut(0.0)
        self.throttle_control = controls.DutyCycleOut(0.0)

        # self.encoder.setInverted(True)

        self.arm_follower_control = controls.Follower(
            self.right_motor.device_id, signals.MotorAlignmentValue.ALIGNED
        )

    def _config_arm_motors(self):
        self.arm_motor_config = configs.TalonFXSConfiguration()

        self.arm_motor_config.current_limits.stator_current_limit = self.arm_amps
        self.arm_motor_config.current_limits.stator_current_limit_enable = True
        self.arm_motor_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE

        self.arm_motor_config.commutation.motor_arrangement = (
            signals.MotorArrangementValue.BRUSHED_DC
        )

        tryUntilOk(5, lambda: self.left_motor.configurator.apply(self.arm_motor_config))
        tryUntilOk(
            5, lambda: self.right_motor.configurator.apply(self.arm_motor_config)
        )

    def _config_spin_motor(self):
        # Configure motors
        spin_config = configs.TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        spin_config.motor_output.inverted = (
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        spin_config.current_limits.stator_current_limit = self.spin_amps
        spin_config.current_limits.stator_current_limit_enable = True
        tryUntilOk(5, lambda: self.spin_motor.configurator.apply(spin_config))

    def _p_controller(self, setpoint: float, position: float) -> float:
        """
        Pure p controller that has diffrent gains for going up and down cause gravity
        """

        error = clamp((setpoint - position) / 90, -1.0, 1.0)

        if abs(error) < self.tolerance:
            return 0.0

        # 0.0 is up and 90 is down so if we want to go down error will be positive
        if error < 0.0:
            return self.down_kP * error
        return self.up_kP * error

    """
    CONTROL METHODS
    """

    def set_arm_voltage(self, voltage: units.volt) -> None:
        self.arm_control = self.volt_control.with_output(voltage)
        self.arm_voltage = voltage
        self.arm_manual = True

    def set_wheel_voltage(self, voltage: units.volt) -> None:
        self.spin_control = self.volt_control.with_output(voltage)
        self.target_angle = Intake_Angle.DOWN

    def set_arm_throttle(self, throttle: float):
        self.arm_control = self.throttle_control.with_output(throttle)
        self.arm_manual = True

    def set_spin_throttle(self, throttle: float):
        self.spin_control = self.throttle_control.with_output(throttle)
        self.target_angle = Intake_Angle.DOWN

    def set_arm_angle(self, angle):
        self.target_angle = angle
        self.arm_manual = False

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_spin_supply_amps(self):
        return self.spin_motor_supply_amps.value

    @feedback
    def get_arm_angle(self):
        """Return the angle of the hinge normalized to [-180,180].
        An angle of 0 refers to the intake in the up/stowed position.
        """
        angle = self.get_arm_position() * 360
        if angle > 180:
            angle -= 360
        return angle

    @feedback
    def get_arm_position(self):
        return (self.encoder.get() - 0.9434523809523809) % 1

    @feedback
    def get_requested_angle(self):
        return self.target_angle

    @feedback
    def get_arm_voltage(self):
        return self.arm_voltage

    def execute(self) -> None:
        arm_angle = self.get_arm_angle()

        if not self.arm_manual:
            self.arm_voltage = -self._p_controller(arm_angle, self.target_angle)

        if (arm_angle <= Intake_Angle.STOWED.value and self.arm_voltage < 0.0) or (
            arm_angle >= Intake_Angle.DOWN.value and self.arm_voltage > 0.0
        ):
            self.arm_voltage = 0.0

        self.right_motor.set_control(self.volt_control.with_output(self.arm_voltage))
        self.left_motor.set_control(self.volt_control.with_output(self.arm_voltage))

        self.spin_motor.set_control(self.spin_control)

        BaseStatusSignal.refresh_all(self.spin_motor_supply_amps)
