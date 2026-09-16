from enum import Enum

from phoenix6 import BaseStatusSignal, configs, controls, signals, units
from phoenix6.hardware import TalonFX
from wpilib import DutyCycleEncoder

from lemonlib.ctre import tryUntilOk
from lemonlib.smart import SmartProfile
from modified_libs.magicbot import feedback, will_reset_to


class Arm_Angle(float, Enum):
    STOWED = 0.0
    DOWN = 90.0


class Intake:

    spin_motor: TalonFX
    arm_motor: TalonFX

    encoder: DutyCycleEncoder

    profile: SmartProfile

    spin_amps: units.ampere
    arm_amps: units.ampere

    target_angle = will_reset_to(Arm_Angle.DOWN.value)
    spin_control = will_reset_to(controls.StaticBrake())
    arm_voltage = will_reset_to(0.0)
    arm_manual = False

    def setup(self) -> None:
        self._config_arm_motors()
        self._config_spin_motor()

        self.spin_motor_supply_amps = self.spin_motor.get_supply_current(False)

        self.volt_control = controls.VoltageOut(0.0)
        self.throttle_control = controls.DutyCycleOut(0.0)

    def _config_arm_motors(self):
        self.arm_motor_config = configs.TalonFXConfiguration()

        self.arm_motor_config.current_limits.stator_current_limit = self.arm_amps
        self.arm_motor_config.current_limits.stator_current_limit_enable = True
        self.arm_motor_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE

        tryUntilOk(5, lambda: self.arm_motor.configurator.apply(self.arm_motor_config))

    def _config_spin_motor(self):
        # Configure motors
        spin_config = configs.TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        spin_config.current_limits.stator_current_limit = self.spin_amps
        spin_config.current_limits.stator_current_limit_enable = True
        tryUntilOk(5, lambda: self.spin_motor.configurator.apply(spin_config))

    """
    CONTROL METHODS
    """

    def set_arm_voltage(self, voltage: units.volt) -> None:
        self.arm_control = self.volt_control.with_output(voltage)
        self.arm_manual = True

    def set_wheel_voltage(self, voltage: units.volt) -> None:
        self.spin_control = self.volt_control.with_output(voltage)
        self.target_angle = Arm_Angle.DOWN.value

    def set_arm_throttle(self, throttle: float):
        self.arm_control = self.throttle_control.with_output(throttle)
        self.arm_manual = True

    def set_spin_throttle(self, throttle: float):
        self.spin_control = self.throttle_control.with_output(throttle)
        self.target_angle = Arm_Angle.DOWN.value

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
        angle = self.encoder.get() * 360
        if angle > 180:
            angle -= 360
        return angle

    @feedback
    def get_arm_position(self):
        return self.encoder.get()

    @feedback
    def get_requested_angle(self):
        return self.target_angle

    def on_enable(self):
        self.arm_controller = self.profile.create_arm_controller("Intake")

    def execute(self) -> None:
        if not self.arm_manual:
            self.arm_voltage = self.arm_controller.calculate(
                self.get_arm_angle(), self.target_angle
            )

        if (self.get_arm_angle() <= Arm_Angle.STOWED and self.arm_voltage > 0) or (
            self.get_arm_angle() >= Arm_Angle.DOWN and self.arm_voltage < 0
        ):
            self.arm_motor.set_control(self.volt_control.with_output(self.arm_voltage))

        self.spin_motor.set_control(self.spin_control)

        BaseStatusSignal.refresh_all(self.spin_motor_supply_amps)
