from phoenix6 import BaseStatusSignal, configs, controls, signals, units
from phoenix6.hardware import TalonFX, TalonFXS

from lemonlib.ctre import tryUntilOk
from modified_libs.magicbot import feedback, will_reset_to


class Intake:

    spin_motor: TalonFX
    right_motor: TalonFXS
    left_motor: TalonFXS

    spin_amps: units.ampere
    arm_amps: units.ampere

    arm_control = will_reset_to(controls.CoastOut())
    spin_control = will_reset_to(controls.StaticBrake())

    def setup(self) -> None:
        self._config_arm_motors()
        self._config_spin_motor()

        self.spin_motor_supply_amps = self.spin_motor.get_supply_current(False)

        self.volt_control = controls.VoltageOut(0.0)
        self.throttle_control = controls.DutyCycleOut(0.0)

        self.arm_follower_control = controls.Follower(
            self.right_motor.device_id, signals.MotorAlignmentValue.OPPOSED
        )

    def _config_arm_motors(self):
        self.arm_motor_config = configs.TalonFXSConfiguration()

        self.arm_motor_config.current_limits.stator_current_limit = self.arm_amps
        self.arm_motor_config.current_limits.stator_current_limit_enable = True

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
        spin_config.current_limits.stator_current_limit = self.spin_amps
        spin_config.current_limits.stator_current_limit_enable = True
        tryUntilOk(5, lambda: self.spin_motor.configurator.apply(spin_config))

    """
    CONTROL METHODS
    """

    def set_arm_voltage(self, voltage: units.volt) -> None:
        self.arm_control = self.volt_control.with_output(voltage)

    def set_wheel_voltage(self, voltage: units.volt) -> None:
        self.spin_control = self.volt_control.with_output(voltage)

    def set_arm_throttle(self, throttle: float):
        self.arm_control = self.throttle_control.with_output(throttle)

    def set_spin_throttle(self, throttle: float):
        self.spin_control = self.throttle_control.with_output(throttle)

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_spin_supply_amps(self):
        return self.spin_motor_supply_amps.value

    def execute(self) -> None:
        self.right_motor.set_control(self.arm_control)
        self.spin_motor.set_control(self.spin_control)

        BaseStatusSignal.refresh_all(self.spin_motor_supply_amps)
