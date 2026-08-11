from phoenix6 import BaseStatusSignal, configs, controls, signals, units
from phoenix6.hardware import TalonFXS

from lemonlib.ctre import tryUntilOk
from modified_libs.magicbot import feedback, will_reset_to


class Kicker:
    left_motor: TalonFXS
    right_motor: TalonFXS
    amps: units.ampere

    kicker_control = will_reset_to(controls.CoastOut())

    def setup(self):
        self.config = configs.TalonFXSConfiguration()

        self.config.current_limits = (
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(self.amps)
            .with_stator_current_limit_enable(True)
        )
        self.config.commutation = configs.CommutationConfigs().with_motor_arrangement(
            signals.MotorArrangementValue.NEO550_JST
        )
        self.config.motor_output = configs.MotorOutputConfigs().with_neutral_mode(
            signals.NeutralModeValue.BRAKE
        )

        tryUntilOk(5, self.right_motor.configurator.apply(self.config))
        tryUntilOk(5, self.left_motor.configurator.apply(self.config))

        self.volt_control = controls.VoltageOut(0.0)
        self.throttle_control = controls.DutyCycleOut(0.0)
        self.follow_control = controls.Follower(
            self.right_motor.device_id, signals.MotorAlignmentValue.OPPOSED
        )

        self.left_motor.set_control(self.follow_control)

        self.right_motor_supply_amps = self.right_motor.get_supply_current(False)
        self.left_motor_supply_amps = self.left_motor.get_supply_current(False)

    """
    CONTROL METHODS
    """

    def set_voltage(self, voltage: units.volt):
        self.kicker_control = self.volt_control.with_output(voltage)

    def set_throttle(self, throttle: float):
        self.kicker_control = self.throttle_control.with_output(throttle)

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_kicker_amps(self):
        return self.right_motor_supply_amps.value + self.left_motor_supply_amps.value

    def execute(self):
        self.right_motor.set_control(self.kicker_control)

        BaseStatusSignal.refresh_all(
            self.left_motor_supply_amps, self.right_motor_supply_amps
        )
