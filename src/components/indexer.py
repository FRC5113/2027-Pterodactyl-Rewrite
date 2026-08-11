from phoenix6 import BaseStatusSignal, configs, controls, signals, units
from phoenix6.hardware import TalonFXS

from lemonlib.ctre import tryUntilOk
from modified_libs.magicbot import feedback, will_reset_to


class Indexer:

    conveyor_motor: TalonFXS
    conveyor_amps: units.ampere

    conveyor_control = will_reset_to(controls.CoastOut())

    def setup(self):
        self.config = configs.TalonFXSConfiguration()

        self.config.current_limits = (
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(self.conveyor_amps)
            .with_stator_current_limit_enable(True)
        )
        self.config.commutation = configs.CommutationConfigs().with_motor_arrangement(
            signals.MotorArrangementValue.BRUSHED_DC
        )
        self.config.motor_output = configs.MotorOutputConfigs().with_neutral_mode(
            signals.NeutralModeValue.COAST
        )

        tryUntilOk(5, lambda: self.conveyor_motor.configurator.apply(self.config))

        self.volt_control = controls.VoltageOut(0.0)
        self.throttle_control = controls.DutyCycleOut(0.0)

        self.conveyor_supply_amps = self.conveyor_motor.get_supply_current(False)

    """
    CONTROL METHODS
    """

    def set_voltage(self, voltage: units.volt):
        self.conveyor_control = self.volt_control.with_output(voltage)

    def set_throttle(self, throttle: float):
        self.conveyor_control = self.throttle_control.with_output(throttle)

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_supply_amps(self):
        return self.conveyor_supply_amps.value

    def execute(self):
        self.conveyor_motor.set_control(self.conveyor_control)

        BaseStatusSignal.refresh_all(self.conveyor_supply_amps)
