from components.intake import Intake
from controllers.drive_control import DriveControl
from controllers.shooter_controller import ShooterController


class example_auto:
    MODE_NAME = "Just to show"

    drive_control: DriveControl
    shooter_controller: ShooterController
    intake: Intake

    """
    Very very very poorly done but just to show its possible
    """

    def on_enable(self) -> None:
        self.drive_back_time = 2.0  # Seconds
        self.intake_down_time = 1.5 + self.drive_back_time  # Seconds

    def on_iteration(self, tm: float) -> None:
        if tm < self.drive_back_time:
            self.drive_control.request_drive_robot(-1.0, 0.0, 0.0)  # Drives backward

        elif tm < self.intake_down_time:
            self.intake.set_arm_voltage(8.0)  # Lowers Intake

        else:
            self.shooter_controller.request_shot(46.0)  # Shoots
            self.intake.set_spin_throttle(0.8)  # Spins intake to get any stuck balls
