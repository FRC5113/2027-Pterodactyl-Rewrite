from magicbot import AutonomousStateMachine, timed_state

from components.intake import Intake
from controllers.drive_control import DriveControl
from controllers.shooter_controller import ShooterController


class hard_code_shoot(AutonomousStateMachine):
    MODE_NAME = "Hard Code Shoot"

    drive_control: DriveControl
    shooter_controller: ShooterController
    intake: Intake

    @timed_state(duration=1.5, next_state="intake_out", first=True)
    def move_back(self):
        self.drive_control.request_drive_robot(-1.0, 0.0, 0.0)
        self.intake.set_arm_voltage(8.0)

    @timed_state(duration=16.5)
    def hard_shoot(self):
        self.shooter_controller.request_shot(45.5)
        self.intake.set_spin_throttle(0.8)

    @timed_state(duration=0.5, next_state="hard_shoot")
    def intake_out(self):
        self.intake.set_arm_voltage(8.0)


class hard_code_shoot_angled(AutonomousStateMachine):
    MODE_NAME = "Hard Code Shoot angled"
    DEFAULT = True

    drive_control: DriveControl
    shooter_controller: ShooterController
    intake: Intake

    @timed_state(duration=1.5, next_state="intake_out", first=True)
    def move_back(self):
        self.drive_control.request_drive_robot(-1.0, 0.0, 0.0)
        self.intake.set_arm_voltage(8.0)

    @timed_state(duration=16.5)
    def hard_shoot(self):
        self.shooter_controller.request_shot(46.0)
        self.intake.set_spin_throttle(0.8)

    @timed_state(duration=0.5, next_state="hard_shoot")
    def intake_out(self):
        self.intake.set_arm_voltage(8.0)
