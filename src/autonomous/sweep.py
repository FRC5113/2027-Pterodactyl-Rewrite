import choreo
import choreo.util
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import Alliance, DriverStationBackend, RobotBase

from components.intake import Intake
from components.swerve_drive import SwerveDrive
from controllers.drive_control import DriveControl
from controllers.score_controller import ScoreController


class RightSweep(AutonomousStateMachine):
    MODE_NAME = "Right Sweep"

    score_controller: ScoreController
    drive_control: DriveControl
    drivetrain: SwerveDrive
    intake: Intake

    def setup(self):
        _load_traj = choreo.load_swerve_trajectory

        self.rt_intake = _load_traj("rt_intake")
        self.end_of_rt_intake_to_shoot = _load_traj("end_of_rt_intake_to_shoot")

    def on_enable(self) -> None:
        if RobotBase.is_simulation():
            self.drivetrain.set_pose(
                self.rt_intake.get_initial_pose(
                    DriverStationBackend.get_alliance() == Alliance.RED
                )
            )
        return super().on_enable()

    @state(first=True)
    def goto_intake(self, state_tm):
        self.drive_control.request_follow_trajectory(self.rt_intake, state_tm)

        if self.drive_control.get_following_trajectory() == False:
            self.intake.set_arm_voltage(8.0)
            self.next_state("lower_intake")

    @timed_state(duration=1, next_state="intaking")
    def lower_intake(self):
        self.intake.set_arm_voltage(8.0)

    @timed_state(duration=3.0, next_state="return_hub")
    def intaking(self):
        self.drive_control.request_drive_robot(1.0, 0.0, 0.0)
        self.intake.set_arm_voltage(8.0)
        self.intake.set_spin_throttle(0.8)

    @state
    def return_hub(self, state_tm):
        self.drive_control.request_follow_trajectory(
            self.end_of_rt_intake_to_shoot, state_tm
        )

        if self.drive_control.get_following_trajectory() == False:
            self.next_state("shooting")

    @timed_state(duration=20.0)
    def shooting(self):
        self.score_controller.request_score()
        self.intake.set_spin_throttle(0.8)


class LeftSweep(AutonomousStateMachine):
    MODE_NAME = "Left Sweep"

    score_controller: ScoreController
    drive_control: DriveControl
    drivetrain: SwerveDrive
    intake: Intake

    def setup(self):
        _load_traj = choreo.load_swerve_trajectory

        self.lt_intake = _load_traj("lt_intake")
        self.end_of_lt_intake_to_shoot = _load_traj("end_of_lt_intake_to_shoot")

    def on_enable(self) -> None:
        if RobotBase.is_simulation():
            self.drivetrain.set_pose(
                self.lt_intake.get_initial_pose(
                    DriverStationBackend.get_alliance() == Alliance.RED
                )
            )
        return super().on_enable()

    @state(first=True)
    def goto_intake(self, state_tm):
        self.drive_control.request_follow_trajectory(self.lt_intake, state_tm)

        if self.drive_control.get_following_trajectory() == False:
            self.intake.set_arm_voltage(8.0)
            self.next_state("lower_intake")

    @timed_state(duration=1, next_state="intaking")
    def lower_intake(self):
        self.intake.set_arm_voltage(8.0)

    @timed_state(duration=3.0, next_state="return_hub")
    def intaking(self):
        self.drive_control.request_drive_robot(1.0, 0.0, 0.0)
        self.intake.set_arm_voltage(8.0)
        self.intake.set_spin_throttle(0.8)

    @state
    def return_hub(self, state_tm):
        self.drive_control.request_follow_trajectory(
            self.end_of_lt_intake_to_shoot, state_tm
        )

        if self.drive_control.get_following_trajectory() == False:
            self.next_state("shooting")

    @timed_state(duration=20.0)
    def shooting(self):
        self.score_controller.request_score()
        self.intake.set_spin_throttle(0.8)
