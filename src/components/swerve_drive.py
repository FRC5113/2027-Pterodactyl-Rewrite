from magicbot import will_reset_to
from phoenix6 import SignalLogger, hardware, swerve, utils
from phoenix6.swerve import requests
from wpilib import DriverStation, SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    SwerveModuleState,
)
from wpiutil import Sendable, SendableBuilder

from generated.tuner_constants import TunerConstants
from lemonlib.smart import SmartProfile
from lemonlib.util import Alert, AlertType


class SwerveDrive(Sendable):
    """
    Swerve drive using the Phoenix 6 Swerve API (SwerveDrivetrain).
    """

    max_speed: units.meters_per_second
    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    stopped = will_reset_to(True)

    BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(
        0
    )  # Blue alliance sees forward as 0 degrees (toward red alliance wall)
    RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(
        180
    )  # Red alliance sees forward as 180 degrees (toward blue alliance wall)

    """
    INITIALIZATION
    """

    def __init__(self) -> None:
        self.state = None
        Sendable.__init__(self)
        SmartDashboard.putData("Swerve Drive", self)

    def setup(self) -> None:
        # Build the Phoenix 6 SwerveDrivetrain — it creates all hardware internally
        self.drivetrain = swerve.SwerveDrivetrain(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            TunerConstants.drivetrain_constants,
            [
                TunerConstants.front_left,
                TunerConstants.front_right,
                TunerConstants.back_left,
                TunerConstants.back_right,
            ],
        )

        self.pigeon_alert = Alert(
            "Pigeon heading has been reset.", AlertType.INFO, timeout=3.0
        )

        self.has_applied_operator_perspective = False  # Keep track if we've ever applied the operator perspective before or not

        self.state = self.drivetrain.get_state_copy()

        self.drivetrain.register_telemetry(self.telemeterize)

    def on_enable(self):
        self._operator_perspective_update()  # Try to apply the operator perspective immediately on enable

    """
    INFORMATIONAL METHODS
    """

    def get_module_states(
        self,
    ) -> list[SwerveModuleState]:
        return self.state.module_states

    """
    TELEMETRY
    """

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SignalLogger.
        """

        # Not in phoenix 6 yet, but when we add it, we can telemeterize the full state like this:
        # SignalLogger.write_struct("DriveState/Pose", Pose2d, state.pose)
        # SignalLogger.write_struct("DriveState/Speeds", ChassisSpeeds, state.speeds)
        # SignalLogger.write_struct_array(
        #     "DriveState/ModuleStates", SwerveModuleState, state.module_states
        # )
        # SignalLogger.write_struct_array(
        #     "DriveState/ModuleTargets", SwerveModuleState, state.module_targets
        # )
        # SignalLogger.write_struct_array(
        #     "DriveState/ModulePositions", SwerveModulePosition, state.module_positions
        # )

        SignalLogger.write_double(
            "DriveState/OdometryPeriod", state.odometry_period, "seconds"
        )
        SignalLogger.write_integer("DriveState/FailedDaqs", state.failed_daqs)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SwerveDrive")
        if self.state is None:
            return
        builder.addDoubleProperty(
            "Robot Angle",
            lambda: self.state.pose.rotation().degrees(),
            lambda _: None,
        )
        for i, label in enumerate(
            ("Front Left", "Front Right", "Back Left", "Back Right")
        ):
            _i = i

            def _vel(idx=_i):
                return self.state.module_states[idx].speed * 5

            def _ang(idx=_i):
                return self.state.module_states[idx].angle.degrees()

            builder.addDoubleProperty(f"{label} Velocity", _vel, lambda _: None)
            builder.addDoubleProperty(f"{label} Angle", _ang, lambda _: None)

        if self.state.module_targets is not None:
            swerve_setpoints = []
            for state in self.state.module_targets:
                swerve_setpoints += [state.angle.degrees(), state.speed]
            builder.addDoubleArrayProperty(
                "Swerve Setpoints", lambda: swerve_setpoints, lambda _: None
            )

        if self.state.module_states is not None:
            swerve_measurements = []
            for ms in self.state.module_states:
                swerve_measurements += [ms.angle.degrees(), ms.speed]
            builder.addDoubleArrayProperty(
                "Swerve Measurements", lambda: swerve_measurements, lambda _: None
            )

    """
    CONTROL METHODS
    """

    def apply_control(self, control: requests.SwerveRequest) -> None:
        """Apply a swerve request to the drivetrain. This will be applied on the next call to execute()."""
        self.stopped = False
        self.pending_request = control

    def addVisionPoseEstimate(
        self,
        pose: Pose2d,
        timestamp: units.seconds,
        std_devs: tuple[float, float, float],
    ) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise.
        """
        self.drivetrain.add_vision_measurement(
            pose, utils.fpga_to_current_time(timestamp), std_devs
        )

    def curr_direction_forward(self):
        """
        Resets the headin got the direction the robot is currently facing.
        """
        self.drivetrain.seed_field_centric()

    def set_angle_relative(self, angle: Rotation2d):
        """
        If the robot is facing some other angle relative to the driver’s forward direction.
        For example, if the robot is facing left, then pass in an angle of +90 degrees (counter-clockwise).
        """

        self.drivetrain.set_operator_perspective_forward(angle)

    def set_blue_alliance_perspective(self):
        """
        When using a path planning library such as PathPlanner or Choreo, the paths often operate using the BlueAlliancePerspective
        Vision libraries similarly often operate using a BlueAlliancePerspective heading.
        """

        self.drivetrain.reset_rotation(Rotation2d())

    def set_pose(self, pose: Pose2d):
        """
        When using a path planning library such as PathPlanner or Choreo, the paths often reset the robot’s pose at the start of the path.
        Vision libraries similarly often operate using a pose.
        """

        self.drivetrain.reset_pose(pose)

    """
    EXECUTE
    """

    def _operator_perspective_update(self):
        """
        Periodically try to apply the operator perspective
        if we haven't yet or if we're currently disabled.
        """
        if not self.has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.drivetrain.set_operator_perspective_forward(
                    self.RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

    def execute(self) -> None:
        self._operator_perspective_update()

        self.state = self.drivetrain.get_state_copy()

        if self.stopped:
            self.drivetrain.set_control(requests.Idle())
            return

        self.drivetrain.set_control(self.pending_request)
