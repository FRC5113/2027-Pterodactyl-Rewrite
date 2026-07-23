from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, hardware, swerve, units, utils
from phoenix6.swerve import requests
from wpilib import (
    Alliance,
    DriverStationBackend,
    Mechanism2d,
    MechanismLigament2d,
    SmartDashboard,
)
from wpimath import (
    ChassisVelocities,
    Pose2d,
    Rotation2d,
    SwerveModulePosition,
    SwerveModuleVelocity,
)
from wpimath.units import meters_per_second, seconds
from wpiutil import Color, Color8Bit

from generated.tuner_constants import TunerConstants
from lemonlib.smart import SmartProfile
from lemonlib.util import Alert, AlertType
from magicbotmod import will_reset_to


class SwerveDrive:  # (Sendable):
    """
    Swerve drive using the Phoenix 6 Swerve API (SwerveDrivetrain).
    """

    max_speed: meters_per_second
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

    # def __init__(self) -> None:
    #     Sendable.__init__(self)
    #     SmartDashboard.putData("Swerve Drive", self)

    def setup(self) -> None:
        self.telemetry = Telemetry(self.max_speed)

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

        # Keep track if we've ever applied the operator perspective before or not
        self.has_applied_operator_perspective = False

        self.state = self.drivetrain.get_state_copy()

        self.pending_request = requests.Idle()

        self.drivetrain.register_telemetry(
            lambda state: self.telemetry.telemeterize(state)
        )

    def on_enable(self) -> None:
        self._operator_perspective_update()  # Try to apply the operator perspective immediately on enable

    """
    INFORMATIONAL METHODS
    """

    def get_module_states(
        self,
    ) -> swerve.SwerveDrivetrain.SwerveDriveState:
        """
        :returns: A list of `SwerveModuleState` objects for each swerve module.
        `rtype`: SwerveModuleState
        """
        return self.state

    def get_pose(self) -> Pose2d:
        """
        :returns: The estimated drivetrain pose from the swerve state.
        :rtype: Pose2d
        """
        return self.drivetrain.get_state_copy().pose

    def get_chassis(self) -> ChassisVelocities:
        """
        :returns: The applied drivetrain chassis from the swerve state.
        :rtype: ChassisVelocities
        """

        return self.state.velocity

    """
    TELEMETRY
    """
    # def initSendable(self, builder: SendableBuilder) -> None:
    #     """
    #     Called during robot initialization to initialize the sendable object.
    #     """
    #     builder.setSmartDashboardType("SwerveDrive")
    #     builder.addDoubleProperty(
    #         "Robot Angle",
    #         lambda: self.state.pose.rotation().degrees(),
    #         lambda _: None,
    #     )
    #     for i, label in enumerate(
    #         ("Front Left", "Front Right", "Back Left", "Back Right")
    #     ):
    #         _i = i

    #         def _vel(idx: int = _i):
    #             return self.state.module_velocities[idx].velocity

    #         def _ang(idx: int = _i):
    #             return self.state.module_positions[idx].angle.degrees()

    #         builder.addDoubleProperty(f"{label} Velocity", _vel, lambda _: None)
    #         builder.addDoubleProperty(f"{label} Angle", _ang, lambda _: None)

    #     builder.addDoubleArrayProperty(
    #         "Swerve Setpoints",
    #         lambda: [
    #             value
    #             for state in self.state.module_targets
    #             for value in (state.angle.degrees(), state.velocity)
    #         ],
    #         lambda _: None,
    #     )
    #     builder.addDoubleArrayProperty(
    #         "Swerve Measurements",
    #         lambda: [
    #             value
    #             for state in self.state.module_velocities
    #             for value in (state.angle.degrees(), state.velocity)
    #         ],
    #         lambda _: None,
    #     )

    """
    CONTROL METHODS
    """

    def apply_control(self, control: requests.SwerveRequest) -> None:
        """
        Apply a swerve request to the drivetrain. This will be applied on the next call to execute().
        """
        self.stopped = False
        self.pending_request = control

    def addVisionPoseEstimate(
        self,
        pose: Pose2d,
        timestamp: seconds,
        std_devs: tuple[float, float, float],
    ) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise.
        """
        self.drivetrain.add_vision_measurement(
            pose, utils.get_current_time_seconds(timestamp), std_devs
        )

    def curr_direction_forward(self) -> None:
        """
        Resets the direction the robot is currently facing.
        """
        self.drivetrain.seed_field_centric()

    def set_angle_relative(self, angle: Rotation2d):
        """
        If the robot is facing some other angle relative to the drivers forward direction.
        For example, if the robot is facing left, then pass in an angle of +90 degrees (counter-clockwise).
        """
        self.drivetrain.set_operator_perspective_forward(angle)

    def reset_heading_to_blue_origin(self) -> None:
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

    def _operator_perspective_update(self) -> None:
        """
        Periodically try to apply the operator perspective
        if we haven't yet or if we're currently disabled.
        """
        if (
            not self.has_applied_operator_perspective
            or DriverStationBackend.isDisabled()
        ):
            alliance_color = DriverStationBackend.getAlliance()
            if alliance_color is not None:
                self.drivetrain.set_operator_perspective_forward(
                    self.RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == Alliance.RED
                    else self.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self.has_applied_operator_perspective = True

    def execute(self) -> None:
        """
        Requests the pending request to the swerve drivetrain every robot iteration.
        """
        self._operator_perspective_update()

        if self.stopped:
            self.drivetrain.set_control(requests.Idle())
            return

        self.drivetrain.set_control(self.pending_request)

        self.state = self.drivetrain.get_state_copy()


class Telemetry:
    def __init__(self, max_speed: units.meters_per_second):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed

        # What to publish over networktables for telemetry
        self._inst = NetworkTableInstance.getDefault()

        # Robot swerve drive state
        self._drive_state_table = self._inst.getTable("DriveState")
        self._drive_pose = self._drive_state_table.getStructTopic(
            "Pose", Pose2d
        ).publish()
        self._drive_velocity = self._drive_state_table.getStructTopic(
            "Velocity", ChassisVelocities
        ).publish()
        self._drive_module_positions = self._drive_state_table.getStructArrayTopic(
            "ModulePositions", SwerveModulePosition
        ).publish()
        self._drive_module_velocities = self._drive_state_table.getStructArrayTopic(
            "ModuleVelocities", SwerveModuleVelocity
        ).publish()
        self._drive_module_targets = self._drive_state_table.getStructArrayTopic(
            "ModuleTargets", SwerveModuleVelocity
        ).publish()
        self._drive_timestamp = self._drive_state_table.getDoubleTopic(
            "Timestamp"
        ).publish()
        self._drive_odometry_frequency = self._drive_state_table.getDoubleTopic(
            "OdometryFrequency"
        ).publish()

        # Robot pose for field positioning
        self._table = self._inst.getTable("Pose")
        self._field_pub = self._table.getDoubleArrayTopic("Robot").publish()
        self._field_type_pub = self._table.getStringTopic(".type").publish()

        # Mechanisms to represent the swerve module states
        self._module_mechanisms: list[Mechanism2d] = [
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
        ]
        # A direction and length changing ligament for speed representation
        self._module_speeds: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
            self._module_mechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
            self._module_mechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
            self._module_mechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
        ]

        # Set up the module state Mechanism2d telemetry
        for i, module_mechanism in enumerate(self._module_mechanisms):
            SmartDashboard.putData(f"Module {i}", module_mechanism)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
        """
        # Telemeterize the swerve drive state
        self._drive_pose.set(state.pose)
        self._drive_velocity.set(state.velocity)
        self._drive_module_positions.set(state.module_positions)
        self._drive_module_velocities.set(state.module_velocities)
        self._drive_module_targets.set(state.module_targets)
        self._drive_timestamp.set(state.timestamp)
        self._drive_odometry_frequency.set(1.0 / state.odometry_period)

        # Also write to log file
        SignalLogger.write_struct("DriveState/Pose", Pose2d, state.pose)
        SignalLogger.write_struct(
            "DriveState/Velocity", ChassisVelocities, state.velocity
        )
        SignalLogger.write_struct_array(
            "DriveState/ModulePositions", SwerveModulePosition, state.module_positions
        )
        SignalLogger.write_struct_array(
            "DriveState/ModuleVelocities", SwerveModuleVelocity, state.module_velocities
        )
        SignalLogger.write_struct_array(
            "DriveState/ModuleTargets", SwerveModuleVelocity, state.module_targets
        )
        SignalLogger.write_double(
            "DriveState/OdometryPeriod", state.odometry_period, "seconds"
        )
        SignalLogger.write_integer("DriveState/FailedDaqs", state.failed_daqs)

        # Telemeterize the pose to a Field2d
        self._field_type_pub.set("Field2d")

        pose_array: list = [state.pose.x, state.pose.y, state.pose.rotation().degrees()]
        self._field_pub.set(pose_array)

        # Telemeterize each module state to a Mechanism2d
        for i, module_state in enumerate(state.module_velocities):
            self._module_speeds[i].setAngle(module_state.angle.degrees())
            self._module_directions[i].setAngle(module_state.angle.degrees())
            self._module_speeds[i].setLength(
                module_state.velocity / (2 * self._max_speed)
            )
