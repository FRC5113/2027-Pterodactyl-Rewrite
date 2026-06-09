import typing

from phoenix6 import unmanaged
from pyfrc.physics.core import PhysicsInterface
from wpilib import RobotController

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot

    def update_sim(self, now, tm_diff):
        # Keep Phoenix 6 devices enabled in sim
        unmanaged.feed_enable(100)

        # Phoenix 6 SwerveDrivetrain handles ALL motor, encoder, and Pigeon2
        # simulation internally — drive motors, steer motors, CANcoders, and
        # Pigeon2 yaw are all updated in one call.
        self.robot.drivetrain.drivetrain.update_sim_state(
            tm_diff,
            RobotController.getBatteryVoltage(),
        )
