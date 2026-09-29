from components.intake import Intake
from wpilib.simulation import SingleJointedArmSim, DutyCycleEncoderSim
from wpimath import DCMotor
import math
from wpilib import (
    Mechanism2d,
    MechanismLigament2d,
    MechanismObject2d,
    MechanismRoot2d,
    SmartDashboard,
)
from wpiutil import Color8Bit


class IntakeSim:

    def __init__(self, intake: Intake) -> None:
        self.intake = intake

        arm_length = 0.38

        self.arm_sim = SingleJointedArmSim(
            DCMotor(
                12.0, 7.91, 24.0, 5.0, 10.0 * (math.pi / 30), 2
            ),  # Snowblower specs
            1.0,  # gearing
            0.001,  # very much esimated moi and not at all accurate
            arm_length,  # esimate of arm length
            0.0,
            math.pi / 2,
            True,
            math.pi / 2,
        )

        self.mech = Mechanism2d(2.0, 1.5)

        # Ground Anchor Point 1: Bottom/Main Driver Pivot
        self.driver_root = self.mech.getRoot("DriverPivot", 0.5, 0.3)
        self.driver_ligament = self.driver_root.appendLigament(
            "DriverArm", arm_length, 0, 6, Color8Bit(0, 0, 255)
        )

        # Floating End Node: The Intake Collector Frame
        # Extends from the end of the driver arm
        self.intake_frame = self.driver_ligament.appendLigament(
            "IntakeHead", 0.15, 90, 8, Color8Bit(255, 0, 0)
        )

        # Ground Anchor Point 2: Top/Follower Pivot (Offset slightly on the chassis)
        self.follower_root = self.mech.getRoot("FollowerPivot", 0.5, 0.5)
        self.follower_ligament = self.follower_root.appendLigament(
            "FollowerArm", arm_length, 0, 4, Color8Bit(0, 255, 0)
        )

        SmartDashboard.putData("Intake Sim", self.mech)

        self.left_motor_sim = self.intake.left_motor.sim_state
        self.right_motor_sim = self.intake.right_motor.sim_state
        self.encoder_sim = DutyCycleEncoderSim(self.intake.encoder)

    def simulation_periodic(self):
        self.arm_sim.setInputVoltage(-self.right_motor_sim.motor_voltage)
        self.arm_sim.update(0.02)

        # Extract calculated angle from physics engine
        driver_angle_rad = self.arm_sim.getAngle()
        driver_angle_deg = math.degrees(driver_angle_rad)

        follower_angle_deg = driver_angle_deg

        intake_relative_angle = 90.0 - driver_angle_deg

        self.driver_ligament.setAngle(driver_angle_deg)
        self.follower_ligament.setAngle(follower_angle_deg)
        self.intake_frame.setAngle(intake_relative_angle)

        self.encoder_sim.set(
            ((intake_relative_angle / 360) + self.intake.ENCODER_OFFSET) % 1.0
        )
