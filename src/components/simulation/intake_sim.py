import math

import telemetry
from wpilib import (
    Mechanism2d,
)
from wpilib.simulation import DutyCycleEncoderSim, SingleJointedArmSim
from wpimath import DCMotor
from wpiutil import Color8Bit

from components.intake import Intake


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
            -math.pi,
            math.pi,
            True,
            math.pi / 2,
        )

        self.mech = Mechanism2d(2.0, 1.5)

        # Ground Anchor Point 1: Bottom/Main Driver Pivot
        self.driver_root = self.mech.get_root("DriverPivot", 0.5, 0.3)
        self.driver_ligament = self.driver_root.append_ligament(
            "DriverArm", arm_length, 0, 6, Color8Bit(0, 0, 255)
        )

        # Floating End Node: The Intake Collector Frame
        # Extends from the end of the driver arm
        self.intake_frame = self.driver_ligament.append_ligament(
            "IntakeHead", 0.15, 90, 8, Color8Bit(255, 0, 0)
        )

        # Ground Anchor Point 2: Top/Follower Pivot (Offset slightly on the chassis)
        self.follower_root = self.mech.get_root("FollowerPivot", 0.5, 0.5)
        self.follower_ligament = self.follower_root.append_ligament(
            "FollowerArm", arm_length, 0, 4, Color8Bit(0, 255, 0)
        )

        self.left_motor_sim = self.intake.left_motor.sim_state
        self.right_motor_sim = self.intake.right_motor.sim_state
        self.encoder_sim = DutyCycleEncoderSim(self.intake.encoder)

    def simulation_periodic(self):
        self.arm_sim.set_input_voltage(-self.right_motor_sim.motor_voltage)
        self.arm_sim.update(0.02)

        # Extract calculated angle from physics engine
        driver_angle_rad = self.arm_sim.get_angle()
        driver_angle_deg = math.degrees(driver_angle_rad)

        follower_angle_deg = driver_angle_deg

        intake_relative_angle = 90.0 - driver_angle_deg

        self.driver_ligament.set_angle(driver_angle_deg)
        self.follower_ligament.set_angle(follower_angle_deg)
        self.intake_frame.set_angle(intake_relative_angle)

        self.encoder_sim.set(
            ((intake_relative_angle / 360) + self.intake.ENCODER_OFFSET) % 1.0
        )

        telemetry.log("Intake Sim", self.mech)
