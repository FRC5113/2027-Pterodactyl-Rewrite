from robotpy_fields import get_field

from components.odometry import Odometry
from components.swerve_drive import SwerveDrive
from lemonlib.simulation import LemonCameraSim, LemonVisionSim


class OdometrySim:
    def __init__(self, odometry: Odometry, swerve: SwerveDrive):
        self._get_pose = swerve.get_pose

        self.field = get_field(odometry.field_layout)
        self.vision_sim = LemonVisionSim(self.field)

        self.front_right_sim = LemonCameraSim(
            odometry.camera_front_right, self.field, 74.4, 60.0
        )
        self.vision_sim.add_camera(self.front_right_sim)

    def simulation_periodic(self):
        self.vision_sim.update(self._get_pose())
