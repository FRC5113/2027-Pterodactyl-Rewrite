import math

from photonlibpy import PhotonPoseEstimator
from robotpy_fields import Field,FieldId,FieldImage,get_field
from wpilib import DriverStation, Field2d,DriverStationBackend,RobotBase
from wpimath import Pose2d, Pose3d, Rotation2d
import telemetry

from components.swerve_drive import SwerveDrive
from lemonlib.vision import LemonCamera

class Odometry:
    drivetrain: SwerveDrive

    camera_front_right: LemonCamera

    field_layout: FieldId
    drivetrain: SwerveDrive

    def setup(self):
        cameras = (
            self.camera_front_right,
        )
        self._camera_estimator_pairs = tuple(
            (cam, PhotonPoseEstimator(get_field(self.field_layout), cam.camera_to_bot))
            for cam in cameras
        )

        self.linear_baseline_std = 0.10  # meters
        self.angular_baseline_std = math.radians(10)
        self.angular_baseline_std_sim = math.radians(30)


        # Stale timestamp tracking per camera
        self._last_timestamps: list[float] = [0.0] * len(self._camera_estimator_pairs)

        self.field = Field2d()

    def _compute_std_devs(
        self, avg_dist: float, tag_count: int, is_single_tag_gyro_fused: bool = False
    ) -> tuple[float, float, float]:
        """Compute standard deviations for a vision measurement."""
        angular_baseline = (
            self.angular_baseline_std_sim if RobotBase.is_simulation() else self.angular_baseline_std
        )
        std_factor = (avg_dist**2) / tag_count
        std_xy = self.linear_baseline_std * std_factor
        if is_single_tag_gyro_fused:
            # Gyro heading is far more trustworthy; tell Kalman filter to ignore vision heading
            std_rot = 1e6
        else:
            std_rot = angular_baseline * std_factor
        return (std_xy, std_xy, std_rot)

    def execute(self):
        disabled = DriverStationBackend.is_disabled()

        for cam_idx, (cam, pose_est) in enumerate(self._camera_estimator_pairs):
            cam.update()

            for res in cam.results:
                targets = res.getTargets()
                if not targets:
                    continue

                best_target = res.getBestTarget()
                if best_target and best_target.poseAmbiguity > 0.2:
                    continue

                # Try multi-tag estimation first (most accurate)
                pupdate = pose_est.estimateCoprocMultiTagPose(res)

                if pupdate is None:
                    pupdate = pose_est.estimateCoprocMultiTagPose(res)
                    if pupdate is None:
                        continue

                is_gyro_fused = False

                pose3d = pupdate.estimatedPose
                twod_pose = pose3d.to_pose2d()
                ts = pupdate.timestampSeconds

                # Compute std devs
                tag_count = len(targets)
                total_dist = sum(
                    t.getBestCameraToTarget().translation().norm() for t in targets
                )
                avg_dist = total_dist / tag_count
                if avg_dist > 2.0 and not disabled:
                    continue

                std_devs = self._compute_std_devs(avg_dist, tag_count, is_gyro_fused)

                self.drivetrain.addVisionPoseEstimate(twod_pose,ts,std_devs)
                self.field.set_robot_pose(twod_pose)
        telemetry.log("esitame", self.field)