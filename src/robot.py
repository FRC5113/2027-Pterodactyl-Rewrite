import wpilib
import json
import telemetry
import tunables
import constants as constants
from components.intake import Intake
from lemonlib import LemonInput
from components.dispenser import Controller
from components.swerve import SwerveDrive
if constants.ODOMETRY:
    from components.odometry import Odometry
from components.auto import AutoContext, AutoRunner
from smartunits import (
    meters_per_second, radians_per_second, volts, AngularVelocity,
)
from smartunits import LinearVelocity as Velocity
import math
from lemonlib.util import curve, AlertManager, AlertType
from pathlib import Path
from wpimath import Pose2d, Rotation2d
from wpiutil import Color8Bit
if constants.SIM:
    from lemonlib.simulation import KrakenSim
OFF_COLOR = Color8Bit(120, 120, 120)
ON_COLOR = Color8Bit(0, 200, 0)
class MyRobot(wpilib.TimedRobot):
    intake: Intake
    shot_controller: Controller
    swerve: SwerveDrive
    odometry: Odometry

    def __init__(self):
        super().__init__()
        self.MAX_LINEAR_VELOCITY : Velocity = meters_per_second.of(3)
        self.MAX_ANGULAR_VELOCITY : AngularVelocity = radians_per_second.of(6)

        self.intake = Intake()
        self.shot_controller = Controller()
        self.swerve = SwerveDrive()
        self.odometry = Odometry(self.swerve)
        self.DEADBAND = 0.1 #arbitrary

        # auto selector: every trajectory in deploy/choreo, published as "Auto"
        self.auto_chooser = tunables.Selectable()
        for i, traj in enumerate(sorted((Path(wpilib.get_deploy_directory()) / "choreo").glob("*.traj"))):
            if i == 0:
                self.auto_chooser.add_default(traj.stem, str(traj))
            else:
                self.auto_chooser.add(traj.stem, str(traj))
        tunables.publish("Auto", self.auto_chooser)
        self.field = wpilib.Field2d()
        self.loop_count = 0
        if constants.SIM:
            # one bar per mechanism, each published under its own name; grows and turns green while it runs
            self.mechs = {name: wpilib.Mechanism2d(3, 1) for name in ("Intake", "Indexer", "Shooter")}
            self.mech_bars = {
                name: mech.get_root(name, 0.25, 0.5).append_ligament(name, 0.1, 0, 20, OFF_COLOR)
                for name, mech in self.mechs.items()
            }

            self.actual_poses = []
            self.field_table = telemetry.get_table("Field")
            self.sim_table = telemetry.get_table("Sim")
        if wpilib.RobotBase.is_simulation() and constants.SIM:
            # free-spinning rotor sims so current limits don't clamp the voltage readouts
            self.mech_sims = {
                name: KrakenSim(motor, 0.01 if name == "Shooter/left" else 0.001, 1.0)
                for name, motor in self._sim_motors().items()
            }

    def _sim_motors(self):
        indexer = self.shot_controller.indexer
        return {
            "Intake/spin": self.intake.spin_motor,
            "Intake/arm": self.intake.right_motor,
            "Indexer/left_kicker": indexer.left_kicker_motor,
            "Indexer/right_kicker": indexer.right_kicker_motor,
            "Indexer/conveyor": indexer.conveyor_motor,
            "Shooter/left": self.shot_controller.shooter.left_motor,
        }

    def robot_periodic(self):
        pose = self.swerve.get_pose()
        self.field.set_robot_pose(pose)
        if self.is_autonomous_enabled() and self.loop_count % 3 == 0:  # breadcrumb every 60 ms
            self.actual_poses = (self.actual_poses + [pose])[-85:]
            self.field.get_object("Actual").set_poses(self.actual_poses)
        self.loop_count += 1
        if constants.SIM:
            self.field_table.log("Field", self.field)

    def simulation_periodic(self):
        self.swerve.sim_update(0.02, wpilib.RobotController.get_battery_voltage())
        for mech_sim in self.mech_sims.values():
            mech_sim.update(0.02)
        for name, motor in self._sim_motors().items():
            self.sim_table.log(f"{name}_volts", float(motor.sim_state.motor_voltage))
        shooter = self.shot_controller.shooter.left_motor
        self.sim_table.log("Shooter/velocity_rps", abs(shooter.get_velocity().value_as_double))
        shooter_level = abs(shooter.get_velocity().value_as_double) / self.shot_controller.MAX_VELOCITY
        levels = {
            "Intake": abs(self.intake.spin_motor.sim_state.motor_voltage) / 12,
            "Indexer": abs(self.shot_controller.indexer.conveyor_motor.sim_state.motor_voltage) / 12,
            "Shooter": shooter_level,
        }
        for name, level in levels.items():
            self.mech_bars[name].set_length(max(0.1, 2.5 * min(level, 1.0)))
            self.mech_bars[name].set_color(ON_COLOR if level > 0.2 else OFF_COLOR)
        for name, mech in self.mechs.items():
            self.sim_table.log(f"Mechanisms/{name}", mech)
        self.sim_table.log("Shooter/shooting", bool(self.shot_controller.should_shoot or self.shot_controller.should_force_shoot))
    def teleop_init(self):
        self.primary = LemonInput(0)  # Gotta create on telop init so we know that driver station is
        self.secondary = LemonInput(1)  # connected to robot and to get most up to date info
    def robot_init(self):
        self.swerve.reset_heading()
    def teleop_periodic(self):
        self.odometry.execute() #prevent datalag

        # <Intake>
        # rollers
        if self.secondary.getLeftTriggerAxis() >= 0.8:
            self.intake.set_voltage(volts.of(-10.0))
        elif self.secondary.getLeftBumper():
            self.intake.set_voltage(volts.of(10.0))
        else:
            self.intake.set_voltage(volts.of(0.0))
        # hinge
        if self.secondary.getXButton():
            
            self.intake.set_arm_voltage(volts.of(-10.0))
        elif self.secondary.getBButton():
            self.intake.set_arm_voltage(volts.of(6.0))
        # </Intake>

        # <Shooter>
        if self.secondary.getAButton():#TODO: Make sure button matches what Ryan expects. 
            self.shot_controller.force_shoot()
        elif self.secondary.getRightTriggerAxis() >= 0.8:
            self.shot_controller.start_shoot(self.odometry.get_target_distance())
        else:
            self.shot_controller.stop_shoot()
        # </Shooter>
        
        # <Swerve>
        self.curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x,
            0.0,
            deadband=0.1,
            max_mag=1.0
        )
        SCALE_FACTOR = 1.0
        if self.primary.getRightTriggerAxis() >= 0.8 and self.primary.getLeftTriggerAxis() >= 0.8:
            SCALE_FACTOR = 0.25
        elif self.primary.getRightTriggerAxis() >= 0.8:
            SCALE_FACTOR = 0.5
        elif self.primary.getLeftTriggerAxis() >= 0.8:
            SCALE_FACTOR == 0.75
        # Scale dimensionless joystick inputs by the maximum speeds.
        vx = -meters_per_second.of(
            self.curve(self.primary.getLeftY()) * SCALE_FACTOR
            * self.MAX_LINEAR_VELOCITY.into(meters_per_second)
        ).into(meters_per_second)

        vy = -meters_per_second.of(
            self.curve(self.primary.getLeftX()) * SCALE_FACTOR
            * self.MAX_LINEAR_VELOCITY.into(meters_per_second)
        ).into(meters_per_second)

        omega = -radians_per_second.of(
            self.curve(self.primary.getRightX())
            * self.MAX_ANGULAR_VELOCITY.into(radians_per_second)
        ).into(radians_per_second)

        self.swerve.drive(
            vx,
            vy,
            omega,
            field_relative=True
        )
        if self.primary.getSquareButton():
            self.swerve.reset_heading()

        #</Swerve>
        
        # <Execute>
        #odometry moved to top
        self.intake.execute()
        try:
            self.shot_controller.execute()
        except RuntimeError:
            AlertManager.instant_alert("Shot physicaly impossible with given parameters", AlertType.ERROR)
        self.swerve.execute()
        # </Execute>

    def teleopInit(self):
        self.primary = LemonInput(0)
        self.secondary = LemonInput(1)
    def autonomous_init(self):
        # clear anything left on from a previous run
        self.intake.set_voltage(volts.of(0.0))
        self.intake.set_arm_voltage(volts.of(0.0))
        self.shot_controller.stop_shoot()
        self.shot_controller.indexer.conveyor_off()
        self.shot_controller.indexer.kicker_off()
        self.ctx = AutoContext(self.swerve, self.intake, self.shot_controller, self.odometry)
        try:
            selected = self.auto_chooser.get_selected()
            if selected is None:
                raise ValueError("no auto selected")
            self.routine1 = AutoRunner(selected)
            with open(selected, "r") as file:
                samples = json.loads(file.read())["trajectory"]["samples"]
            poses = [Pose2d(row["x"], row["y"], Rotation2d(row["heading"])) for row in samples]
            self.field.get_object("Planned").set_poses(poses[::2][:85])
            self.actual_poses = []
            self.field.get_object("Actual").set_poses([])
            self.swerve.reset_pose(poses[0])
        except Exception as e:
            self.routine1 = None
            AlertManager.instant_alert(f"auto load failed: {e}", AlertType.ERROR)
    def autonomous_periodic(self):
        if self.routine1 is not None:
            self.routine1.execute(self.ctx)
        self.ctx.execute()