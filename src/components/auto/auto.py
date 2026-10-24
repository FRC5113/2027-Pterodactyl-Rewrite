from components.swerve import SwerveDrive
from components.dispenser import Controller
from components.intake import Intake
from components.odometry import Odometry
import json
import math
import time
from lemonlib.util import AlertManager, AlertType
from smartunits import meters_per_second
from smartunits.angular_velocity import radians_per_second
from smartunits.voltage import volts
class AutoContext:
    sd: SwerveDrive
    it: Intake
    sc: Controller
    od: Odometry
    def __init__(self, sd: SwerveDrive, it: Intake, sc: Controller, od: Odometry):
        self.sd = sd
        self.it = it
        self.sc = sc
        self.od = od
    def execute(self):
        self.od.execute()
        self.it.execute()
        self.sc.execute()
        self.sd.execute()
def shoot_delay(ctx: AutoContext):
    start = time.time_ns()
    while time.time_ns() - start < 3.0e9:
        ctx.sc.force_shoot()
def intake_stuff(ctx: AutoContext):
    start = time.time_ns()
    while time.time_ns() - start < 1.0e9:
        ctx.it.set_arm_voltage(volts.of(6.0))
    ctx.it.set_arm_voltage(volts.of(12.0))
callbacks = {#the python type system wont allow type hints in lambdas, because it is bad.
    "IntakeOn": lambda ctx: intake_stuff(ctx),
    "IntakeOff": lambda ctx: ctx.it.set_voltage(volts.of(0.0)),
    "Shoot": lambda ctx: shoot_delay(ctx)
}

class AutoRunner():
    KP_TRANSLATION = 4.0  # (m/s) per meter of position error, TODO tune
    KP_HEADING = 4.0  # (rad/s) per radian of heading error, TODO tune
    MAX_CORRECTION = 1.5  # m/s, caps the position correction so a big error (e.g. at the start) can't swing the robot across the field
    MAX_STEP_S = 0.04  # the trajectory clock advances at most this much per loop, so a stalled loop can't make the plan run ahead of the robot
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.first_iteration = True
    def execute(self, ctx: AutoContext):
        if self.first_iteration:
            self.last_time_s = time.time_ns() / 1e9
            self.dt = 0.0
            with open(self.file_path, "r") as file:
                self.contents = json.loads(file.read())
            self.first_iteration = False
        now_s = time.time_ns() / 1e9
        self.dt += min(now_s - self.last_time_s, self.MAX_STEP_S)
        self.last_time_s = now_s
        dt = self.dt
        try:
            samples = self.contents["trajectory"]["samples"]
        except:
            AlertManager.instant_alert("Malformed Trajectory file, auto stopping", AlertType.WARNING)
            return
        if samples:
            ref = samples[-1] #past the end of the trajectory, hold the final pose
            for i, row in enumerate(samples): #if perf becomes an issue, binary search here would be an easy optimization
                if dt - row["t"] < 0:
                    distFromLastPos = abs(dt - samples[i - 1]["t"])
                    distFromFirstNeg = abs(dt - row["t"])
                    relativeDistPos = distFromLastPos / (distFromLastPos + distFromFirstNeg)
                    relativeDistNeg = distFromFirstNeg / (distFromLastPos + distFromFirstNeg)
                    ref = {k: relativeDistNeg * samples[i - 1][k] + relativeDistPos * row[k]
                           for k in ("x", "y", "heading", "vx", "vy", "omega")}
                    break
            # trajectory velocity + proportional correction toward the trajectory pose
            pose = ctx.sd.get_pose()
            fx = self.KP_TRANSLATION * (ref["x"] - pose.x)
            fy = self.KP_TRANSLATION * (ref["y"] - pose.y)
            correction = math.hypot(fx, fy)
            if correction > self.MAX_CORRECTION:
                fx *= self.MAX_CORRECTION / correction
                fy *= self.MAX_CORRECTION / correction
            vx = ref["vx"] + fx
            vy = ref["vy"] + fy
            omega = ref["omega"] + self.KP_HEADING * math.remainder(ref["heading"] - pose.rotation().radians(), math.tau)
            ctx.sd.drive(meters_per_second.of(vx), meters_per_second.of(vy), radians_per_second.of(omega), True)
        EVENT_TOLERANCE = 0.03 #30 ms
        for event in self.contents["events"]:
            if abs(event["from"]["targetTimestamp"] - dt) < EVENT_TOLERANCE:
                callbacks[event["name"]](ctx)