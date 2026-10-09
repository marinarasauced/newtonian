
from __future__ import annotations

import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

from src.modules.params import Parameters
from src.modules.registered.input.base import INPUTS, Input


@INPUTS.register("racecar")
class InputRacecar(Input):

    def __init__(self, name: str, p: Parameters) -> None:
        super().__init__(name=name, p=p)
        kind = p.get("msg", "twist")
        if kind == "ackermann":
            self.msg_type = AckermannDriveStamped
        elif kind == "twist":
            self.msg_type = Twist
        else:
            raise ValueError(f"input '{name}' must be 'twist' or 'ackermann'")

        self.angular = p.get("angular", "yaw_rate")
        self.L = float(p.get("wheelbase", 0.33))
        self.T = float(p.get("track", 0.24))
        self.r = float(p.get("wheel_radius", 0.05))
        self.max_steer = float(p.get("max_steer", 0.4189))
        self.max_speed = float(p.get("max_speed", 10.0))

        if "timeout" not in p:
            self.timeout = 0.5

    def _command(self, msg) -> tuple[float, float]:
        if self.msg_type is not Twist:
            return msg.drive.speed, msg.drive.steering_angle
        v = msg.linear.x
        if self.angular == "steering_angle":
            return v, msg.angular.z
        if abs(v) < 1.0e-3:
            return v, 0.0
        return v, math.atan(self.L * msg.angular.z / v)

    def on_model(self, sim) -> None:
        info = sim.robot(self.robot_name)
        q_start = sim.model.joint_q_start.numpy()
        qd_start = sim.model.joint_qd_start.numpy()

        def joint(name):
            return info.find_joints(name)[0]

        steer = self.p.subtree("steer_joints").as_kwargs()
        wheels = self.p.subtree("wheel_joints").as_kwargs()
        self.steer = {side: int(q_start[joint(n)]) for side, n in steer.items()}
        self.wheels = {role: int(qd_start[joint(n)]) for role, n in wheels.items()}

        for side in self.steer:
            if side not in ("left", "right"):
                raise ValueError(f"input '{self.name}' must be 'left' / 'right'")
        for role in self.wheels:
            if role not in ("front_left", "front_right", "rear_left", "rear_right"):
                raise ValueError(f"input '{self.name}' wheel_joints keys must be in ('front_left', 'front_right', 'rear_left', 'rear_right')")

    def on_timeout(self, sim) -> None:
        self._set(sim, 0.0, 0.0)

    def handle(self, sim, msg) -> None:
        v, delta = self._command(msg)
        v = max(-self.max_speed, min(self.max_speed, v))
        delta = max(-self.max_steer, min(self.max_steer, delta))
        self._set(sim, v, delta)

    def _set(self, sim, v: float, delta: float) -> None:
        L, half = self.L, self.T / 2.0
        angles: dict[str, float]
        speeds: dict[str, float]
        if abs(delta) < 1.0e-4:
            angles = {"left": delta, "right": delta}
            speeds = {role: v for role in ("front_left", "front_right", "rear_left", "rear_right")}
        else:
            R = L / math.tan(delta)
            angles = {"left": math.atan(L / (R - half)), "right": math.atan(L / (R + half))}
            w = v / R
            speeds = {
                "front_left": math.copysign(1.0, R) * w * math.hypot(L, R - half),
                "front_right": math.copysign(1.0, R) * w * math.hypot(L, R + half),
                "rear_left": w * (R - half),
                "rear_right": w * (R + half),
            }
        for side, q in self.steer.items():
            sim.targets.q[q] = angles[side]
        for role, qd in self.wheels.items():
            sim.targets.qd[qd] = speeds[role] / self.r
        sim.targets.dirty = True
