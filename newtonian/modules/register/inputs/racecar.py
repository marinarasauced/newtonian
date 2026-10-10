
from __future__ import annotations

from typing import TYPE_CHECKING

import math

from geometry_msgs.msg import Twist

from newtonian.common import host
from newtonian.modules.params import Parameters
from newtonian.modules.register.inputs.base import INPUTS, Input

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@INPUTS.register("racecar")
class InputRacecar(Input):
    msg_type = Twist

    def __init__(self, name: str, p: Parameters) -> None:
        super().__init__(name=name, p=p)

        self.L = float(p.get("wheelbase", 0.33))
        self.T = float(p.get("track", 0.24))
        self.r = float(p.get("wheel_radius", 0.05))
        self.max_steer = float(p.get("max_steer", 0.4189))
        self.max_speed = float(p.get("max_speed", 10.0))
        self.max_accel = float(p.get("max_accel", 3.0))
        self.max_decel = float(p.get("max_decel", 6.0))
        self.max_steer_rate = float(p.get("max_steer_rate", 3.0))
        self._v = 0.0
        self._delta = 0.0
        self._goal = (0.0, 0.0)

        if "timeout" not in p:
            self.timeout = 0.5

    def _command(self, msg) -> tuple[float, float]:
        v = msg.linear.x
        if abs(v) < 1.0e-3:
            return v, 0.0
        return v, math.atan(self.L * msg.angular.z / v)

    def on_model(self, sim: NewtonianSimulator) -> None:
        super().on_model(sim=sim)
        q_start = host(arr=sim.model.joint_q_start)
        qd_start = host(arr=sim.model.joint_qd_start)

        def joint(name):
            return self.robot.find_joints(name)[0]

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
        self._goal = (0.0, 0.0)

    def handle(self, sim, msg) -> None:
        v, delta = self._command(msg)
        v = max(-self.max_speed, min(self.max_speed, v))
        delta = max(-self.max_steer, min(self.max_steer, delta))
        self._goal = (v, delta)

    def _set(self, sim: NewtonianSimulator, v: float, delta: float) -> None:
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
            sim.buffers.joint_target_q[q] = angles[side]
        for role, qd in self.wheels.items():
            sim.buffers.joint_target_qd[qd] = speeds[role] / self.r
        sim.buffers.dirty = True

    def reset(self) -> None:
        super().reset()
        self._v = 0.0
        self._delta = 0.0
        self._goal = (0.0, 0.0)

    def apply(self, sim: NewtonianSimulator) -> None:
        super().apply(sim)
        v, delta = self._goal
        if v == self._v and delta == self._delta:
            return
        dt = sim.frame_dt
        rate = self.max_accel if abs(v) > abs(self._v) and v * self._v >= 0.0 else self.max_decel
        self._v += max(-rate * dt, min(rate * dt, v - self._v))
        self._delta += max(-self.max_steer_rate * dt, min(self.max_steer_rate * dt, delta - self._delta))
        self._set(sim, self._v, self._delta)
