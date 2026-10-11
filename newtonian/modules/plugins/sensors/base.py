
from __future__ import annotations

from typing import TYPE_CHECKING, Any, cast

import newton
import numpy as np
import warp as wp

from builtin_interfaces.msg import Time

from newtonian.modules.arrays import host
from newtonian.modules.params import Parameters
from newtonian.modules.registry import Registry
from newtonian.modules.transforms import TRANSFORM_ROS_TO_GL, convert_pose_to_transform, tf_mul

from newtonian.modules.plugins.base import Plugin

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator
    from newtonian.modules.plugins.robots import Robot


SENSORS: Registry = Registry("sensor")



# --- Sensor Plugins -------------------------------------------------------------


class Sensor(Plugin):
    """
    """
    msg_type: type
    default_qos = "sensor_data"

    def __init__(self, name: str, p: Parameters) -> None:
        """
        """
        super().__init__(name=name, p=p)

        self.topic: str = p.get(key="topic", default=name)
        self.rate = p.get_float(key="rate", default=0.0, at_least=0.0)

        self.qos: str = p.get(key="qos", default=self.default_qos)
        self.qos_depth = p.get_int(key="qos_depth", default=10, at_least=1)
        self.robot_name: str | None = p.get(key="robot", default=None)
        self.frame_id: str = p.get(key="frame_id", default="")
        self._due_time = 0.0

    def is_due(self, sim_time: float) -> bool:
        """
        True if the sensor must update.
        False otherwise.
        """
        if self.rate == 0.0:
            return True
        if sim_time + 1.0e-9 < self._due_time:
            return False
        period = 1.0 / self.rate
        self._due_time = max(self._due_time + period, sim_time + period * 0.5)
        return True

    def reset(self) -> None:
        """
        Resets the due update schedule.
        """
        self._due_time = 0.0

    def robot(self, sim: NewtonianSimulator) -> Robot:
        """
        Returns the robot that this sensor belongs to.
        """
        return sim.robots.resolve(name=self.robot_name, caller=f"sensor '{self.name}'")

    def to_msg(self, stamp: Time) -> Any:
        """
        """
        raise NotImplementedError

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        """
        raise NotImplementedError


class SensorFixed(Sensor):
    """
    """

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        """
        p = self.p

        # robot assignment
        robot = self.robot(sim=sim).name
        self.robot_name = robot

        # site assignment if configured
        if "site" in p:
            self.site_label = p.get(key="site")
            self.frame_id = self.frame_id or self.site_label
            return

        # site assignment if not configured
        body_name = p.get(key="body", default=None)
        prefix = robot + "/"
        bodies = [i for i, label in enumerate(builder.body_label) if label.startswith(prefix) and (body_name is None or label.rsplit("/", 1)[-1] == body_name)]
        if not bodies:
            raise ValueError(f"sensor '{self.name}' is missing body {body_name!r} in robot '{robot}'")
        self.site_label = f"{self.name}_site"
        builder.add_site(body=bodies[0], xform=convert_pose_to_transform(pose=p.get(key="offset", default=None)), label=f"{prefix}{self.site_label}")
        self.frame_id = self.frame_id or self.name

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        self.site = self.robot(sim=sim).find_shapes(pattern=self.site_label)[0]
        self.body = int(host(arr=sim.model.shape_body)[self.site])
        self.offset = host(arr=sim.model.shape_transform)[self.site].astype(np.float64)

    def pose(self, cache: Cache) -> np.ndarray:
        """
        """
        if self.body < 0:
            return self.offset
        return tf_mul(a=cache.body_q()[self.body].astype(np.float64), b=self.offset)


class SensorRaytrace(SensorFixed):
    """
    """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        super().on_model(sim=sim)
        p = self.p

        cfg = newton.sensors.SensorTiledCamera.RenderConfig()
        if "max_distance" in p or hasattr(self, "range_max"):
            cfg.max_distance = float(p.get(key="max_distance", default=getattr(self, "range_max", 1000.0)))

        self.tracer = newton.sensors.SensorTiledCamera(model=sim.model, default_render_config=cfg)
        sim.require_raytrace()

    def transform(self, sim: NewtonianSimulator, cache: Cache) -> wp.array2d:
        """
        """
        X = tf_mul(a=self.pose(cache=cache), b=TRANSFORM_ROS_TO_GL).astype(np.float32)
        out = wp.array(data=[[wp.transformf(wp.vec3f(*X[:3]), wp.quatf(*X[3:]))]], dtype=wp.transformf, device=sim.device)
        return cast(typ="wp.array2d[wp.transformf]", val=out)



# --- Cache ----------------------------------------------------------------------


class Cache:
    """
    Cached data that several sensors may read from for a given frame.
    A new cache is computed for each update call, ensuring that it is never stale.
    """

    def __init__(self, sim: NewtonianSimulator) -> None:
        """
        """
        self.sim = sim
        self._body_q: np.ndarray | None = None
        self._bvh_shapes_refit = False

    def body_q(self) -> np.ndarray:
        """
        """
        if self._body_q is None:
            self._body_q = host(arr=self.sim.state.body_q)
        return self._body_q

    def bvh_refit_shapes(self) -> None:
        """
        """
        if not self._bvh_shapes_refit:
            self.sim.model.bvh_refit_shapes(state=self.sim.state)
            self._bvh_shapes_refit = True



# --- Helper Functions -----------------------------------------------------------


def noise(sim: NewtonianSimulator, std: float, shape: int | tuple[int, ...]) -> np.ndarray:
    """
    """
    return sim.rng.normal(0.0, std, shape) if std > 0.0 else np.zeros(shape)


def update_sensors(sim: NewtonianSimulator, sensors: list[Sensor]) -> list[Sensor]:
    """
    """
    due = [s for s in sensors if s.is_due(sim_time=sim.sim_time)]
    cache = Cache(sim=sim)
    with wp.ScopedDevice(device=sim.device):
        for s in due:
            s.update(sim=sim, cache=cache)
    return due
