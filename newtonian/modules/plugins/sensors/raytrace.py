
from __future__ import annotations

from typing import TYPE_CHECKING, cast

import newton
import numpy as np
import warp as wp

from newtonian.modules.transforms import TRANSFORM_ROS_TO_GL, tf_mul
from newtonian.modules.plugins.sensors.base import Cache, SensorFixed

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class SensorFixedRaytrace(SensorFixed):
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
