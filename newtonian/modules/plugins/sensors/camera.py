
from __future__ import annotations

from typing import TYPE_CHECKING, cast

import numpy as np
import warp as wp

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

from newtonian.modules.plugins.sensors.base import SENSORS, Cache
from newtonian.modules.plugins.sensors.raytrace import SensorRaytrace

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class _Base(SensorRaytrace):
    """
    """
    msg_type = Image

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        super().on_model(sim=sim)
        p = self.p

        self.width = p.get_int(key="width", default=640, at_least=1)
        self.height = p.get_int(key="height", default=480, at_least=1)
        self.rays = self.tracer.utils.compute_camera_rays_pinhole(width=self.width, height=self.height, camera_fovs=p.get_float(key="fov", default=1.0, above=0.0))

    def _image(self, stamp: Time, data: np.ndarray, encoding: str, step: int) -> Image:
        """
        """
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.height, msg.width = self.height, self.width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = data.tobytes()

        return msg


@SENSORS.register("camera_color")
class SensorCameraColor(_Base):
    """
    """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        super().on_model(sim=sim)

        self.color = self.tracer.utils.create_color_image_output(width=self.width, height=self.height, camera_count=1)

    def to_msg(self, stamp: Time) -> Image:
        """
        """
        return self._image(stamp=stamp, data=self.data, encoding="rgb8", step=self.width*3)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        """
        cache.bvh_refit_shapes()
        self.tracer.update(state=sim.state, camera_transforms=self.transform(sim=sim, cache=cache), camera_rays=self.rays, color_image=self.color)
        rgba = cast(typ="wp.array", val=self.color).numpy().reshape(self.height, self.width).view(np.uint8).reshape(self.height, self.width, 4)
        self.data = np.ascontiguousarray(rgba[..., :3])


@SENSORS.register("camera_depth")
class SensorCameraDepth(_Base):
    """
    """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        super().on_model(sim=sim)

        self.depth = self.tracer.utils.create_depth_image_output(width=self.width, height=self.height, camera_count=1)

    def to_msg(self, stamp: Time) -> Image:
        """
        """
        return self._image(stamp=stamp, data=self.data, encoding="32FC1", step=self.width*4)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        """
        cache.bvh_refit_shapes()
        self.tracer.update(state=sim.state, camera_transforms=self.transform(sim=sim, cache=cache), camera_rays=self.rays, forward_depth_image=self.depth)
        depth = cast(typ="wp.array", val=self.depth).numpy().reshape(self.height, self.width).astype(np.float32)
        depth[depth <= 0.0] = np.nan
        self.data = depth
