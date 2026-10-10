
from __future__ import annotations

import newton

from newtonian.modules.register.robots.base import ROBOTS, Robot


@ROBOTS.register("urdf")
class RobotURDF(Robot):

    def load(self, builder: newton.ModelBuilder, path: str, xform, floating: bool, args: dict) -> None:
        builder.add_urdf(source=path, xform=xform, floating=floating, **args)
