
from __future__ import annotations

import newton

from newtonian.modules.plugins.robots.base import ROBOTS, Robot


@ROBOTS.register("mjcf")
class RobotMJCF(Robot):

    def load(self, builder: newton.ModelBuilder, path: str, xform, floating: bool, args: dict) -> None:
        builder.add_mjcf(source=path, xform=xform, floating=floating, **args)
