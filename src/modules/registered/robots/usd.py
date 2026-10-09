
from __future__ import annotations

import newton

from src.modules.registered.robots.base import ROBOTS, Robot


@ROBOTS.register("usd")
class RobotUSD(Robot):

    def load(self, builder: newton.ModelBuilder, path: str, xform, floating: bool, args: dict) -> None:
        builder.add_usd(source=path, xform=xform, floating=floating, **args)
