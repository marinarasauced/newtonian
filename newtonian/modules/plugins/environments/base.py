
from __future__ import annotations

from typing import TYPE_CHECKING

import newton

from newtonian.modules.params import Parameters, ParameterError
from newtonian.modules.registry import Registry

from newtonian.modules.plugins.base import Plugin

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


ENVIRONMENTS: Registry = Registry("environment")



# --- Environment Plugins -------------------------------------------------------------


class Environment(Plugin):
    """
    """
    default_color = (0.6, 0.6, 0.6)

    def __init__(self, name: str, p: Parameters) -> None:
        """
        """
        super().__init__(name=name, p=p)

        color = p.get(key="color", default=self.default_color)
        if len(color) != 3:
            raise ParameterError(f"{p.prefix} expected [r, g, b] but got '{color!r}'")
        self.color: tuple[float, float, float] = (float(color[0]), float(color[1]), float(color[2]))

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        """
        p = self.p

        cfg = builder.default_shape_cfg.copy()
        for key in ("mu", "restitution", "ke", "kd", "kf", "margin"):
            if key in p:
                setattr(cfg, key, float(p.get(key)))
        self.cfg = cfg
