
from __future__ import annotations

import newton

from newtonian.modules.params import Parameters
from newtonian.modules.registry import Registry

ENVIRONMENTS: Registry = Registry("environment")


class Environment:
    """
    """
    default_color = (0.6, 0.6, 0.6)

    def __init__(self, name: str, p: Parameters) -> None:
        """
        """
        self.name = name
        self.p = p

    def on_build(self, sim, builder: newton.ModelBuilder) -> None:
        """
        """
        raise NotImplementedError

    def on_model(self, sim) -> None:
        """
        """
        pass

    def reset(self) -> None:
        """
        """
        pass

    def configure(self, builder: newton.ModelBuilder) -> newton.ModelBuilder.ShapeConfig:
        """
        """
        p = self.p
        cfg = builder.default_shape_cfg.copy()
        for key in ("mu", "restitution", "ke", "kd", "kf", "margin"):
            if key in p:
                setattr(cfg, key, float(p.get(key)))
        return cfg

    def color(self) -> tuple[float, float, float]:
        """
        """
        c = self.p.get(key="color", default=self.default_color)
        if len(c) != 3:
            raise ValueError(f"environment '{self.name}' color must be [r, g, b]")
        return float(c[0]), float(c[1]), float(c[2])
