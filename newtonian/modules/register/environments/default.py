
from __future__ import annotations

import newton

from newtonian.modules.register.environments.base import ENVIRONMENTS, Environment


@ENVIRONMENTS.register("default")
class EnvironmentDefault(Environment):

    def on_build(self, sim, builder: newton.ModelBuilder):
        builder.add_ground_plane(height=float(self.p.get("height", 0.0)), cfg=self.configure(builder=builder), color=self.color())
