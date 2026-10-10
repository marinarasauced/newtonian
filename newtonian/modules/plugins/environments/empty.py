
from __future__ import annotations

import newton

from newtonian.modules.plugins.environments.base import ENVIRONMENTS, Environment


@ENVIRONMENTS.register("empty")
class EnvironmentEmpty(Environment):

    def on_build(self, sim, builder: newton.ModelBuilder):
        pass
