
from __future__ import annotations

import newton

from src.modules.registered.environment.base import ENVIRONMENTS, Environment


@ENVIRONMENTS.register("default")
class EnvironmentDefault(Environment):

    def on_build(self, sim, builder: newton.ModelBuilder):
        pass
