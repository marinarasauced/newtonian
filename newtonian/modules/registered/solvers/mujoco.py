
from __future__ import annotations

import newton

from src.modules.registered.solvers.base import SOLVERS, SolverPlugin


@SOLVERS.register("mujoco")
class SolverMuJoCoPlugin(SolverPlugin):
    solver_cls = newton.solvers.SolverMuJoCo
    reports_contact_forces = True

    def builder_prepare(self, builder: newton.ModelBuilder) -> None:
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder=builder)

    @property
    def uses_collision_pipeline(self) -> bool:
        return not self.args.get("use_mujoco_contacts", True)
