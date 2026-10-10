
from __future__ import annotations

import newton

from newtonian.modules.register.solvers.base import SOLVERS, SolverPlugin


@SOLVERS.register("mujoco")
class SolverMuJoCoPlugin(SolverPlugin):
    solver_cls = newton.solvers.SolverMuJoCo

    def builder_prepare(self, builder: newton.ModelBuilder) -> None:
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder=builder)

    @property
    def reports_contact_forces(self) -> bool:
        return True

    @property
    def uses_collision_pipeline(self) -> bool:
        return not self.args.get("use_mujoco_contacts", True)
