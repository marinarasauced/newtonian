
from __future__ import annotations

import newton

from newtonian.modules.plugins.solvers.base import SOLVERS, Solver


@SOLVERS.register("mujoco")
class SolverMuJoCo(Solver):
    """
    """
    solver_cls = newton.solvers.SolverMuJoCo

    def before_build(self, builder: newton.ModelBuilder) -> None:
        """
        """
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder=builder)

    @property
    def reports_contact_forces(self) -> bool:
        """
        """
        return True

    @property
    def uses_collision_pipeline(self) -> bool:
        """
        """
        return not self.args.get("use_mujoco_contacts", True)
