
from __future__ import annotations

import newton

from newtonian.modules.plugins.solvers.base import SOLVERS, Solver


@SOLVERS.register("kamino")
class SolverKamino(Solver):
    """
    """
    solver_cls = newton.solvers.SolverKamino

    def before_build(self, builder: newton.ModelBuilder) -> None:
        """
        """
        newton.solvers.SolverKamino.register_custom_attributes(builder=builder)

    def create(self, model: newton.Model) -> newton.solvers.SolverBase:
        """
        """
        defaults = self.solver_cls.Config(use_collision_detector=True, use_fk_solver=True)
        args = {key: type(getattr(defaults, key))(**value) if isinstance(value, dict) else value for key, value in self.args.items()}
        return self.solver_cls(model, config=self.solver_cls.Config.from_model(model, **args))

    @property
    def reports_contact_forces(self) -> bool:
        """
        """
        return self.uses_collision_pipeline

    @property
    def supports_lazy_drives(self) -> bool:
        """
        """
        return False

    @property
    def uses_collision_pipeline(self) -> bool:
        """
        """
        return not self.args.get("use_collision_detector", False)
