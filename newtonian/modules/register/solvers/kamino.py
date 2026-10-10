
from __future__ import annotations

import newton

from newtonian.modules.register.solvers.base import SOLVERS, SolverPlugin


@SOLVERS.register("kamino")
class SolverKaminoPlugin(SolverPlugin):
    solver_cls = newton.solvers.SolverKamino
    supports_lazy_drives =  False

    def builder_prepare(self, builder: newton.ModelBuilder) -> None:
        newton.solvers.SolverKamino.register_custom_attributes(builder=builder)

    def create(self, model: newton.Model) -> newton.solvers.SolverBase:
        defaults = self.solver_cls.Config(use_collision_detector=True, use_fk_solver=True)
        args = {key: type(getattr(defaults, key))(**value) if isinstance(value, dict) else value for key, value in self.args.items()}
        return self.solver_cls(model, config=self.solver_cls.Config.from_model(model, **args))

    @property
    def reports_contact_forces(self) -> bool:
        return self.uses_collision_pipeline

    @property
    def uses_collision_pipeline(self) -> bool:
        return not self.args.get("use_collision_detector", False)
