
from __future__ import annotations

import newton

from newtonian.modules.register.solvers.base import SOLVERS, SolverPlugin


@SOLVERS.register("xpbd")
class SolverXBPDPlugin(SolverPlugin):
    solver_cls = newton.solvers.SolverXPBD

    @property
    def reports_contact_forces(self) -> bool:
        return True
