
from __future__ import annotations

import newton

from src.modules.registered.solvers.base import SOLVERS, SolverPlugin


@SOLVERS.register("xpbd")
class SolverXBPDPlugin(SolverPlugin):
    solver_cls = newton.solvers.SolverXPBD
    reports_contact_forces = True
