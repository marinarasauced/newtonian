
from __future__ import annotations

import newton

from src.modules.registry import Registry


SOLVERS: Registry = Registry("solver")


class SolverPlugin:
    """
    Plugin for building and using one Newton solver type.
    """
    solver_cls: type
    reports_contact_forces = False
    supports_lazy_drives = True

    def __init__(self, args: dict) -> None:
        """
        """
        self.args = args

    def builder_prepare(self, builder: newton.ModelBuilder) -> None:
        """
        Called on the scene builder before any assets are loaded.
        """

    def builder_finalize(self, builder: newton.ModelBuilder) -> None:
        """
        Called on the scene builder before finalize.
        """

    def create(self, model: newton.Model) -> newton.solvers.SolverBase:
        return self.solver_cls(model, **self.args)

    @property
    def uses_collision_pipeline(self) -> bool:
        """
        True if contacts come from Newton's collision pipeline ;
        False if the solver must compute its own contacts.
        """
        return True

    def update_contacts(self, solver, contacts: newton.Contacts, state: newton.State) -> None:
        """
        Writes contact forces into the buffer after a frame.
        """
        solver.update_contacts(contacts, state)
