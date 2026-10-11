
from __future__ import annotations

from typing import TYPE_CHECKING

import newton

from newtonian.modules.params import Parameters

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class Plugin:
    """
    """

    def __init__(self, name: str, p: Parameters) -> None:
        """
        """
        self.name = name
        self.p = p

    def on_build(self, sim: NewtonianSimulator, builder: newton.ModelBuilder) -> None:
        """
        """

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """

    def reset(self) -> None:
        """
        """
