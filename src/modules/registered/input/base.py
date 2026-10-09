
from __future__ import annotations

import newton

from src.modules.params import Parameters
from src.modules.registry import Registry


INPUTS: Registry = Registry("input")


class Input:
    """
    """
    msg_type: type

    def __init__(self, name: str, p: Parameters) -> None:
        """
        """
        self.name = name
        self.p = p

        self.topic: str = p.get("topic", name)
        self.qos_depth = int(p.get("qos_depth", 10))
        self.robot_name: str | None = p.get("robot", None)
        self.timeout = float(p.get("timeout", 0.0))
        self._msg = None
        self._msg_time = None

    def on_build(self, sim, builder: newton.ModelBuilder) -> None:
        """
        """
        pass

    def on_model(self, sim) -> None:
        """
        """
        pass

    def on_timeout(self, sim) -> None:
        """
        """
        pass

    def callback(self, msg) -> None:
        """
        """
        self._msg = msg
        self._msg_time = None

    def apply(self, sim) -> None:
        """
        """
        msg = self._msg
        if msg is None:
            return
        if self._msg_time is None:
            self._msg_time = sim.sim_time
        if self.timeout > 0.0 and sim.sim_time - self._msg_time > self.timeout:
            self._msg = None
            self.on_timeout(sim)
            return
        self.handle(sim, msg)
        sim.robot(self.robot_name).engage(sim)

    def handle(self, sim, msg) -> None:
        """
        """
        raise NotImplementedError
