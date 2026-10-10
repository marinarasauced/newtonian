
from __future__ import annotations

import newton

from newtonian.modules.params import Parameters
from newtonian.modules.registry import Registry


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
        self.qos: str = p.get("qos", "default")
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
        self.robot = sim.robots.resolve(name=self.robot_name)

    def on_timeout(self, sim) -> None:
        """
        """
        pass

    def reset(self) -> None:
        """
        """
        self._msg = None

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
        self.robot.engage(sim)

    def handle(self, sim, msg) -> None:
        """
        """
        raise NotImplementedError
