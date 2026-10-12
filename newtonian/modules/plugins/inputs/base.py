
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import newton

from newtonian.modules.params import Parameters
from newtonian.modules.registry import Registry

from newtonian.modules.plugins.base import Plugin

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


INPUTS: Registry = Registry("input")



# --- Input Plugins --------------------------------------------------------------


class Input(Plugin):
    """
    """
    msg_type: type
    default_qos = "default"
    default_timeout = 0.0

    def __init__(self, name: str, p: Parameters) -> None:
        """
        """
        super().__init__(name=name, p=p)

        self.topic: str = p.get(key="topic", default=name)

        self.qos: str = p.get_choice(key="qos", choices=("default", "sensor_data"), default=self.default_qos)
        self.qos_depth = p.get_int(key="qos_depth", default=10)
        self.robot_name: str | None = p.get(key="robot", default=None)
        self.timeout = p.get_float(key="timeout", default=0.0, at_least=0.0)
        self._msg = None
        self._msg_time = None

    def callback(self, msg: Any) -> None:
        """
        """
        self._msg = msg
        self._msg_time = None

    def control(self, sim: NewtonianSimulator) -> None:
        """
        """
        msg = self._msg
        if msg is None:
            return
        if self._msg_time is None:
            self._msg_time = sim.sim_time
        if self.timeout > 0.0 and sim.sim_time - self._msg_time > self.timeout:
            self._msg = None
            self.on_timeout(sim=sim)
            return
        self.handle(sim, msg)
        self.robot.engage(sim=sim)

    def handle(self, sim: NewtonianSimulator, msg: Any) -> None:
        """
        """
        raise NotImplementedError

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        super().on_model(sim=sim)

        self.robot = sim.robots.resolve(name=self.robot_name, caller=f"input '{self.name}'")

    def on_timeout(self, sim: NewtonianSimulator) -> None:
        """
        """

    def reset(self) -> None:
        """
        """
        super().reset()
        
        self._msg = None
        self._msg_time = None
