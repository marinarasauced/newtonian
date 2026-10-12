
from __future__ import annotations

from typing import TYPE_CHECKING

import newton
import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import WrenchStamped

from newtonian.modules.arrays import host, need
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, Sensor

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class _Base(Sensor):
    """
    """
    msg_type = WrenchStamped

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        p = self.p

        bodies = self.robot.find_bodies(pattern=p.get(key="bodies"))
        self.sensor = newton.sensors.SensorContact(model=sim.model, sensing_bodies=bodies)
        self.frame_id = self.frame_id or "world"
        sim.require_contacts()

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        """
        if sim.contacts is None:
            raise ValueError(f"sensor cannot update when contacts are missing")
        self.sensor.update(state=sim.state, contacts=sim.contacts)
        self.data = host(arr=self.sensor.total_force).sum(axis=0)

    def _wrench(self, stamp: Time, data: np.ndarray) -> WrenchStamped:
        """
        """
        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.wrench.force.x = float(data[0])
        msg.wrench.force.y = float(data[1])
        msg.wrench.force.z = float(data[2])

        return msg


@SENSORS.register("contact_force")
class SensorContactForce(_Base):
    """
    """

    def to_msg(self, stamp: Time) -> WrenchStamped:
        """
        """
        return self._wrench(stamp=stamp, data=self.data)


@SENSORS.register("contact_scalar")
class SensorContactScalar(_Base):
    """
    """

    def to_msg(self, stamp: Time) -> WrenchStamped:
        """
        """
        msg = self._wrench(stamp=stamp, data=self.data)
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = float(np.linalg.norm(self.data))
        
        return msg
