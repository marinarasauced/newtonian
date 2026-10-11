
from __future__ import annotations

from typing import TYPE_CHECKING

import newton
import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import WrenchStamped

from newtonian.modules.arrays import host, need
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, SensorFree

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class _Base(SensorFree):
    """
    """
    msg_type = WrenchStamped

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        p = self.p

        robot = self.robot(sim=sim)
        bodies = robot.find_bodies(pattern=p.get(key="bodies"))
        self.sensor = newton.sensors.SensorContact(model=sim.model, sensing_bodies=bodies)
        self.frame_id = self.frame_id or "world"
        sim.require_contacts()

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        """
        self.sensor.update(state=sim.state, contacts=need(arr=sim.contacts))
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
class SensorFreeContactForce(_Base):
    """
    """

    def to_msg(self, stamp: Time) -> WrenchStamped:
        """
        """
        return self._wrench(stamp=stamp, data=self.data)


@SENSORS.register("contact_scalar")
class SensorFreeContactScalar(_Base):
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
