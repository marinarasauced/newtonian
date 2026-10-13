
from __future__ import annotations

from typing import TYPE_CHECKING

import newton
import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import WrenchStamped

from newtonian.modules.arrays import to_numpy
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, Sensor

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


class _Base(Sensor):
    """
    Net contact force on a group of robot bodies.
    """
    msg_type = WrenchStamped

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        Creates the contact sensor for 'bodies', and asks the simulator for contacts.
        """
        super().on_model(sim=sim)
        p = self.p

        bodies = self.robot.find_bodies(pattern=p.get(key="bodies"))
        self.sensor = newton.sensors.SensorContact(model=sim.model, sensing_bodies=bodies)
        self.frame_id = self.frame_id or "world"
        sim.require_contacts()

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        Sums the contact forces on the bodies.
        """
        if sim.contacts is None:
            raise ValueError("sensor cannot update when contacts are missing")
        self.sensor.update(state=sim.state, contacts=sim.contacts)
        self.data = to_numpy(arr=self.sensor.total_force).sum(axis=0)

    def _wrench(self, stamp: Time, data: np.ndarray) -> WrenchStamped:
        """
        Returns 'data' as a wrench message.
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
    Net contact force in the world frame.
    """

    def to_msg(self, stamp: Time) -> WrenchStamped:
        """
        Returns the force vector.
        """
        return self._wrench(stamp=stamp, data=self.data)


@SENSORS.register("contact_scalar")
class SensorContactScalar(_Base):
    """
    Magnitude of the net contact force.
    """

    def to_msg(self, stamp: Time) -> WrenchStamped:
        """
        Returns the force magnitude in 'force.z'.
        """
        msg = self._wrench(stamp=stamp, data=self.data)
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = float(np.linalg.norm(self.data))

        return msg
