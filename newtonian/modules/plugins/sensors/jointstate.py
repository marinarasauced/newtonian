
from __future__ import annotations

from typing import TYPE_CHECKING

from dataclasses import dataclass

import newton
import numpy as np

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState

from newtonian.modules.arrays import host
from newtonian.modules.plugins.sensors.base import SENSORS, Cache, Sensor

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


_JOINT_TYPES = (int(newton.JointType.REVOLUTE), int(newton.JointType.PRISMATIC))


@dataclass
class _Data:
    """
    """
    q: np.ndarray
    qd: np.ndarray


class _Base(Sensor):
    """
    """
    msg_type = JointState
    default_qos = "default"

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        p = self.p

        joint_type = host(arr=sim.model.joint_type)
        joint_q_start = host(arr=sim.model.joint_q_start)
        joint_qd_start = host(arr=sim.model.joint_qd_start)
        joints = [j for j in self.robot.find_joints(pattern=p.get(key="joints", default="*")) if joint_type[j] in _JOINT_TYPES]
        self.names = [self.robot.joints[j] for j in joints]
        self.joint_q_idx = joint_q_start[joints]
        self.joint_qd_idx = joint_qd_start[joints]

    def _jointstate(self, stamp: Time, data: _Data) -> JointState:
        """
        """
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.name = self.names
        msg.position = data.q.astype(float).tolist()
        msg.velocity = data.qd.astype(float).tolist()

        return msg


@SENSORS.register("jointstate")
class SensorJointState(_Base):
    """
    """

    def to_msg(self, stamp: Time) -> JointState:
        """
        """
        return self._jointstate(stamp=stamp, data=self.data)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        """
        self.data = _Data(
            q=host(arr=sim.state.joint_q)[self.joint_q_idx],
            qd=host(arr=sim.state.joint_qd)[self.joint_qd_idx],
        )
