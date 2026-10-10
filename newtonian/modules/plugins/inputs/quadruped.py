
from __future__ import annotations

from typing import TYPE_CHECKING

from sensor_msgs.msg import JointState

from newtonian.modules.arrays import host
from newtonian.modules.plugins.inputs.base import INPUTS, Input

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@INPUTS.register("quadruped")
class InputQuadruped(Input):
    msg_type = JointState

    def on_model(self, sim: NewtonianSimulator) -> None:
        super().on_model(sim=sim)
        q_start = host(arr=sim.model.joint_q_start)
        qd_start = host(arr=sim.model.joint_qd_start)
        self.index = {name: (int(q_start[j]), int(qd_start[j])) for j, name in self.robot.joints.items()}

    def handle(self, sim: NewtonianSimulator, msg) -> None:
        for i, name in enumerate(msg.name):
            idx = self.index.get(name)
            if idx is None:
                continue
            q, qd = idx
            if i < len(msg.position):
                sim.buffers.joint_target_q[q] = msg.position[i]
            if i < len(msg.velocity):
                sim.buffers.joint_target_qd[qd] = msg.velocity[i]
            if i < len(msg.effort):
                sim.buffers.joint_f[qd] = msg.effort[i]
        sim.buffers.dirty = True
