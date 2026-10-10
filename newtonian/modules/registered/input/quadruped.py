
from __future__ import annotations

from sensor_msgs.msg import JointState

from src.modules.registered.input.base import INPUTS, Input


@INPUTS.register("quadruped")
class InputQuadruped(Input):
    msg_type = JointState

    def on_model(self, sim) -> None:
        info = sim.robot(self.robot_name)
        q_start = sim.model.joint_q_start.numpy()
        qd_start = sim.model.joint_qd_start.numpy()
        self.index = {name: (int(q_start[j]), int(qd_start[j])) for j, name in info.joints.items()}

    def handle(self, sim, msg) -> None:
        for i, name in enumerate(msg.name):
            idx = self.index.get(name)
            if idx is None:
                continue
            q, qd = idx
            if i < len(msg.position):
                sim.targets.q[q] = msg.position[i]
            if i < len(msg.velocity):
                sim.targets.qd[qd] = msg.velocity[i]
            if i < len(msg.effort):
                sim.targets.f[qd] = msg.effort[i]
        sim.targets.dirty = True
