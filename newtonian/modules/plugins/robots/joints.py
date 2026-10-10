
from __future__ import annotations

import newton

from newtonian.modules.plugins.robots.match import match


def apply_initial_joint_q(builder: newton.ModelBuilder, values: dict) -> None:
    """
    """
    for pattern, value in values.items():
        joints = match(names=builder.joint_label, pattern=pattern)
        if not joints:
            raise ValueError(f"no joint matches '{pattern}'")
        for j in joints:
            q = builder.joint_q_start[j]
            builder.joint_q[q] = float(value)
            builder.joint_target_q[q] = float(value)
