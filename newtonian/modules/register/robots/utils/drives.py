
from __future__ import annotations

import newton

from newtonian.modules.params import Parameters
from newtonian.modules.register.robots.utils.common import match


DRIVE_MODES = {
    "none": newton.JointTargetMode.NONE,
    "position": newton.JointTargetMode.POSITION,
    "velocity": newton.JointTargetMode.VELOCITY,
    "position_velocity": newton.JointTargetMode.POSITION_VELOCITY,
    "effort": newton.JointTargetMode.EFFORT,
}

_DRIVE_FIELDS = {
    "ke": "joint_target_ke",
    "kd": "joint_target_kd",
    "effort_limit": "joint_effort_limit",
    "velocity_limit": "joint_velocity_limit",
    "armature": "joint_armature",
    "damping": "joint_damping",
    "friction": "joint_friction",
}


def _dofs(builder: newton.ModelBuilder, joint: int) -> range:
    """
    """
    start = builder.joint_qd_start[joint]
    return range(start, start + sum(builder.joint_dof_dim[joint]))


def apply_drives(builder: newton.ModelBuilder, drives: Parameters) -> dict[int, tuple[float, float]]:
    """
    """
    lazy: set[int] = set()
    for drive in drives.children():
        d = drives.subtree(drive)
        joints = match(names=builder.joint_label, pattern=d.get("joints"))
        if not joints:
            raise ValueError(f"drive '{drive}' has no matching joints")
        mode = d.get("mode", None)
        is_lazy = d.get("lazy", None)
        for j in joints:
            for dof in _dofs(builder=builder, joint=j):
                if is_lazy is not None:
                    (lazy.add if is_lazy else lazy.discard)(dof)
                if mode is not None:
                    builder.joint_target_mode[dof] = int(DRIVE_MODES[mode])
                for key, field in _DRIVE_FIELDS.items():
                    if key in d:
                        getattr(builder, field)[dof] = float(d.get(key))
    gains = {}
    for dof in sorted(lazy):
        gains[dof] = (builder.joint_target_ke[dof], builder.joint_target_kd[dof])
        builder.joint_target_ke[dof] = 0.0
        builder.joint_target_kd[dof] = 0.0
    return gains
