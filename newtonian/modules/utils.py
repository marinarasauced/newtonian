
from __future__ import annotations

import numpy as np
import warp as wp


def convert_pose_to_transform(pose) -> wp.transform:
    """
    """
    pose = list(pose or [0.0] * 6) + [0.0] * 6
    x, y, z, roll, pitch, yaw = (float(value) for value in pose[:6])
    q = wp.quat_rpy(roll=roll, pitch=pitch, yaw=yaw)
    return wp.transform(wp.vec3(x, y, z), q)
