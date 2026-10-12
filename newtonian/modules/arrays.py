
from __future__ import annotations

import numpy as np
import warp as wp


def to_numpy(arr: wp.array | None) -> np.ndarray:
    """
    """
    return require(arr).numpy()


def require(arr: wp.array | None) -> wp.array:
    """
    """
    if arr is None:
        raise RuntimeError("Newton array is not allocated.")
    return arr
