
from __future__ import annotations

import numpy as np
import warp as wp


def host(arr: wp.array | None) -> np.ndarray:
    """
    """
    return need(arr).numpy()


def need(arr: wp.array | None) -> wp.array:
    """
    """
    if arr is None:
        raise RuntimeError("Newton array is not allocated.")
    return arr
