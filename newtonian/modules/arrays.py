
from __future__ import annotations

from typing import TypeVar

import numpy as np
import warp as wp


T = TypeVar("T")


def host(arr: wp.array | None) -> np.ndarray:
    """
    """
    return need(arr).numpy()


def need(arr: T | None) -> T:
    """
    """
    if arr is None:
        raise RuntimeError("Newton array is not allocated.")
    return arr
