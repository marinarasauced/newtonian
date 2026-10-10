
from __future__ import annotations

from pathlib import Path

import newton
import numpy as np
import warp as wp

import yaml

from src.modules.paths import resolve
from src.modules.registered.environments.base import ENVIRONMENTS, Environment


@ENVIRONMENTS.register("track")
class EnvironmentTrack(Environment):

    def on_build(self, sim, builder: newton.ModelBuilder):
        from PIL import Image

        map = Path(resolve(self.p.get("map")))
        meta = yaml.safe_load(map.read_text())
        image = np.asarray(Image.open(map.parent / meta["image"]).convert("L"), dtype=np.float32) / 255.0
        occ = image if meta.get("negate", 0) else 1.0 - image
        occupied = occ > float(meta.get("occupied_thresh", 0.65))

        res = float(meta["resolution"])
        ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
        height = float(self.p.get("wall_height", 0.3))
        cfg = self.configure(builder)
        c = self.color()
        rows = occupied.shape[0]

        for r0, c0, r1, c1 in _merge(occupied):
            x = ox + (c0 + c1) / 2.0 * res
            y = oy + (rows - (r0 + r1) / 2.0) * res
            builder.add_shape_box(
                body=-1,
                xform=wp.transform(wp.vec3(x, y, height / 2), wp.quat_identity()),
                hx=(c1 - c0) * res / 2,
                hy=(r1 - r0) * res / 2,
                hz=height / 2,
                cfg=cfg,
                color=c,
            )


def _merge(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """
    """
    mask = mask.copy()
    rectangles = []
    rows, cols = mask.shape
    for r in range(rows):
        c = 0
        while c < cols:
            if not mask[r, c]:
                c += 1
                continue
            c_end = c
            while c_end < cols and mask[r, c_end]:
                c_end += 1
            r_end = r + 1
            while r_end < rows and mask[r_end, c:c_end].all():
                r_end += 1
            mask[r:r_end, c:c_end] = False
            rectangles.append((r, c, r_end, c_end))
            c = c_end
    return rectangles
