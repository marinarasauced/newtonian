
from __future__ import annotations

from fnmatch import fnmatchcase

import newton

from src.modules.params import Parameters
from src.modules.paths import resolve
from src.modules.registry import Registry
from src.modules.registered.robots.joints import apply_initial_joint_q
from src.modules.registered.robots.drives import apply_drives
from src.modules.registered.robots.utils import shorthand
from src.modules.utils import convert_pose_to_transform


ROBOTS: Registry = Registry("robot")


class Robot:
    """
    """

    def __init__(self, name: str, p: Parameters) -> None:
        self.name = name
        self.p = p

    def load(self, builder: newton.ModelBuilder, path: str, xform, floating: bool, args: dict) -> None:
        """
        """
        raise NotImplementedError

    def on_build(self, sim, scene: newton.ModelBuilder) -> None:
        """
        """
        p = self.p

        builder = newton.ModelBuilder()
        sim.solver_plugin.builder_prepare(builder)
        _configure(cfg=builder.default_joint_cfg, values=p.subtree("joint_cfg").as_kwargs(), what="joint_cfg")
        _configure(cfg=builder.default_shape_cfg, values=p.subtree("shape_cfg").as_kwargs(), what="shape_cfg")

        self.load(
            builder=builder,
            path=resolve(p.get(key="file")),
            xform=convert_pose_to_transform(p.get("pose", None)),
            floating=bool(p.get("floating", True)),
            args=p.subtree("loader_args").as_kwargs(),
        )

        apply_drives(builder=builder, drives=p.subtree("drives"))
        apply_initial_joint_q(builder=builder, values=p.subtree("initial_joint_q").as_kwargs())
        
        scene.add_builder(builder=builder, label_prefix=self.name)

    def on_model(self, sim) -> None:
        """
        """
        prefix = self.name + "/"

        def own(labels):
            return {i: shorthand(l) for i, l in enumerate(labels) if l.startswith(prefix)}

        model = sim.model
        self.joints = own(model.joint_label)
        self.bodies = own(model.body_label)
        self.shapes = own(model.body_label)
        if not self.bodies:
            raise ValueError(f"robot '{self.name}' has no bodies in the model")
        self.root = min(self.bodies)

    def reset(self) -> None:
        """
        """
        pass

    def _find(self, table: dict[int, str], pattern, what: str, robot: str) -> list[int]:
        """
        """
        patterns = [pattern] if isinstance(pattern, str) else list(pattern)
        out = [i for i, n in table.items() if any(fnmatchcase(n, p) for p in patterns)]
        if not out:
            raise ValueError(f"robot 'robot' has no {what} matching {pattern!r}")
        return out

    def find_joints(self, pattern) -> list[int]:
        """
        """
        return self._find(table=self.joints, pattern=pattern, what="joint", robot=self.name)

    def find_bodies(self, pattern) -> list[int]:
        """
        """
        return self._find(table=self.bodies, pattern=pattern, what="body", robot=self.name)

    def find_shapes(self, pattern) -> list[int]:
        """
        """
        return self._find(table=self.shapes, pattern=pattern, what="shape/site", robot=self.name)

    def body(self, name: str | None) -> int:
        """
        """
        return self.root if not name else self.find_bodies(name)[0]


def _configure(cfg, values: dict, what: str) -> None:
    """
    """
    for k, v in values.items():
        if not hasattr(cfg, k):
            raise KeyError(f"{what} has no field '{k}'")
        setattr(cfg, k, v)
