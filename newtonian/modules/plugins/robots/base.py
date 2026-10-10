
from __future__ import annotations

from fnmatch import fnmatchcase

import newton
import numpy as np
import warp as wp

from newtonian.modules.params import Parameters
from newtonian.modules.paths import resolve
from newtonian.modules.registry import Registry
from newtonian.modules.plugins.robots.joints import apply_initial_joint_q
from newtonian.modules.plugins.robots.drives import apply_drives
from newtonian.modules.plugins.robots.match import shorthand
from newtonian.modules.transforms import convert_pose_to_transform


ROBOTS: Registry = Registry("robot")


class Robot:
    """
    """

    def __init__(self, name: str, p: Parameters) -> None:
        self.name = name
        self.p = p
        self._lazy_dofs = np.zeros(0, dtype=np.int32)
        self._lazy_gains = np.zeros((0, 2), dtype=np.float32)
        self._drives_active = True
        self._sim = None

    def load(self, builder: newton.ModelBuilder, path: str, xform, floating: bool, args: dict) -> None:
        """
        """
        raise NotImplementedError

    def on_build(self, sim, scene: newton.ModelBuilder) -> None:
        """
        """
        p = self.p

        builder = newton.ModelBuilder(up_axis=scene.up_axis, gravity=scene.gravity)
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

        body_armature = float(p.get("body_armature", 0.0))
        if body_armature > 0.0:
            for b in range(builder.body_count):
                inertia = np.asarray(builder.body_inertia[b], dtype=np.float32).reshape(3, 3)
                builder.body_inertia[b] = wp.mat33(inertia + np.eye(3, dtype=np.float32) * body_armature)

        lazy = apply_drives(builder=builder, drives=p.subtree("drives"))
        apply_initial_joint_q(builder=builder, values=p.subtree("initial_joint_q").as_kwargs())

        dof_offset = scene.joint_dof_count
        scene.add_builder(builder=builder, label_prefix=self.name)
        self._lazy_dofs = np.array([dof_offset + d for d in lazy], dtype=np.int32)
        self._lazy_gains = np.array(list(lazy.values()), dtype=np.float32).reshape(-1, 2)
        self._drives_active = not len(self._lazy_dofs)

    def on_model(self, sim) -> None:
        """
        """
        prefix = self.name + "/"

        def own(labels):
            return {i: shorthand(l) for i, l in enumerate(labels) if l.startswith(prefix)}

        model = sim.model
        self.joints = own(model.joint_label)
        self.bodies = own(model.body_label)
        self.shapes = own(model.shape_label)
        if not self.bodies:
            raise ValueError(f"robot '{self.name}' has no bodies in the model")
        self.root = min(self.bodies)
        self._sim = sim

    def engage(self, sim) -> None:
        """
        """
        if not self._drives_active:
            self._set_lazy_gains(sim, self._lazy_gains)
            self._drives_active = True

    def reset(self) -> None:
        """
        """
        if self._drives_active and len(self._lazy_dofs):
            self._set_lazy_gains(self._sim, np.zeros_like(self._lazy_gains))
            self._drives_active = False

    def _set_lazy_gains(self, sim, gains: np.ndarray) -> None:
        """
        """
        model = sim.model
        for col, arr in enumerate((model.joint_target_ke, model.joint_target_kd)):
            values = arr.numpy()
            values[self._lazy_dofs] = gains[:, col]
            arr.assign(values)
        sim.solver.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)

    def _find(self, table: dict[int, str], pattern, what: str, robot: str) -> list[int]:
        """
        """
        patterns = [pattern] if isinstance(pattern, str) else list(pattern)
        out = [i for i, n in table.items() if any(fnmatchcase(n, p) for p in patterns)]
        if not out:
            raise ValueError(f"robot '{robot}' has no {what} matching {pattern!r}")
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


class Robots(dict[str, Robot]):
    """
    Robots of a scene as a normal dict where the robot's name keys its instance.
    An additional function 'resolve' finds the robot that a plugin's 'robot' key refers to.
    """

    def resolve(self, name: str | None, caller: str) -> Robot:
        """
        Returns the robot object that a plugin's 'robot' key refers to.
        The 'caller' argument refers to the plugin asking to resolve.
        """
        if name:
            if name not in self:
                raise ValueError(f"{caller} : unknown robot '{name}' w.r.t. expected {sorted(self)}")
            return self[name]
        if not self:
            raise ValueError(f"{caller} : the scene has no robot")
        if len(self) > 1:
            raise ValueError(f"{caller} : set 'robot' to one of {sorted(self)}")
        return next(iter(self.values()))


def _configure(cfg, values: dict, what: str) -> None:
    """
    """
    for k, v in values.items():
        if not hasattr(cfg, k):
            raise KeyError(f"{what} has no field '{k}'")
        setattr(cfg, k, v)
