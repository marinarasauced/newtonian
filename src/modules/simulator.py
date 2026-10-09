
from __future__ import annotations

import logging

import newton
import numpy as np
import warp as wp

from src.modules.params import Parameters
from src.modules.registered.environments import ENVIRONMENTS, Environment
from src.modules.registered.input import INPUTS, Input
from src.modules.registered.robots import ROBOTS, Robot
# from src.modules.registered.sensors import *
from src.modules.registered.solvers import SOLVERS

log = logging.getLogger("newtonian")


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


class Targets:
    """
    """

    def __init__(self, control: newton.Control) -> None:
        """
        """
        self.q = host(arr=control.joint_target_q).copy()
        self.qd = host(arr=control.joint_target_qd).copy()
        self.f = host(arr=control.joint_f).copy()
        self.dirty = False

    def push(self, control: newton.Control) -> None:
        """
        """
        need(control.joint_target_q).assign(self.q)
        need(control.joint_target_qd).assign(self.qd)
        need(control.joint_f).assign(self.f)
        self.dirty = False


class Simulator:
    """
    """

    def __init__(self, p: Parameters) -> None:
        """
        """
        self.p = p
        s = p.subtree("sim")

        self.device = wp.get_device(s.get("device", None))
        self.frame_rate = float(s.get("frame_rate", 100.0))
        self.frame_dt = 1.0 / self.frame_rate
        self.substeps = int(s.get("substeps", 10))
        self.sim_dt = self.frame_dt / self.substeps
        self.gravity = float(s.get("gravity", -9.81))
        self.rng = np.random.default_rng(int(s.get("seed", 0)))
        self.use_graph = bool(s.get("cuda_graph", True))
        self.viewer_type = s.get("viewer", "none")

        self.sim_time = 0.0
        self.frame = 0
        self.viewer = None
        self.graph = None
        self.contacts = None
        self.collision_pipeline = None
        self._need_contacts = False
        self._need_raytrace = False
        self._body_q = None
        self._bvh_fresh = False

        with wp.ScopedDevice(self.device):
            self._build()

    def _build(self) -> None:
        """
        """
        p = self.p

        solver_type = p.get("solver.type", "mujoco")
        self.solver_plugin = SOLVERS.get(solver_type)(p.subtree(f"solver.{solver_type}").as_kwargs())

        scene = newton.ModelBuilder(gravity=(0.0, 0.0, self.gravity))
        self.solver_plugin.builder_prepare(scene)

        self.environments: list[Environment] = [ENVIRONMENTS.get(q.get("type"))(n, q) for n, q in p.entries("environment")]
        self.robots: dict[str, Robot] = {n: ROBOTS.get(q.get("type"))(n, q) for n, q in p.entries("robots")}
        self.robot_names = list(self.robots)
        # self.sensors: list[Sensor] = [SENSORS.get(q.get("type"))(n, q) for n, q in p.entries("sensors")]
        self.inputs: list[Input] = [INPUTS.get(q.get("type"))(n, q) for n, q in p.entries("inputs")]
        # plugins = (*self.environments, *self.robots.values(), *self.sensors, *self.inputs)
        plugins = (*self.environments, *self.robots.values(), *self.inputs)

        for item in plugins:
            item.on_build(self, scene)

        self.solver_plugin.builder_finalize(scene)
        self.model = scene.finalize()

        for item in plugins:
            item.on_model(self)

        if self._need_contacts and not self.solver_plugin.reports_contact_forces:
            raise ValueError(f"solver must report contact forces for contact sensors")
        if not self.solver_plugin.supports_lazy_drives and any(len(r._lazy_dofs) for r in self.robots.values()):
            raise ValueError(f"lazy drives are not supported by solver '{p.get('solver.type')}'")
        self.solver = self.solver_plugin.create(self.model)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        newton.eval_fk(self.model, need(self.model.joint_q), need(self.model.joint_qd), self.state_0)

        if self.solver_plugin.uses_collision_pipeline:
            self.collision_pipeline = newton.CollisionPipeline(self.model, **p.subtree("solver.collision_args").as_kwargs())
            self.contacts = self.collision_pipeline.contacts()
        elif self._need_contacts:
            self.contacts = newton.Contacts(
                self.solver.get_max_contact_count(),
                0,
                requested_attributes=self.model.get_requested_contact_attributes(),
            )

        self.targets = Targets(self.control)
        self._initial_state = self.model.state()
        self._initial_state.assign(self.state_0)
        self._initial_targets = (self.targets.q.copy(), self.targets.qd.copy(), self.targets.f.copy())

        if self.viewer_type == "gl":
            self.viewer = newton.viewer.ViewerGL()
            self.viewer.set_model(self.model)
        elif self.viewer_type != "none":
            raise ValueError(f"viewer must be 'none' or 'gl'")

        self._capture()

    def _capture(self) -> None:
        """
        """
        if not (self.use_graph and self.device.is_cuda):
            return
        if self.substeps % 2:
            log.warning("substeps is odd so CUDA graph is disabled")
        self._simulate()
        with wp.ScopedCapture() as capture:
            self._simulate()
        self.graph = capture.graph
        self._restore_initial_state()

    def _restore_initial_state(self) -> None:
        """
        """
        for state in (self.state_0, self.state_1):
            for name, arr in vars(state).items():
                if isinstance(arr, wp.array) and getattr(self._initial_state, name, None) is None:
                    setattr(self._initial_state, name, wp.zeros_like(arr))
            state.assign(self._initial_state)

    def _simulate(self) -> None:
        """
        """
        contacts = self.contacts
        for _ in range(self.substeps):
            self.state_0.clear_forces()
            if self.viewer is not None:
                self.viewer.apply_forces(self.state_0)
            if self.collision_pipeline is not None and contacts is not None:
                self.collision_pipeline.collide(self.state_0, contacts)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
        if self._need_contacts and contacts is not None:
            self.solver_plugin.update_contacts(self.solver, contacts, self.state_0)

    def step(self) -> None:
        """
        """
        for input in self.inputs:
            input.apply(self)
        if self.targets.dirty:
            self.targets.push(self.control)
        with wp.ScopedDevice(self.device):
            if self.graph is not None:
                wp.capture_launch(self.graph)
            else:
                self._simulate()
        self.sim_time += self.frame_dt
        self.frame += 1
        self._body_q = None
        self._bvh_fresh = False

    # def sense(self) -> list[Sensor]:
    #     """
    #     """
    #     due = [s for s in self.sensors if s.due(self.sim_time)]
    #     with wp.ScopedDevice(self.device):
    #         for s in due:
    #             s.update(self)
    #     return due

    def render(self) -> None:
        """
        """
        if self.viewer is None:
            return
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        if self.contacts is not None:
            self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.end_frame()

    def reset(self) -> None:
        """
        """
        self._restore_initial_state()
        q, qd, f = self._initial_targets
        self.targets.q[:], self.targets.qd[:], self.targets.f[:] = q, qd, f
        self.targets.push(self.control)
        # for item in (*self.environments, *self.robots.values(), *self.sensors, *self.inputs):
        for item in (*self.environments, *self.robots.values(), *self.inputs):
            item.reset()
        self.sim_time = 0.0
        self.frame = 0
        self._body_q = None
        self._bvh_fresh = False

    def close(self) -> None:
        """
        """
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def robot_name(self, name: str | None) -> str:
        """
        """
        if name:
            if name not in self.robot_names:
                raise ValueError(f"unknown robot '{name}'")
            return name
        if len(self.robot_names) != 1:
            raise ValueError(f"'robot' key is required when there are {len(self.robot_names)} robots")
        return self.robot_names[0]

    def robot(self, name: str | None) -> Robot:
        """
        """
        return self.robots[self.robot_name(name)]

    def body_q(self) -> np.ndarray:
        """
        """
        if self._body_q is None:
            self._body_q = host(self.state_0.body_q)
        return self._body_q

    @property
    def state(self) -> newton.State:
        """
        """
        return self.state_0

    def require_contacts(self) -> None:
        """
        """
        self._need_contacts = True

    def require_raytrace(self) -> None:
        """
        """
        self._need_raytrace = True

    def refit_bvh(self) -> None:
        """
        """
        if not self._bvh_fresh:
            self.model.bvh_refit_shapes(self.state_0)
            self._bvh_fresh = True
