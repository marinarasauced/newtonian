
#pyright: reportArgumentType=false
#pyright: reportReturnType=false
#pyright: reportIndexIssue=false
#pyright: reportOperatorIssue=false
#pyright: reportCallIssue=false
#type: ignore

from __future__ import annotations

import newton
import warp as wp

from src.modules.registry import Registry


SOLVERS: Registry = Registry("solver")


class SolverPlugin:
    """
    Plugin for building and using one Newton solver type.
    """
    solver_class: type
    reports_contact_forces = False

    def __init__(self, args: dict) -> None:
        """
        """
        self.args = args

    def builder_prepare(self, builder: newton.ModelBuilder) -> None:
        """
        Called on the scene builder before any assets are loaded.
        """

    def builder_finalize(self, builder: newton.ModelBuilder) -> None:
        """
        Called on the scene builder before 'finalize'.
        """

    def create(self, model: newton.Model) -> newton.solvers.SolverBase:
        return self.solver_class(model, **self.args)

    @property
    def uses_collision_pipeline(self) -> bool:
        """
        True if contacts come from Newton's collision pipeline ;
        False if the solver must compute its own contacts.
        """
        return True

    def update_contacts(self, solver, contacts: newton.Contacts, state: newton.State) -> None:
        """
        Writes contact forces into the buffer after a frame.
        """
        solver.update_contacts(contacts, state)


@wp.kernel
def _estimate_contact_forces(
    # inputs
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    shape_material_ke: wp.array[float],
    shape_material_kd: wp.array[float],
    shape_material_kf: wp.array[float],
    shape_material_ka: wp.array[float],
    shape_material_mu: wp.array[float],
    shape_body: wp.array[int],
    contact_count: wp.array[int],
    contact_point0: wp.array[wp.vec3],
    contact_point1: wp.array[wp.vec3],
    contact_normal: wp.array[wp.vec3],
    contact_shape0: wp.array[int],
    contact_shape1: wp.array[int],
    contact_margin0: wp.array[float],
    contact_margin1: wp.array[float],
    rigid_contact_stiffness: wp.array[float],
    rigid_contact_damping: wp.array[float],
    rigid_contact_friction_scale: wp.array[float],
    friction_smoothing: float,
    # outputs
    force: wp.array[wp.spatial_vector],
):
    """
    Writes updated contact forces to buffer rather than summing per body wrenches into
    """
    tid = wp.tid()
    force[tid] = wp.spatial_vector()

    count = contact_count[0]
    if tid >= count:
        return

    ke = 0.0
    kd = 0.0
    kf = 0.0
    ka = 0.0
    mu = 0.0
    mat_nonzero = 0
    margin_a = contact_margin0[tid]
    margin_b = contact_margin1[tid]
    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]
    if shape_a == shape_b:
        return
    body_a = -1
    body_b = -1
    if shape_a >= 0:
        mat_nonzero += 1
        ke += shape_material_ke[shape_a]
        kd += shape_material_kd[shape_a]
        kf += shape_material_kf[shape_a]
        ka += shape_material_ka[shape_a]
        mu += shape_material_mu[shape_a]
        body_a = shape_body[shape_a]
    if shape_b >= 0:
        mat_nonzero += 1
        ke += shape_material_ke[shape_b]
        kd += shape_material_kd[shape_b]
        kf += shape_material_kf[shape_b]
        ka += shape_material_ka[shape_b]
        mu += shape_material_mu[shape_b]
        body_b = shape_body[shape_b]
    if mat_nonzero > 0:
        ke /= float(mat_nonzero)
        kd /= float(mat_nonzero)
        kf /= float(mat_nonzero)
        ka /= float(mat_nonzero)
        mu /= float(mat_nonzero)

    if rigid_contact_stiffness:
        contact_ke = rigid_contact_stiffness[tid]
        ke = contact_ke if contact_ke > 0.0 else ke
        contact_kd = rigid_contact_damping[tid]
        kd = contact_kd if contact_kd > 0.0 else kd
        contact_mu = rigid_contact_friction_scale[tid]
        mu = mu * contact_mu if contact_mu > 0.0 else mu

    n = -contact_normal[tid]
    bx_a = contact_point0[tid]
    bx_b = contact_point1[tid]
    r_a = wp.vec3(0.0)
    r_b = wp.vec3(0.0)
    if body_a >= 0:
        X_wb_a = body_q[body_a]
        X_com_a = body_com[body_a]
        bx_a = wp.transform_point(X_wb_a, bx_a) - margin_a * n
        r_a = bx_a - wp.transform_point(X_wb_a, X_com_a)

    if body_b >= 0:
        X_wb_b = body_q[body_b]
        X_com_b = body_com[body_b]
        bx_b = wp.transform_point(X_wb_b, bx_b) + margin_b * n
        r_b = bx_b - wp.transform_point(X_wb_b, X_com_b)

    d = wp.dot(n, bx_a - bx_b)

    if d >= ka:
        return

    bv_a = wp.vec3(0.0)
    bv_b = wp.vec3(0.0)
    if body_a >= 0:
        qd_a = body_qd[body_a]
        bv_a = wp.spatial_top(qd_a) + wp.cross(wp.spatial_bottom(qd_a), r_a)

    if body_b >= 0:
        qd_b = body_qd[body_b]
        bv_b = wp.spatial_top(qd_b) + wp.cross(wp.spatial_bottom(qd_b), r_b)

    v = bv_a - bv_b

    vn = wp.dot(n, v)
    vt = v - n * vn

    fn = d * ke

    fd = wp.min(vn, 0.0) * kd * wp.step(d)

    ft = wp.vec3(0.0)
    if d < 0.0:
        vs = wp.norm_huber(vt, delta=friction_smoothing)
        if vs > 0.0:
            fr = vt / vs
            ft = fr * wp.min(kf * vs, -mu * (fn + fd))

    f_a = -(n * (fn + fd) + ft)
    force[tid] = wp.spatial_vector(f_a, wp.cross(r_a, f_a))


class SolverWarpedPlugin(SolverPlugin):
    """
    Plugin for Newton solver types that do not update contacts.

    This 'warped' extension of the SolverPlugin class computes contact forces at the end of each frame state using the same formula as Newton.
    These forces are an approximation of the forces applied in the solvers' substeps.
    """
    reports_contact_forces = True

    def update_contacts(self, solver, contacts: newton.Contacts, state: newton.State) -> None:
        if contacts.force is None or not contacts.rigid_contact_max:
            return
        model = solver.model
        wp.launch(
            kernel=_estimate_contact_forces,
            dim=contacts.rigid_contact_max,
            inputs=[
                state.body_q,
                state.body_qd,
                model.body_com,
                model.shape_material_ke,
                model.shape_material_kd,
                model.shape_material_kf,
                model.shape_material_ka,
                model.shape_material_mu,
                model.shape_body,
                contacts.rigid_contact_count,
                contacts.rigid_contact_point0,
                contacts.rigid_contact_point1,
                contacts.rigid_contact_normal,
                contacts.rigid_contact_shape0,
                contacts.rigid_contact_shape1,
                contacts.rigid_contact_margin0,
                contacts.rigid_contact_margin1,
                contacts.rigid_contact_stiffness,
                contacts.rigid_contact_damping,
                contacts.rigid_contact_friction,
                solver.friction_smoothing,
            ],
            outputs= [
                contacts.force,
            ],
            device=model.device,
        )


@SOLVERS.register("xpbd")
class SolverXBPDPlugin(SolverPlugin):
    solver_class = newton.solvers.SolverXPBD
    reports_contact_forces = True


@SOLVERS.register("mujoco")
class SolverMuJoCoPlugin(SolverPlugin):
    solver_class = newton.solvers.SolverMuJoCo
    reports_contact_forces = True

    def builder_prepare(self, builder: newton.ModelBuilder) -> None:
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder=builder)

    @property
    def uses_collision_pipeline(self) -> bool:
        return not self.args.get("use_mujoco_contacts", True)


@SOLVERS.register("featherstone")
class SolverFeatherstonePlugin(SolverWarpedPlugin):
    solver_class = newton.solvers.SolverFeatherstone


@SOLVERS.register("kamino")
class SolverKaminoPlugin(SolverPlugin):
    solver_class = newton.solvers.SolverKamino

    def builder_prepare(self, builder: newton.ModelBuilder) -> None:
        newton.solvers.SolverKamino.register_custom_attributes(builder=builder)

    def create(self, model: newton.Model) -> newton.solvers.SolverBase:
        defaults = self.solver_class.Config(use_collision_detector=True, use_fk_solver=True)
        args = {k: type(getattr(defaults, k))(**v) if isinstance(v, dict) else v for k, v in self.args.items()}
        return self.solver_class(model, config=self.solver_class.Config.from_model(model, **args))

    @property
    def uses_collision_pipeline(self) -> bool:
        return not self.args.get("use_collision_detector", False)

    @property
    def reports_contact_forces(self) -> bool:
        return self.uses_collision_pipeline
