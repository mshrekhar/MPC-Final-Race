"""
F1TENTH Frenet MPC Solver Generator — acados / SQP-RTI
"""

import os
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, tan, sqrt

N = 30
DT = 0.08
L = 0.33


def create_model():
    model = AcadosModel()
    model.name = "f1tenth_frenet"

    s = SX.sym("s")
    e_y = SX.sym("e_y")
    e_psi = SX.sym("e_psi")
    v = SX.sym("v")
    delta = SX.sym("delta")
    x = vertcat(s, e_y, e_psi, v, delta)

    delta_dot = SX.sym("delta_dot")
    a = SX.sym("a")
    u = vertcat(delta_dot, a)

    kappa = SX.sym("kappa")
    p = vertcat(kappa)

    # SMOOTH lower bound on (1 - kappa*e_y). C1-continuous → Gauss-Newton happy.
    denom_raw = 1.0 - kappa * e_y
    eps = 0.3
    denom = 0.5 * (denom_raw + eps + sqrt((denom_raw - eps) ** 2 + 0.01))

    s_dot = v * cos(e_psi) / denom
    e_y_dot = v * sin(e_psi)
    e_psi_dot = (v / L) * tan(delta) - kappa * s_dot
    v_dot = a
    delta_state_dot = delta_dot

    model.x = x
    model.u = u
    model.p = p
    model.f_expl_expr = vertcat(s_dot, e_y_dot, e_psi_dot, v_dot, delta_state_dot)
    return model


def create_ocp():
    ocp = AcadosOcp()
    model = create_model()
    ocp.model = model

    nx = 5
    nu = 2
    np_ = 1
    ny = 6
    ny_e = 4

    ocp.dims.N = N

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[0, 1] = 1.0
    ocp.cost.Vx[1, 2] = 1.0
    ocp.cost.Vx[2, 3] = 1.0
    ocp.cost.Vx[3, 4] = 1.0

    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[4, 0] = 1.0
    ocp.cost.Vu[5, 1] = 1.0

    ocp.cost.Vx_e = np.zeros((ny_e, nx))
    ocp.cost.Vx_e[0, 1] = 1.0
    ocp.cost.Vx_e[1, 2] = 1.0
    ocp.cost.Vx_e[2, 3] = 1.0
    ocp.cost.Vx_e[3, 4] = 1.0

    ocp.cost.W = np.diag([35.0, 15.0, 3.0, 12.0, 65.0, 0.20])
    ocp.cost.W_e = np.diag([45.0, 18.0, 3.0, 1.0])

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # Control bounds — hard
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([-3.2, -8.0])
    ocp.constraints.ubu = np.array([ 3.2,  8.0])

    # State bounds: e_y idx 1, e_psi idx 2 — these become SOFT (slacked).
    # v idx 3 and delta idx 4 stay HARD.
    # s idx 0 has no real bound.
    ocp.constraints.idxbx = np.array([1, 2, 3, 4])
    ocp.constraints.lbx   = np.array([-1.0, -0.7, 0.0, -0.6189])
    ocp.constraints.ubx   = np.array([ 1.0,  0.7, 8.0,  0.6189])

    # SOFT constraints on e_y (idx 1) and e_psi (idx 2).
    # idxsbx indexes INTO idxbx — so 0=e_y, 1=e_psi (positions in idxbx above).
    ocp.constraints.idxsbx = np.array([0, 1])
    ns = 2
    ocp.cost.zl = 1e3 * np.ones(ns)
    ocp.cost.zu = 1e3 * np.ones(ns)
    ocp.cost.Zl = 1e2 * np.ones(ns)
    ocp.cost.Zu = 1e2 * np.ones(ns)

    # Same for terminal stage
    ocp.constraints.idxbx_e = np.array([1, 2, 3, 4])
    ocp.constraints.lbx_e   = np.array([-1.0, -0.7, 0.0, -0.6189])
    ocp.constraints.ubx_e   = np.array([ 1.0,  0.7, 8.0,  0.6189])
    ocp.constraints.idxsbx_e = np.array([0, 1])
    ocp.cost.zl_e = 1e3 * np.ones(ns)
    ocp.cost.zu_e = 1e3 * np.ones(ns)
    ocp.cost.Zl_e = 1e2 * np.ones(ns)
    ocp.cost.Zu_e = 1e2 * np.ones(ns)

    ocp.constraints.x0 = np.zeros(nx)
    ocp.parameter_values = np.zeros(np_)

    ocp.solver_options.tf = N * DT
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.num_stages = 4
    ocp.solver_options.num_steps = 1
    ocp.solver_options.print_level = 0

    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.nlp_solver_step_length = 1.0
    ocp.solver_options.qp_solver_warm_start = 2

    # SANE LM. The 10.0 was strangling the optimizer.
    ocp.solver_options.levenberg_marquardt = 1e-3

    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ocp.code_export_directory = os.path.join(pkg_root, "c_generated_code_frenet")
    return ocp


if __name__ == "__main__":
    print("Generating F1TENTH Frenet MPC solver...")
    ocp = create_ocp()
    AcadosOcpSolver(ocp, json_file="f1tenth_frenet.json", generate=True, build=True)
    print("Done.")