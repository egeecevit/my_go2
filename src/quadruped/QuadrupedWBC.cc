/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#include "quadruped/QuadrupedWBC.hh"

#include <qpOASES.hpp>

namespace {

typedef QuadrupedDynamics::MatrixV MatrixV;
typedef QuadrupedDynamics::VectorV VectorV;

constexpr int NV = QuadrupedDynamics::NV;
constexpr int NUM_LEGS = QuadrupedDynamics::NUM_LEGS;

// QP (6.8) variables: the six floating-base acceleration corrections, then
// three force corrections for every CONTACT foot, in leg order. A swing foot
// gets no columns rather than columns pinned to zero (lb == ub): with pinned
// columns, setToMPC()'s cold start failed every 2-foot solve when
// w_base == w_force (status 36, RET_INIT_FAILED_HOTSTART) and reported false
// infeasibility for some draws at w_base >= 1e6. Without them, 0 failures in
// 400000 random draws on 1 to 4 feet with w_base / w_force from 1e-6 to 1e8,
// equal weights included.
constexpr int QP_MAX_VARS = 6 + 3 * NUM_LEGS;
// The six floating-base rows of (6.1), then four pyramid faces per contact foot.
constexpr int QP_MAX_CONS = 6 + 4 * NUM_LEGS;
// Working-set changes allowed per cold start. Over test_wbc's random draws,
// most with F_des outside the pyramid, a solve needed at most 20 (38 over
// 160000 draws on 1 to 4 feet with weight pairs up to 1e7); running out means
// the data is broken, not that the budget is tight.
constexpr int QP_MAX_WORKING_SET = 200;

// Row-major for qpOASES. Fixed maximum sizes keep the storage on the stack
// while the actual size follows the contact set.
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, QP_MAX_VARS,
                      QP_MAX_VARS>
    QpHessian;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, QP_MAX_CONS,
                      QP_MAX_VARS>
    QpConstraints;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, QP_MAX_VARS, 1> QpVars;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, QP_MAX_CONS, 1> QpCons;

// Eigenvalue cut-off of the inverse in insertLevel(), relative to the trace of
// the UNPROJECTED J W^-1 J^T. A direction taken by a higher priority leaves
// only round-off in the projected matrix, of order eps * cond(M) times that
// trace, i.e. below 1e-12 for cond(M) up to 1e4. A direction is cut only when
// the higher levels have left it 1e8 times weaker than the task's own scale.
// The price: a direction kept just above the cut-off has its error amplified
// by up to 1/sqrt(1e-8) = 1e4 in vdot_h, and vdot_h jumps back to the
// truncated solution when the cut-off is crossed (a 3 m/s^2 conflict gave
// |vdot_h| ~ 2.6e3 just above it, 2.6 below). A larger cut-off would bound that
// but stops meeting nearly dependent yet achievable tasks exactly: at 1e-7
// test_wbc's random square stacks already lose directions. Dropped directions
// are not reported; the task error shows only in J vdot_h + Jdv - acc_cmd.
// The reference is the unprojected matrix because a fully blocked task has a
// projected matrix that is round-off throughout, and a cut-off relative to
// itself would invert that noise.
constexpr double INVERSE_CUTOFF = 1e-8;

// One level of the recursion (6.5)-(6.6) in the metric W^-1:
//   Jp = J N,   Jbar = W^-1 Jp^T (Jp W^-1 Jp^T)^+,   N <- N (I - Jbar Jp).
// W^-1 = M^-1 is the dynamically consistent inverse (6.4) of the acceleration
// stage; W^-1 = I is the Moore-Penrose inverse that Kim et al. use in the
// kinematic stage.
//
// (Jp W^-1 Jp^T)^+ is a truncated eigen-decomposition pseudo-inverse. When a
// task conflicts with a higher priority, Jp loses rank and (6.4)'s plain inverse
// does not exist. Truncation is still an exact inverse on the kept subspace, so
// Jbar Jp stays a projector and N stays a null-space projector: a lower task
// cannot leak into a higher one, and an achievable task is met exactly. Damped
// least squares would be smooth near singularities but would break both
// guarantees in every direction, by lambda^2 / (sigma^2 + lambda^2).
void insertLevel(const QuadrupedDynamics::Jacobian& J, const MatrixV& Winv, MatrixV& N,
                 Eigen::Matrix<double, NV, 3>& Jbar) {
  const QuadrupedDynamics::Jacobian Jp = J * N;
  const Eigen::Matrix<double, NV, 3> WJpt = Winv * Jp.transpose();
  const double cutoff = INVERSE_CUTOFF * (J * Winv * J.transpose()).trace();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(Jp * WJpt);
  Eigen::Vector3d inv = eig.eigenvalues();
  for (int k = 0; k < 3; k++) inv[k] = inv[k] > cutoff ? 1.0 / inv[k] : 0.0;
  Jbar.noalias() = WJpt * eig.eigenvectors() * inv.asDiagonal() * eig.eigenvectors().transpose();
  // Equals (6.6)'s N (I - Jbar Jp) without its 18x18x18 product (a third of
  // solve()): N is idempotent with N W^-1 = W^-1 N^T, so N Jbar = Jbar.
  // Truncation keeps both properties, since inv is the exact pseudo-inverse
  // of the kept part.
  N.noalias() -= Jbar * Jp;
}

}  // namespace

QuadrupedWBC::QuadrupedWBC() {}

QuadrupedWBC::~QuadrupedWBC() {}

bool QuadrupedWBC::hierarchy(const input_t& in, QuadrupedDynamics::VectorV& vdot_h,
                             Eigen::Vector3d q_des[NUM_LEGS],
                             Eigen::Vector3d qd_des[NUM_LEGS]) {
  if (in.num_tasks < 0 || in.num_tasks > MAX_TASKS) return false;
  const QuadrupedDynamics::terms_t& dyn = *in.dyn;
  if (!dyn.M.allFinite()) return false;

  int nc = 0;
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    if (!in.q[leg].allFinite()) return false;
    if (!in.contact[leg]) continue;
    if (!dyn.contact[leg].J.allFinite() || !dyn.contact[leg].Jdv.allFinite()) return false;
    nc++;
  }
  // (6.5) needs a contact to seed from. With none the robot is in flight and no
  // torque can hold the body tasks.
  if (nc == 0) return false;
  for (int i = 0; i < in.num_tasks; i++) {
    const task_t& t = in.task[i];
    if (!t.J.allFinite() || !t.Jdv.allFinite() || !t.acc_cmd.allFinite() ||
        !t.pos_err.allFinite() || !t.vel_des.allFinite())
      return false;
  }

  const Eigen::LLT<MatrixV> llt(dyn.M);
  if (llt.info() != Eigen::Success) return false;
  const MatrixV Minv = llt.solve(MatrixV::Identity());

  // N, vdot_h: acceleration stage. Nk, dq, qd: kinematic stage.
  const MatrixV I = MatrixV::Identity();
  MatrixV N = I, Nk = I;
  VectorV dq = VectorV::Zero(), qd = VectorV::Zero();
  vdot_h.setZero();
  Eigen::Matrix<double, NV, 3> Jbar;

  // Seed (6.5), one contact foot per level. This equals the stacked form: every
  // increment stays in range(M^-1 J_c^T), where J_c vdot + Jdv_c = 0 has the
  // single solution -Jbar_c Jdv_c, and the final N is I - Jbar_c J_c. Per foot,
  // only 3x3 matrices are ever inverted.
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    if (!in.contact[leg]) continue;
    const QuadrupedDynamics::point_t& c = dyn.contact[leg];
    insertLevel(c.J, Minv, N, Jbar);
    const Eigen::Vector3d acc_err = -c.Jdv - c.J * vdot_h;
    vdot_h += Jbar * acc_err;
    // The contact feet are held still. dq = qd = 0 already does that, so only
    // the null space carries over.
    insertLevel(c.J, I, Nk, Jbar);
  }

  for (int i = 0; i < in.num_tasks; i++) {
    const task_t& t = in.task[i];

    // (6.6): only the part of the command the higher levels left unmet.
    insertLevel(t.J, Minv, N, Jbar);
    const Eigen::Vector3d acc_err = t.acc_cmd - t.Jdv - t.J * vdot_h;
    vdot_h += Jbar * acc_err;

    // The same recursion on position error and desired velocity.
    insertLevel(t.J, I, Nk, Jbar);
    const Eigen::Vector3d pos_err = t.pos_err - t.J * dq;
    const Eigen::Vector3d vel_err = t.vel_des - t.J * qd;
    dq += Jbar * pos_err;
    qd += Jbar * vel_err;
  }

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    q_des[leg] = in.q[leg] + dq.segment<3>(6 + 3 * leg);
    qd_des[leg] = qd.segment<3>(6 + 3 * leg);
  }
  return true;
}

bool QuadrupedWBC::solve(const input_t& in, output_t& out) {
  // Not a qpOASES success code, so a failure before the QP cannot leave the
  // previous call's status behind.
  out.qp_status = qpOASES::RET_QP_NOT_SOLVED;
  QuadrupedDynamics::VectorV vdot_h;
  if (!hierarchy(in, vdot_h, out.q_des, out.qd_des)) return false;
  const QuadrupedDynamics::terms_t& dyn = *in.dyn;
  if (!dyn.h.allFinite()) return false;
  // hierarchy() has checked that there is at least one.
  int legs[NUM_LEGS], nc = 0;
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    if (!in.contact[leg]) continue;
    if (!in.f_des[leg].allFinite()) return false;
    legs[nc++] = leg;
  }
  const int nv = 6 + 3 * nc, ncon = 6 + 4 * nc;

  const double INF = qpOASES::INFTY;
  const double mu = _params.friction;

  // Cost of (6.8). The gradient is zero because the cost is on the corrections
  // themselves: with no conflict the QP returns vdot_h and F_des untouched.
  QpHessian H = QpHessian::Zero(nv, nv);
  H.diagonal().head<6>().setConstant(_params.w_base);
  H.diagonal().tail(3 * nc).setConstant(_params.w_force);
  const QpVars g = QpVars::Zero(nv);

  QpVars lb(nv), ub(nv);
  QpConstraints A = QpConstraints::Zero(ncon, nv);
  QpCons lbA(ncon), ubA(ncon);
  lb.head<6>().setConstant(-INF);
  ub.head<6>().setConstant(INF);

  // Floating-base rows of (6.8) after substituting (6.7):
  //   M_vv dvdot_v - S_v J_c^T dF = -S_v (M vdot_h + h - J_c^T F_des).
  A.topLeftCorner<6, 6>() = dyn.M.topLeftCorner<6, 6>();
  Eigen::Matrix<double, 6, 1> rhs = -(dyn.M.topRows<6>() * vdot_h + dyn.h.head<6>());

  for (int k = 0; k < nc; k++) {
    const int col = 6 + 3 * k;
    const int row = 6 + 4 * k;
    const Eigen::Matrix<double, 6, 3> JtBase = dyn.contact[legs[k]].J.leftCols<6>().transpose();
    const Eigen::Vector3d& Fd = in.f_des[legs[k]];
    A.block<6, 3>(0, col) = -JtBase;
    rhs += JtBase * Fd;

    // Bounds and pyramid act on the TOTAL force F_des + dF, so an infeasible
    // F_des is pulled back inside rather than passed through.
    lb.segment<2>(col).setConstant(-INF);
    ub.segment<2>(col).setConstant(INF);
    lb[col + 2] = _params.fz_min - Fd.z();
    ub[col + 2] = _params.fz_max - Fd.z();
    for (int axis = 0; axis < 2; axis++) {
      const int r = row + 2 * axis;
      // f_axis - mu fz <= 0
      A(r, col + axis) = 1.0;
      A(r, col + 2) = -mu;
      lbA[r] = -INF;
      ubA[r] = -(Fd[axis] - mu * Fd.z());
      // f_axis + mu fz >= 0
      A(r + 1, col + axis) = 1.0;
      A(r + 1, col + 2) = mu;
      lbA[r + 1] = -(Fd[axis] + mu * Fd.z());
      ubA[r + 1] = INF;
    }
  }
  lbA.head<6>() = rhs;
  ubA.head<6>() = rhs;

  // A fresh problem sized to this contact set, cold-started: nothing carries
  // over between ticks. setToMPC(), as in MdlConvexMPC: the default set starts
  // every bound active and adds the equalities one by one, 3-4x slower here
  // for no accuracy gain (residuals ~1e-11 either way). qpOASES allocates on the
  // heap in its constructor and init(), about 120-200 mallocs per solve
  // depending on its iterations (measured; hierarchy() makes none). Accepted:
  // MdlConvexMPC also solves through qpOASES every tick.
  qpOASES::QProblem qp(nv, ncon);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);
  qpOASES::int_t nWSR = QP_MAX_WORKING_SET;
  const qpOASES::returnValue ret = qp.init(H.data(), g.data(), A.data(), lb.data(), ub.data(),
                                           lbA.data(), ubA.data(), nWSR, nullptr);
  out.qp_status = (int)ret;
  if (qpOASES::getSimpleStatus(ret) != 0) return false;
  QpVars x(nv);
  qp.getPrimalSolution(x.data());

  // (6.7), then (6.9) from the twelve actuated rows.
  out.vdot = vdot_h;
  out.vdot.head<6>() += x.head<6>();
  QuadrupedDynamics::VectorV gen = dyn.M * out.vdot + dyn.h;
  for (int leg = 0; leg < NUM_LEGS; leg++) out.f[leg].setZero();
  for (int k = 0; k < nc; k++) {
    const int leg = legs[k];
    out.f[leg] = in.f_des[leg] + x.segment<3>(6 + 3 * k);
    gen.noalias() -= dyn.contact[leg].J.transpose() * out.f[leg];
  }
  out.tau = gen.tail<3 * NUM_LEGS>();
  return true;
}
