/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

// Invariants of the whole-body controller (thesis Sec. 6.2.1, Eq. 6.3-6.9).
//
// Every property checked here is algebraic: it holds for any symmetric
// positive definite M, any h and any full-row-rank Jacobians. So the inputs are
// random but well conditioned, drawn from a fixed seed, and the test does not
// depend on QuadrupedDynamics::compute(). A real-model check belongs with the
// MuJoCo tests.
//
// Appended later: MdlAMB's principal-frame helpers of the AMB swing task
// (thesis 6.13-6.26), and two checks on the real Go2 model that need no
// simulator (feedforward sense against the joint-space hip roll, static stand).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

#include "quadruped/QuadrupedWBC.hh"
// For the principal-frame helpers and the real-model checks at the end.
#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlAMB.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

// Linking MdlAMB.o drags in the hardware singletons (test_amb_template.cc);
// none is instantiated here. The macro names Hardware<T> unqualified.
namespace rtcore {
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);
}  // namespace rtcore

typedef QuadrupedDynamics::MatrixV MatrixV;
typedef QuadrupedDynamics::VectorV VectorV;
typedef QuadrupedDynamics::terms_t terms_t;
typedef QuadrupedWBC::input_t input_t;
typedef QuadrupedWBC::output_t output_t;
typedef QuadrupedWBC::task_t task_t;

static constexpr int NV = QuadrupedDynamics::NV;
static constexpr int NL = QuadrupedDynamics::NUM_LEGS;
static constexpr int TRIALS = 300;

// Tolerance of the hierarchy's equations, relative to the size of the solution:
// residual / (1 + |solution|). Round-off in J vdot grows with |vdot|, which is
// large when a draw's tasks are nearly dependent. Over these draws the stacked
// contact-and-task Jacobian reaches cond ~2.5e4 (cond(M) <= 30), and the
// relative error measured stays below 20 eps cond, i.e. ~1e-10 at worst. 1e-9
// leaves a decade; a real defect shows as O(1).
static constexpr double TOL_REL = 1e-9;
// qpOASES enforces bounds and constraints to its boundTolerance, ~1e-10 of the
// data. 1e-6 N on forces of O(100) N is a looser absolute ceiling, and the
// plan's limit for the floating-base residual.
static constexpr double TOL_QP = 1e-6;
// The KKT stationarity residual of the QP solution, relative to |H x|.
// qpOASES solves the KKT system of its final working set exactly up to
// round-off: 5e-14 measured over these draws, under 3e-11 over 2000 draws on
// all contact subsets with w_base / w_force up to 1e6. A pyramid face off by
// 10% leaves 0.3, w_base off by 0.1% leaves 2e-4.
static constexpr double TOL_KKT = 1e-9;

static const char* g_case = "(none)";

// NDEBUG-proof check: the build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                             \
  do {                                                                            \
    if (!(cond)) {                                                                \
      std::fprintf(stderr, "FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, \
                   #cond);                                                        \
      std::exit(1);                                                               \
    }                                                                             \
  } while (0)

// Prints the worst error of one property and fails when it exceeds tol.
static void report(const char* what, double worst, double tol) {
  std::printf("  %-62s worst %.2e (tol %.0e)\n", what, worst, tol);
  if (!(worst <= tol)) {
    std::fprintf(stderr, "FAIL [%s] %s: %.3e > %.0e\n", g_case, what, worst, tol);
    std::exit(1);
  }
}

static double maxAbs(const Eigen::VectorXd& v) { return v.size() ? v.cwiseAbs().maxCoeff() : 0.0; }

// A residual measured against the size of the solution it came from.
static double rel(const Eigen::VectorXd& residual, const Eigen::VectorXd& solution) {
  return maxAbs(residual) / (1.0 + maxAbs(solution));
}

// ---------------------------------------------------------------------------
//  Random fixtures
// ---------------------------------------------------------------------------

static std::mt19937 g_rng(20261125);

template <typename Derived>
static void randomize(Eigen::MatrixBase<Derived>& m, double lo, double hi) {
  std::uniform_real_distribution<double> u(lo, hi);
  for (int i = 0; i < m.rows(); i++)
    for (int j = 0; j < m.cols(); j++) m(i, j) = u(g_rng);
}

// M = B B^T + I with B uniform in [-1, 1]: symmetric positive definite with
// eigenvalues from 1 to about 25, the "well conditioned" the tolerances assume.
static void randomTerms(terms_t& dyn) {
  MatrixV B;
  randomize(B, -1.0, 1.0);
  dyn.M = B * B.transpose() + MatrixV::Identity();
  randomize(dyn.h, -10.0, 10.0);
  for (int leg = 0; leg < NL; leg++) {
    randomize(dyn.contact[leg].J, -1.0, 1.0);
    randomize(dyn.contact[leg].Jdv, -1.0, 1.0);
  }
}

static task_t randomTask() {
  task_t t;
  randomize(t.J, -1.0, 1.0);
  randomize(t.Jdv, -1.0, 1.0);
  randomize(t.acc_cmd, -5.0, 5.0);
  randomize(t.pos_err, -0.05, 0.05);
  randomize(t.vel_des, -0.5, 0.5);
  return t;
}

// FL-RR is the thesis' stance diagonal, FR-RL the other one; four feet is the
// end of the load transfer. Two feet leave 12 free directions, four feet 6, so
// 4 and 2 random tasks fill them exactly and are generically all achievable.
static const bool FEET_FL_RR[NL] = {true, false, false, true};
static const bool FEET_FR_RL[NL] = {false, true, true, false};
static const bool FEET_ALL[NL] = {true, true, true, true};
static const bool FEET_NONE[NL] = {false, false, false, false};

static int numContacts(const bool contact[NL]) {
  return (int)std::count(contact, contact + NL, true);
}

static void makeInput(const terms_t& dyn, const bool contact[NL], int num_tasks, input_t& in) {
  in = input_t();
  in.dyn = &dyn;
  for (int leg = 0; leg < NL; leg++) {
    randomize(in.q[leg], -1.0, 1.0);
    in.contact[leg] = contact[leg];
    in.f_des[leg] = Eigen::Vector3d(0.0, 0.0, 40.0);
  }
  in.num_tasks = num_tasks;
  for (int i = 0; i < num_tasks; i++) in.task[i] = randomTask();
}

static Eigen::Vector3d taskAcc(const task_t& t, const VectorV& vdot) { return t.J * vdot + t.Jdv; }

// Stacked J_c vdot + Jdv_c of the contact feet.
static Eigen::VectorXd contactAcc(const input_t& in, const VectorV& vdot) {
  Eigen::VectorXd a(3 * numContacts(in.contact));
  for (int leg = 0, k = 0; leg < NL; leg++)
    if (in.contact[leg]) a.segment<3>(3 * k++) = in.dyn->contact[leg].J * vdot + in.dyn->contact[leg].Jdv;
  return a;
}

static VectorV hierarchyOf(const input_t& in) {
  VectorV vdot;
  Eigen::Vector3d q_des[NL], qd_des[NL];
  T_CHECK(QuadrupedWBC::hierarchy(in, vdot, q_des, qd_des));
  T_CHECK(vdot.allFinite());
  return vdot;
}

// The solution of the stacked contact and task equations G vdot = b with the
// least vdot^T M vdot, computed without any recursion: with M = L L^T and
// w = L^T vdot it is the minimum-norm w of (G L^-T) w = b. For a compatible
// stack, the recursion with the dynamically consistent inverse (6.4) keeps
// every increment in M^-1 range(G^T) and must return exactly this; a
// Moore-Penrose recursion returns the solution of least |vdot| instead.
static VectorV weightedMinimumNorm(const input_t& in) {
  const int nc = numContacts(in.contact);
  Eigen::MatrixXd G(3 * nc + 3 * in.num_tasks, NV);
  Eigen::VectorXd b(G.rows());
  for (int leg = 0, k = 0; leg < NL; leg++)
    if (in.contact[leg]) {
      G.middleRows<3>(3 * k) = in.dyn->contact[leg].J;
      b.segment<3>(3 * k++) = -in.dyn->contact[leg].Jdv;
    }
  for (int i = 0; i < in.num_tasks; i++) {
    G.middleRows<3>(3 * nc + 3 * i) = in.task[i].J;
    b.segment<3>(3 * nc + 3 * i) = in.task[i].acc_cmd - in.task[i].Jdv;
  }
  const Eigen::LLT<MatrixV> llt(in.dyn->M);
  const Eigen::MatrixXd GLt = llt.matrixL().solve(G.transpose()).transpose();
  const Eigen::VectorXd w = GLt.completeOrthogonalDecomposition().solve(b);
  return llt.matrixU().solve(w);
}

// ---------------------------------------------------------------------------
//  Acceleration stage
// ---------------------------------------------------------------------------

static void testAchievable() {
  g_case = "achievable";
  std::printf("contact consistency and achievable tasks (%d draws)\n", TRIALS);
  double worst_contact = 0.0, worst_task = 0.0, worst_metric = 0.0;
  for (int trial = 0; trial < TRIALS; trial++) {
    terms_t dyn;
    randomTerms(dyn);
    const bool* feet = trial % 3 == 0 ? FEET_FL_RR : trial % 3 == 1 ? FEET_FR_RL : FEET_ALL;
    // From the task count that fills the free directions down to none. Below
    // the full count the stack is underdetermined: many vdot meet it, and only
    // the metric of (6.4) decides which one the hierarchy returns.
    const int max_tasks = feet == FEET_ALL ? 2 : 4;
    input_t in;
    makeInput(dyn, feet, max_tasks - (trial / 3) % (max_tasks + 1), in);
    const VectorV vdot = hierarchyOf(in);
    worst_contact = std::max(worst_contact, rel(contactAcc(in, vdot), vdot));
    for (int i = 0; i < in.num_tasks; i++)
      worst_task = std::max(worst_task, rel(taskAcc(in.task[i], vdot) - in.task[i].acc_cmd, vdot));
    // Only underdetermined stacks: a full one has a single solution, already
    // checked by the residuals above, and comparing two solves of it measures
    // their forward error, eps * cond(G L^-T), up to ~1e-9 over these draws.
    if (in.num_tasks < max_tasks)
      worst_metric = std::max(worst_metric, rel(vdot - weightedMinimumNorm(in), vdot));
  }
  report("J_c vdot_h + Jdv_c = 0", worst_contact, TOL_REL);
  report("J_i vdot_h + Jdv_i = acc_cmd_i (2 feet + 0..4 tasks, 4 feet + 0..2)", worst_task, TOL_REL);
  report("underdetermined: vdot_h = least vdot^T M vdot solution (6.4)", worst_metric, TOL_REL);
}

static void testPriority() {
  g_case = "priority";
  std::printf("strict priority (%d draws)\n", TRIALS);
  double worst_t1 = 0.0, worst_t2_free = 0.0, worst_changed = 0.0, worst_contact = 0.0,
         worst_blocked = 0.0, worst_conflict = 0.0;
  for (int trial = 0; trial < TRIALS; trial++) {
    terms_t dyn;
    randomTerms(dyn);
    input_t in;
    makeInput(dyn, FEET_FL_RR, 0, in);
    const task_t t1 = randomTask();
    // t2 shares its first row with t1 but asks 3 m/s^2 more along it: a direct
    // conflict there; its other two rows are free.
    task_t t2 = randomTask();
    t2.J.row(0) = t1.J.row(0);
    t2.Jdv[0] = t1.Jdv[0];
    t2.acc_cmd[0] = t1.acc_cmd[0] + 3.0;
    const task_t t3 = randomTask(), t3b = randomTask(), t4 = randomTask();

    // Baseline {t1, t2}; then a third task added, changed, and a fourth added.
    const task_t variants[4][4] = {{t1, t2}, {t1, t2, t3}, {t1, t2, t3b}, {t1, t2, t3, t4}};
    const int counts[4] = {2, 3, 3, 4};
    VectorV vdot[4];
    for (int v = 0; v < 4; v++) {
      in.num_tasks = counts[v];
      for (int i = 0; i < counts[v]; i++) in.task[i] = variants[v][i];
      vdot[v] = hierarchyOf(in);
      worst_contact = std::max(worst_contact, rel(contactAcc(in, vdot[v]), vdot[v]));
      worst_t1 = std::max(worst_t1, rel(taskAcc(t1, vdot[v]) - t1.acc_cmd, vdot[v]));
      const Eigen::Vector3d e2 = taskAcc(t2, vdot[v]) - t2.acc_cmd;
      worst_t2_free = std::max(worst_t2_free, rel(e2.tail<2>(), vdot[v]));
      worst_conflict = std::max(worst_conflict, std::fabs(std::fabs(e2[0]) - 3.0) / (1.0 + maxAbs(vdot[v])));
    }
    // What a higher level and the contacts see must not depend on what is
    // below them, including t2's unmet first row. Measured against the larger
    // of the two solutions compared.
    for (int v = 1; v < 4; v++) {
      const VectorV& big = maxAbs(vdot[v]) > maxAbs(vdot[0]) ? vdot[v] : vdot[0];
      worst_changed = std::max(worst_changed, rel(contactAcc(in, vdot[v]) - contactAcc(in, vdot[0]), big));
      worst_changed = std::max(worst_changed, rel(taskAcc(t1, vdot[v]) - taskAcc(t1, vdot[0]), big));
      worst_changed = std::max(worst_changed, rel(taskAcc(t2, vdot[v]) - taskAcc(t2, vdot[0]), big));
    }
    const VectorV& big = maxAbs(vdot[3]) > maxAbs(vdot[1]) ? vdot[3] : vdot[1];
    worst_changed = std::max(worst_changed, rel(taskAcc(t3, vdot[3]) - taskAcc(t3, vdot[1]), big));

    // A task fully blocked by a higher one: t1 again with another command. It
    // must change nothing, and the truncated inverse must not amplify the
    // round-off left in its projected Jacobian.
    task_t t1b = t1;
    t1b.acc_cmd += Eigen::Vector3d(1.0, -2.0, 3.0);
    in.num_tasks = 1;
    in.task[0] = t1;
    const VectorV alone = hierarchyOf(in);
    in.num_tasks = 2;
    in.task[1] = t1b;
    worst_blocked = std::max(worst_blocked, rel(hierarchyOf(in) - alone, alone));
  }
  report("higher task t1 met while t2 conflicts with it", worst_t1, TOL_REL);
  report("t2 met along its two rows free of t1", worst_t2_free, TOL_REL);
  report("contacts, t1, t2, t3 unchanged by adding/changing lower tasks", worst_changed, TOL_REL);
  report("contacts held in every variant", worst_contact, TOL_REL);
  report("fully blocked duplicate of t1 changes nothing", worst_blocked, TOL_REL);
  // The fixture really conflicts: t2's shared row stays short by exactly the
  // 3 m/s^2 the two tasks disagree.
  report("t2 shared row misses its command by exactly 3 m/s^2", worst_conflict, TOL_REL);
}

// ---------------------------------------------------------------------------
//  Kinematic stage
// ---------------------------------------------------------------------------

// hierarchy() returns only the joint part of dq = (q_des - q) and of qd_des.
// The base part is recovered as the least-squares solution of the stacked
// equations G [b; joint] = r over its six columns: if the joint part is
// right, an exact b exists and every residual vanishes; a wrong joint part
// leaves a residual, since G has more rows than the six base columns.
static void kinematicResiduals(const input_t& in, const VectorV& joint_part, bool velocity,
                               double& worst_contact, double& worst_task, double& worst_min_norm) {
  const int nc = numContacts(in.contact);
  Eigen::MatrixXd G(3 * nc + 3 * in.num_tasks, NV);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(G.rows());
  for (int leg = 0, k = 0; leg < NL; leg++)
    if (in.contact[leg]) G.middleRows<3>(3 * k++) = in.dyn->contact[leg].J;
  for (int i = 0; i < in.num_tasks; i++) {
    G.middleRows<3>(3 * nc + 3 * i) = in.task[i].J;
    r.segment<3>(3 * nc + 3 * i) = velocity ? in.task[i].vel_des : in.task[i].pos_err;
  }
  VectorV dq = joint_part;
  const Eigen::VectorXd rhs = r - G.rightCols(NV - 6) * joint_part.tail(NV - 6);
  dq.head<6>() = G.leftCols<6>().colPivHouseholderQr().solve(rhs);
  const Eigen::VectorXd res = G * dq - r;
  worst_contact = std::max(worst_contact, rel(res.head(3 * nc), dq));
  worst_task = std::max(worst_task, rel(res.tail(3 * in.num_tasks), dq));
  // An underdetermined stack has many solutions. The Moore-Penrose recursion
  // keeps every increment in the row space of G, so it must return the
  // minimum-norm one; any other metric (M^-1, say) would not. A full stack
  // has a single solution, already pinned by the residuals.
  if (G.rows() < NV) {
    const Eigen::VectorXd min_norm = G.completeOrthogonalDecomposition().solve(r);
    worst_min_norm = std::max(worst_min_norm, rel(min_norm.tail(NV - 6) - joint_part.tail(NV - 6), min_norm));
  }
}

static void testKinematic() {
  g_case = "kinematic";
  std::printf("kinematic stage (%d draws)\n", TRIALS);
  double worst[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int trial = 0; trial < TRIALS; trial++) {
    terms_t dyn;
    randomTerms(dyn);
    const bool* feet = trial % 3 == 0 ? FEET_FL_RR : trial % 3 == 1 ? FEET_FR_RL : FEET_ALL;
    // Every other draw leaves the joints underdetermined (fewer tasks than
    // free directions), so dq is the minimum-norm solution, not forced.
    const int num_tasks = (feet == FEET_ALL ? 2 : 4) - (trial % 2);
    input_t in;
    makeInput(dyn, feet, num_tasks, in);
    VectorV vdot;
    Eigen::Vector3d q_des[NL], qd_des[NL];
    T_CHECK(QuadrupedWBC::hierarchy(in, vdot, q_des, qd_des));
    VectorV dq = VectorV::Zero(), qd = VectorV::Zero();
    for (int leg = 0; leg < NL; leg++) {
      dq.segment<3>(6 + 3 * leg) = q_des[leg] - in.q[leg];
      qd.segment<3>(6 + 3 * leg) = qd_des[leg];
    }
    kinematicResiduals(in, dq, false, worst[0], worst[1], worst[4]);
    kinematicResiduals(in, qd, true, worst[2], worst[3], worst[5]);
  }
  report("J_c dq = 0", worst[0], TOL_REL);
  report("J_i dq = pos_err_i", worst[1], TOL_REL);
  report("J_c qd_des = 0", worst[2], TOL_REL);
  report("J_i qd_des = vel_des_i", worst[3], TOL_REL);
  report("underdetermined: dq = minimum-norm solution (Moore-Penrose)", worst[4], TOL_REL);
  report("underdetermined: qd_des = minimum-norm solution (Moore-Penrose)", worst[5], TOL_REL);
}

// ---------------------------------------------------------------------------
//  WBIC quadratic program
// ---------------------------------------------------------------------------

// Largest violation of the square pyramid and normal bounds by the contact
// forces, plus any force on a foot out of contact. Zero when all hold.
static double forceViolation(const QuadrupedWBC::params_t& p, const input_t& in,
                             const Eigen::Vector3d f[NL]) {
  double v = 0.0;
  for (int leg = 0; leg < NL; leg++) {
    if (!in.contact[leg]) {
      v = std::max(v, maxAbs(f[leg]));
      continue;
    }
    const double cone = p.friction * f[leg].z();
    v = std::max({v, std::fabs(f[leg].x()) - cone, std::fabs(f[leg].y()) - cone,
                  p.fz_min - f[leg].z(), f[leg].z() - p.fz_max});
  }
  return v;
}

// Lawson-Hanson non-negative least squares: argmin |A y - b| over y >= 0.
static Eigen::VectorXd nnls(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  const int n = A.cols();
  Eigen::VectorXd y = Eigen::VectorXd::Zero(n);
  std::vector<bool> passive(n, false);
  const double tol = 1e-14 * (1.0 + A.norm()) * (1.0 + b.norm());
  for (int outer = 0; outer < 3 * n; outer++) {
    // Free the clamped column along which the residual falls fastest.
    const Eigen::VectorXd grad = A.transpose() * (b - A * y);
    int j = -1;
    for (int i = 0; i < n; i++)
      if (!passive[i] && grad[i] > tol && (j < 0 || grad[i] > grad[j])) j = i;
    if (j < 0) break;
    passive[j] = true;
    for (;;) {
      // Least squares on the passive columns: the minimum-norm solution puts
      // zero on the columns blanked out.
      Eigen::MatrixXd Ap = A;
      for (int i = 0; i < n; i++)
        if (!passive[i]) Ap.col(i).setZero();
      const Eigen::VectorXd z = Ap.completeOrthogonalDecomposition().solve(b);
      // Step towards z until the first passive entry reaches zero, and clamp it.
      int k = -1;
      double alpha = 1.0;
      for (int i = 0; i < n; i++)
        if (passive[i] && z[i] <= 0.0 && y[i] / (y[i] - z[i]) < alpha) {
          alpha = y[i] / (y[i] - z[i]);
          k = i;
        }
      y += alpha * (z - y);
      if (k < 0) break;
      passive[k] = false;
      y[k] = 0.0;
    }
  }
  return y;
}

// Stationarity residual of the solution of (6.8), from its KKT conditions,
// derived here rather than read off solve(). With x the corrections
// [dvdot_v; dF of the contact feet], the problem is min 1/2 x^T H x subject to
// E x = r and g_i(F_des + dF) >= 0 for the pyramid faces and the fz bounds. A
// feasible x is optimal iff H x = E^T lambda + sum mu_i grad g_i over the
// active g_i, with mu_i >= 0. Projecting onto null(E) removes the free lambda;
// the best mu >= 0 then comes from non-negative least squares. Returns what
// is left, relative to |H x|. A wrong face, bound or weight leaves O(1), while
// the feasibility checks alone accept a QP whose constraints are too tight.
static double kktResidual(const QuadrupedWBC::params_t& p, const input_t& in, const VectorV& vdot_h,
                          const output_t& out, int& num_active) {
  const int nc = numContacts(in.contact), nx = 6 + 3 * nc;
  Eigen::VectorXd x(nx), Hx(nx);
  x.head<6>() = out.vdot.head<6>() - vdot_h.head<6>();
  Eigen::MatrixXd E(6, nx);
  E.leftCols<6>() = in.dyn->M.topLeftCorner<6, 6>();
  // Gradients of the active g_i, and of each g in terms of the total force F.
  Eigen::MatrixXd C(nx, 6 * nc);
  int na = 0;
  for (int leg = 0, k = 0; leg < NL; leg++) {
    if (!in.contact[leg]) continue;
    const int col = 6 + 3 * k++;
    const Eigen::Vector3d& F = out.f[leg];
    x.segment<3>(col) = F - in.f_des[leg];
    E.middleCols<3>(col) = -in.dyn->contact[leg].J.leftCols<6>().transpose();
    const double mu = p.friction;
    const double g[6] = {mu * F.z() - F.x(), mu * F.z() + F.x(), mu * F.z() - F.y(),
                         mu * F.z() + F.y(), F.z() - p.fz_min,   p.fz_max - F.z()};
    const Eigen::Vector3d grad[6] = {{-1.0, 0.0, mu}, {1.0, 0.0, mu}, {0.0, -1.0, mu},
                                     {0.0, 1.0, mu},  {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
    for (int i = 0; i < 6; i++) {
      // Within TOL_QP of the boundary counts as active. Counting an inactive
      // g only adds a column NNLS may leave at zero.
      if (g[i] > TOL_QP) continue;
      C.col(na).setZero();
      C.col(na++).segment<3>(col) = grad[i];
    }
  }
  num_active = na;
  Hx.head<6>() = p.w_base * x.head<6>();
  Hx.tail(3 * nc) = p.w_force * x.tail(3 * nc);
  const Eigen::MatrixXd P =
      Eigen::MatrixXd::Identity(nx, nx) - E.transpose() * (E * E.transpose()).ldlt().solve(E);
  const Eigen::MatrixXd PC = P * C.leftCols(na);
  const Eigen::VectorXd PHx = P * Hx;
  const Eigen::VectorXd mu = nnls(PC, PHx);
  return maxAbs(PC * mu - PHx) / (1.0 + maxAbs(Hx));
}

static void testQP() {
  g_case = "qp";
  std::printf("WBIC QP (%d draws)\n", TRIALS);
  QuadrupedWBC wbc;
  const QuadrupedWBC::params_t& p = wbc.getParams();
  double worst_base = 0.0, worst_all = 0.0, worst_force = 0.0, worst_joint_acc = 0.0,
         worst_kkt = 0.0;
  int infeasible_des = 0, solved[2] = {0, 0}, with_active = 0, total_active = 0;
  for (int trial = 0; trial < TRIALS; trial++) {
    terms_t dyn;
    randomTerms(dyn);
    const bool four = trial % 2;
    input_t in;
    makeInput(dyn, four ? FEET_ALL : FEET_FL_RR, four ? 2 : 4, in);
    // F_des over a box that is often outside the pyramid or the normal bounds,
    // so the QP must pull the total force back inside.
    for (int leg = 0; leg < NL; leg++) {
      randomize(in.f_des[leg], -60.0, 60.0);
      in.f_des[leg].z() = std::uniform_real_distribution<double>(-50.0, 300.0)(g_rng);
    }
    if (forceViolation(p, in, in.f_des) > 1.0) infeasible_des++;

    const VectorV vdot_h = hierarchyOf(in);
    output_t out;
    T_CHECK(wbc.solve(in, out));
    T_CHECK(out.qp_status == 0);
    solved[four]++;

    VectorV eom = dyn.M * out.vdot + dyn.h;
    for (int leg = 0; leg < NL; leg++)
      if (in.contact[leg]) eom -= dyn.contact[leg].J.transpose() * out.f[leg];
    worst_base = std::max(worst_base, maxAbs(eom.head<6>()));
    eom.tail<3 * NL>() -= out.tau;
    worst_all = std::max(worst_all, maxAbs(eom));
    worst_force = std::max(worst_force, forceViolation(p, in, out.f));
    // (6.7) corrects only the floating base.
    worst_joint_acc = std::max(worst_joint_acc, maxAbs(out.vdot.tail<3 * NL>() - vdot_h.tail<3 * NL>()));
    int num_active = 0;
    worst_kkt = std::max(worst_kkt, kktResidual(p, in, vdot_h, out, num_active));
    with_active += num_active > 0;
    total_active += num_active;
  }
  std::printf("  %d draws with 2 feet and %d with 4 solved; %d had F_des outside the bounds\n",
              solved[0], solved[1], infeasible_des);
  std::printf("  %d draws ended with an active inequality, %.1f per draw on average\n", with_active,
              (double)total_active / TRIALS);
  T_CHECK(infeasible_des > TRIALS / 2);
  T_CHECK(with_active > TRIALS / 2);
  report("floating-base rows of M vdot + h - J_c^T F [N, Nm]", worst_base, TOL_QP);
  report("all 18 rows of M vdot + h - S^T tau - J_c^T F [N, Nm]", worst_all, TOL_QP);
  report("pyramid, fz bounds, zero force off contact [N]", worst_force, TOL_QP);
  report("joint rows of vdot equal vdot_h", worst_joint_acc, 0.0);
  report("optimality: KKT stationarity, multipliers >= 0 (rel. to |H x|)", worst_kkt, TOL_KKT);

  // Consistent and feasible: F_des strictly inside the bounds and h chosen so
  // that vdot_h and F_des already satisfy the floating-base rows. The QP then
  // has nothing to correct.
  double worst_dv = 0.0, worst_df = 0.0;
  for (int trial = 0; trial < TRIALS; trial++) {
    terms_t dyn;
    randomTerms(dyn);
    const bool four = trial % 2;
    input_t in;
    makeInput(dyn, four ? FEET_ALL : FEET_FL_RR, four ? 2 : 4, in);
    for (int leg = 0; leg < NL; leg++) {
      const double fz = std::uniform_real_distribution<double>(20.0, 150.0)(g_rng);
      const double ft = 0.5 * p.friction * fz;
      in.f_des[leg] = Eigen::Vector3d(std::uniform_real_distribution<double>(-ft, ft)(g_rng),
                                      std::uniform_real_distribution<double>(-ft, ft)(g_rng), fz);
    }
    const VectorV vdot_h = hierarchyOf(in);
    VectorV base = -(dyn.M * vdot_h);
    for (int leg = 0; leg < NL; leg++)
      if (in.contact[leg]) base += dyn.contact[leg].J.transpose() * in.f_des[leg];
    dyn.h.head<6>() = base.head<6>();

    output_t out;
    T_CHECK(wbc.solve(in, out));
    T_CHECK(out.qp_status == 0);
    worst_dv = std::max(worst_dv, rel(out.vdot - vdot_h, vdot_h));
    for (int leg = 0; leg < NL; leg++)
      if (in.contact[leg]) worst_df = std::max(worst_df, rel(out.f[leg] - in.f_des[leg], in.f_des[leg]));
  }
  report("consistent input: vdot correction is zero", worst_dv, TOL_REL);
  report("consistent input: force correction is zero", worst_df, TOL_REL);

  // The objective of (6.8). F_des is well inside the bounds and h leaves the
  // floating-base rows off by a few N, Nm. The minimizer of the cost under the
  // equalities alone is then x = H^-1 E^T (E H^-1 E^T)^-1 b in closed form, and
  // when it also satisfies the inequalities it solves (6.8), being the optimum
  // of a relaxation. solve() must return it for every weight pair: the default,
  // the swapped one, and the identity Hessian, which once failed every 2-foot
  // solve.
  const double weights[3][2] = {{1000.0, 1.0}, {1.0, 1000.0}, {1.0, 1.0}};
  double worst_obj_v = 0.0, worst_obj_f = 0.0;
  int used = 0;
  for (int trial = 0; trial < TRIALS; trial++) {
    QuadrupedWBC::params_t pw = p;
    pw.w_base = weights[trial % 3][0];
    pw.w_force = weights[trial % 3][1];
    QuadrupedWBC wbc_w;
    wbc_w.setParams(pw);
    terms_t dyn;
    randomTerms(dyn);
    const bool four = (trial / 3) % 2;
    input_t in;
    makeInput(dyn, four ? FEET_ALL : FEET_FL_RR, four ? 2 : 4, in);
    for (int leg = 0; leg < NL; leg++) {
      const double fz = std::uniform_real_distribution<double>(60.0, 140.0)(g_rng);
      const double ft = 0.3 * p.friction * fz;
      in.f_des[leg] = Eigen::Vector3d(std::uniform_real_distribution<double>(-ft, ft)(g_rng),
                                      std::uniform_real_distribution<double>(-ft, ft)(g_rng), fz);
    }
    const VectorV vdot_h = hierarchyOf(in);
    VectorV base = -(dyn.M * vdot_h);
    for (int leg = 0; leg < NL; leg++)
      if (in.contact[leg]) base += dyn.contact[leg].J.transpose() * in.f_des[leg];
    Eigen::Matrix<double, 6, 1> offset;
    randomize(offset, -2.0, 2.0);
    dyn.h.head<6>() = base.head<6>() + offset;

    // x = [dvdot_v; dF of the contact feet in leg order], E x = -offset.
    const int nc = numContacts(in.contact);
    Eigen::MatrixXd E(6, 6 + 3 * nc);
    E.leftCols<6>() = dyn.M.topLeftCorner<6, 6>();
    for (int leg = 0, k = 0; leg < NL; leg++)
      if (in.contact[leg]) E.middleCols<3>(6 + 3 * k++) = -dyn.contact[leg].J.leftCols<6>().transpose();
    Eigen::VectorXd Hinv(6 + 3 * nc);
    Hinv.head<6>().setConstant(1.0 / pw.w_base);
    Hinv.tail(3 * nc).setConstant(1.0 / pw.w_force);
    const Eigen::MatrixXd HinvEt = Hinv.asDiagonal() * E.transpose();
    const Eigen::VectorXd x = HinvEt * (E * HinvEt).ldlt().solve(-offset);
    VectorV vdot_ref = vdot_h;
    vdot_ref.head<6>() += x.head<6>();
    Eigen::Vector3d f_ref[NL];
    for (int leg = 0, k = 0; leg < NL; leg++)
      f_ref[leg] = in.contact[leg] ? Eigen::Vector3d(in.f_des[leg] + x.segment<3>(6 + 3 * k++))
                                   : Eigen::Vector3d::Zero();
    // A draw whose equality-only minimizer leaves the bounds says nothing here.
    if (forceViolation(pw, in, f_ref) > 0.0) continue;
    used++;

    output_t out;
    T_CHECK(wbc_w.solve(in, out));
    T_CHECK(out.qp_status == 0);
    worst_obj_v = std::max(worst_obj_v, rel(out.vdot - vdot_ref, vdot_ref));
    for (int leg = 0; leg < NL; leg++)
      worst_obj_f = std::max(worst_obj_f, rel(out.f[leg] - f_ref[leg], f_ref[leg]));
  }
  std::printf("  objective: %d of %d draws had their equality-only minimizer inside the bounds\n",
              used, TRIALS);
  T_CHECK(used > 0.9 * TRIALS);
  report("objective: vdot = closed-form minimizer (3 weight pairs)", worst_obj_v, TOL_REL);
  report("objective: forces = closed-form minimizer (3 weight pairs)", worst_obj_f, TOL_REL);
}

// ---------------------------------------------------------------------------
//  Failure and timing
// ---------------------------------------------------------------------------

// solve() must reject the input; hierarchy() must too, unless the bad value
// is one only the QP reads (h, F_des).
static void expectFailure(QuadrupedWBC& wbc, const input_t& in, const char* what,
                          bool qp_only = false) {
  VectorV vdot;
  Eigen::Vector3d q_des[NL], qd_des[NL];
  output_t out;
  const bool h = QuadrupedWBC::hierarchy(in, vdot, q_des, qd_des);
  const bool s = wbc.solve(in, out);
  std::printf("  %-62s hierarchy %s, solve %s\n", what, h ? "true" : "false", s ? "true" : "false");
  T_CHECK(h == qp_only);
  T_CHECK(!s);
  // A failure must not read as the zero "success" status.
  T_CHECK(out.qp_status != 0);
}

static void testFailure() {
  g_case = "failure";
  std::printf("rejected input\n");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  QuadrupedWBC wbc;
  terms_t dyn;
  randomTerms(dyn);
  input_t in;
  makeInput(dyn, FEET_FL_RR, 3, in);
  output_t out;
  T_CHECK(wbc.solve(in, out));

  // Each case corrupts one copy of a good input.
  {
    terms_t d = dyn;
    d.M(3, 7) = nan;
    input_t bad = in;
    bad.dyn = &d;
    expectFailure(wbc, bad, "NaN in M");
  }
  {
    terms_t d = dyn;
    d.h[2] = nan;
    input_t bad = in;
    bad.dyn = &d;
    expectFailure(wbc, bad, "NaN in h", true);
  }
  {
    terms_t d = dyn;
    d.contact[3].J(1, 4) = nan;
    input_t bad = in;
    bad.dyn = &d;
    expectFailure(wbc, bad, "NaN in a contact Jacobian");
  }
  {
    terms_t d = dyn;
    d.contact[0].Jdv[2] = nan;
    input_t bad = in;
    bad.dyn = &d;
    expectFailure(wbc, bad, "NaN in a contact Jdv");
  }
  {
    input_t bad = in;
    bad.task[1].J(2, 9) = nan;
    expectFailure(wbc, bad, "NaN in a task Jacobian");
  }
  {
    input_t bad = in;
    bad.task[2].vel_des[0] = nan;
    expectFailure(wbc, bad, "NaN in a task velocity");
  }
  {
    input_t bad = in;
    bad.q[1][1] = nan;
    expectFailure(wbc, bad, "NaN in a joint angle");
  }
  {
    input_t bad = in;
    bad.f_des[3][0] = nan;
    expectFailure(wbc, bad, "NaN in a contact foot's F_des", true);
  }
  {
    input_t bad = in;
    for (int leg = 0; leg < NL; leg++) bad.contact[leg] = FEET_NONE[leg];
    expectFailure(wbc, bad, "no contact foot");
  }
  {
    terms_t d = dyn;
    d.M = -MatrixV::Identity();
    input_t bad = in;
    bad.dyn = &d;
    expectFailure(wbc, bad, "M not positive definite");
  }

  // Data the header says is not read (feet out of contact, unused task slots)
  // may hold anything.
  {
    terms_t d = dyn;
    d.contact[1].J(0, 0) = nan;
    input_t ok = in;
    ok.dyn = &d;
    ok.f_des[2][2] = nan;
    ok.task[3].acc_cmd[1] = nan;
    output_t o;
    T_CHECK(wbc.solve(ok, o));
    T_CHECK(o.tau.allFinite() && o.vdot.allFinite());
    std::printf("  %-62s accepted\n", "NaN only in unread data (swing foot, unused task)");
  }
}

static void testTiming() {
  g_case = "timing";
  std::printf("time per solve() (hierarchy + QP), Release build\n");
  QuadrupedWBC wbc;
  const int N = 5000;
  for (int four = 0; four < 2; four++) {
    terms_t dyn;
    randomTerms(dyn);
    input_t in;
    makeInput(dyn, four ? FEET_ALL : FEET_FL_RR, four ? 2 : 4, in);
    for (int leg = 0; leg < NL; leg++) in.f_des[leg] = Eigen::Vector3d(5.0, -3.0, 60.0);
    output_t out;
    double total = 0.0, worst = 0.0;
    for (int k = 0; k < N; k++) {
      const auto t0 = std::chrono::steady_clock::now();
      const bool ok = wbc.solve(in, out);
      const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      T_CHECK(ok);
      total += dt;
      worst = std::max(worst, dt);
    }
    std::printf("  %d feet, %d tasks: mean %.1f us, max %.1f us over %d solves\n", four ? 4 : 2,
                in.num_tasks, 1e6 * total / N, 1e6 * worst, N);
  }
}

// ---------------------------------------------------------------------------
//  AMB swing task in the principal frame (thesis 6.13-6.26, MdlAMB statics)
// ---------------------------------------------------------------------------

// Pure geometry, so random unit e_n and hip->foot vectors of swing-leg size.
static Eigen::Vector3d randomUnit() {
  Eigen::Vector3d v;
  do randomize(v, -1.0, 1.0);
  while (v.norm() < 0.1);
  return v.normalized();
}

static void testPrincipalFrame() {
  g_case = "principal frame";
  std::printf("principal-frame helpers (%d draws)\n", TRIALS);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double worst_orth = 0.0, worst_hand = 0.0, worst_xi = 0.0, worst_trip = 0.0, worst_repro = 0.0,
         worst_et = 0.0, worst_ramp = 0.0, worst_arc = 0.0, worst_step = 0.0;
  for (int trial = 0; trial < TRIALS; trial++) {
    const Eigen::Vector3d e_n = randomUnit();
    Eigen::Vector3d d_f;
    randomize(d_f, -0.3, 0.3);
    Eigen::Matrix3d R;
    double rho = 0.0;
    if (!MdlAMB::principalFrame(e_n, d_f, R, rho)) continue;  // in-plane part under 1 cm
    worst_orth = std::max(worst_orth, maxAbs(Eigen::Map<const Eigen::VectorXd>(
                                          Eigen::Matrix3d(R.transpose() * R - Eigen::Matrix3d::Identity()).data(), 9)));
    // Right-handed with e_n first: det +1 and the first column e_n itself.
    worst_hand = std::max({worst_hand, std::fabs(R.determinant() - 1.0), maxAbs(R.col(0) - e_n)});
    // (6.15) in the current frame is (xi_n, rho, 0), and maps back to d_f.
    const Eigen::Vector3d xi = R.transpose() * d_f;
    worst_xi = std::max(worst_xi, maxAbs(xi - Eigen::Vector3d(d_f.dot(e_n), rho, 0.0)));
    worst_trip = std::max(worst_trip, maxAbs(R * xi - d_f));

    // A radial axis stored under a slightly different support line, as the
    // stance feet's rolling moves it, so the reprojection is exercised.
    Eigen::Matrix3d R0;
    double rho0 = 0.0;
    Eigen::Vector3d d0;
    randomize(d0, -0.3, 0.3);
    const Eigen::Vector3d e_n0 = (e_n + 0.05 * randomUnit()).normalized();
    if (!MdlAMB::principalFrame(e_n0, d0, R0, rho0)) continue;
    const Eigen::Vector3d e_r0 = R0.col(1);
    // (6.23): the tangential reference equal to the current arc (6.16), with
    // the current normal offset and radius, reproduces the current foot
    // (6.21)-(6.24), and e_t^d is the current e_t.
    const double xi_t = MdlAMB::tangentialArc(e_n, e_r0, rho, d_f);
    Eigen::Vector3d d_des, e_t_des;
    MdlAMB::swingReference(e_n, e_r0, xi(0), rho, xi_t, d_des, e_t_des);
    worst_repro = std::max(worst_repro, maxAbs(d_des - d_f));
    worst_et = std::max(worst_et, maxAbs(e_t_des - R.col(2)));
    // During the lift rho_d differs from rho: (6.20) inverts (6.16) at the
    // same rho_d, so the reference keeps the current direction at radius rho_d.
    const double rho_d = rho * (0.5 + 0.5 * (u(g_rng) + 1.0));
    MdlAMB::swingReference(e_n, e_r0, xi(0), rho_d,
                           MdlAMB::tangentialArc(e_n, e_r0, rho_d, d_f), d_des, e_t_des);
    worst_ramp = std::max(worst_ramp, maxAbs(d_des - xi(0) * e_n - rho_d * R.col(1)));
    // A tangential offset delta turns the reference about e_n by delta / rho_d,
    // an arc of exactly delta, starting along e_t^d. Within +-1 rad, so the
    // measured angle does not wrap on the small radii some draws have.
    const double delta = rho_d * u(g_rng);
    Eigen::Vector3d d1, d2, e_t1, e_t2;
    MdlAMB::swingReference(e_n, e_r0, xi(0), rho_d, xi_t, d1, e_t1);
    MdlAMB::swingReference(e_n, e_r0, xi(0), rho_d, xi_t + delta, d2, e_t2);
    const Eigen::Vector3d p1 = d1 - xi(0) * e_n, p2 = d2 - xi(0) * e_n;
    const double angle = std::atan2(e_n.dot(p1.cross(p2)), p1.dot(p2));
    worst_arc = std::max(worst_arc, std::fabs(rho_d * angle - delta));
    const double h = 1e-6;
    MdlAMB::swingReference(e_n, e_r0, xi(0), rho_d, xi_t + h, d2, e_t2);
    worst_step = std::max(worst_step, maxAbs((d2 - d1) / h - e_t1));
  }
  report("R_WP orthonormal", worst_orth, 1e-12);
  report("R_WP right-handed, first column e_n", worst_hand, 1e-12);
  report("(6.15) in the current frame = (xi_n, rho, 0)", worst_xi, 1e-12);
  report("(6.15) round trip R_WP xi = d_f [m]", worst_trip, 1e-12);
  report("(6.16)+(6.21)-(6.24) at xi_t^d = xi_t^c reproduce the foot [m]", worst_repro, 1e-12);
  report("(6.22) at xi_t^d = xi_t^c is the current e_t", worst_et, 1e-12);
  report("rho_d != rho: current direction at radius rho_d [m]", worst_ramp, 1e-12);
  report("tangential offset delta = arc of delta about e_n [m]", worst_arc, 1e-12);
  // Forward difference: the chord's error is h / (2 rho_d) ~ 1e-5 at worst.
  report("d(reference)/d(xi_t) = e_t^d", worst_step, 1e-4);

  // Degenerate radius (Sec. 6.3.2): rejected, outputs untouched.
  const Eigen::Vector3d e_n = Eigen::Vector3d(-1.0, 0.0, 0.0);
  const Eigen::Matrix3d sentinel = 7.0 * Eigen::Matrix3d::Ones();
  for (const double in_plane : {0.0, 1e-9, 0.005, 0.0099}) {
    Eigen::Matrix3d R = sentinel;
    double rho = 7.0;
    const Eigen::Vector3d d_f = 0.2 * e_n + in_plane * Eigen::Vector3d(0.0, 0.6, -0.8);
    T_CHECK(!MdlAMB::principalFrame(e_n, d_f, R, rho));
    T_CHECK(R == sentinel && rho == 7.0);
  }
  Eigen::Matrix3d R;
  double rho;
  T_CHECK(MdlAMB::principalFrame(e_n, 0.2 * e_n + Eigen::Vector3d(0.0, 0.012, 0.0), R, rho));
  std::printf("  %-62s rejected below 1 cm, accepted at 1.2 cm\n", "projected radius -> 0");

  // Feedforward (6.18): never above the limit, always along sign(thdd) e_t,
  // untouched when within the limit.
  double worst_norm = 0.0, worst_dir = 0.0, worst_exact = 0.0;
  int limited = 0;
  for (int trial = 0; trial < TRIALS; trial++) {
    const Eigen::Vector3d e_t = randomUnit();
    const double rho_f = 0.3 * (u(g_rng) + 1.0), thdd = 400.0 * u(g_rng), a_max = 50.0;
    Eigen::Vector3d a_f;
    const bool lim = MdlAMB::tangentialFeedforward(rho_f, thdd, e_t, a_max, a_f);
    T_CHECK(lim == (rho_f * std::fabs(thdd) > a_max));
    limited += lim;
    worst_norm = std::max(worst_norm, a_f.norm() - a_max);
    worst_dir = std::max(worst_dir, maxAbs(a_f - (thdd >= 0.0 ? 1.0 : -1.0) * a_f.norm() * e_t));
    if (!lim) worst_exact = std::max(worst_exact, maxAbs(a_f - rho_f * thdd * e_t));
  }
  std::printf("  feedforward: %d of %d draws norm-limited\n", limited, TRIALS);
  T_CHECK(limited > TRIALS / 4 && limited < 3 * TRIALS / 4);
  report("|a_f| - 50 m/s^2 (never positive)", std::max(worst_norm, 0.0), 1e-12);
  report("a_f along sign(thdd) e_t", worst_dir, 1e-12);
  report("a_f = rho thdd e_t when within the limit", worst_exact, 1e-12);
}

// The nominal diagonal posture of test_amb_sim.cc (stance feet on the
// centreline, swing feet at the header-default lift targets), body frame.
static const Eigen::Vector3d FOOT_DIAG[NL] = {{0.15, 0.0, -0.278},
                                              {0.2634, -0.142, -0.18},
                                              {-0.2634, 0.142, -0.18},
                                              {-0.15, 0.0, -0.278}};

// The feedforward has the sense of the joint-space realization: a positive
// template acceleration thdd_j rolls swing leg j so that theta_j grows (roll
// acceleration thdd_j / j_c), and a_f for thdd_j > 0 must point the same way.
static void testFeedforwardSign() {
  g_case = "feedforward sign";
  std::printf("AMB feedforward sense vs the joint-space hip roll (Go2)\n");
  const QuadrupedKinematics kin(createGo2Config());
  const QuadrupedLegDynamics legs(createGo2DynamicsConfig());
  const QuadrupedDynamics wb(createGo2WholeBodyConfig());
  const MdlAMB::params_t p;
  // Level, and rolled 8 deg with some yaw: the sense must not hinge on the
  // body being level.
  const Eigen::Matrix3d Rs[2] = {
      Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(8.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()))};
  double worst_cos = 1.0, worst_dtheta = 0.0;
  for (const Eigen::Matrix3d& R : Rs) {
    MdlAMB::input_t in;
    for (int leg = 0; leg < NL; leg++) {
      T_CHECK(kin.inverseKinematics(leg, FOOT_DIAG[leg], in.q[leg]));
      in.qd[leg].setZero();
    }
    in.R = R;
    in.w_body.setZero();
    in.t = 0.0;
    MdlAMB::proj_t pr;
    T_CHECK(MdlAMB::project(p, in, legs, pr));
    QuadrupedDynamics::state_t st;
    st.R = R;
    for (int leg = 0; leg < NL; leg++) {
      st.q[leg] = in.q[leg];
      st.qd[leg].setZero();
    }
    QuadrupedDynamics::terms_t t0;
    T_CHECK(wb.compute(st, t0));
    for (int k = 0; k < 2; k++) {
      const int leg = k == 0 ? 1 : 2;
      const double j = k == 0 ? pr.j_c : pr.j_d;
      Eigen::Matrix3d Rp;
      double rho = 0.0;
      T_CHECK(MdlAMB::principalFrame(pr.s, t0.foot[leg].p - t0.hip[leg].p, Rp, rho));
      for (const double thdd : {1.0, -1.0}) {
        Eigen::Vector3d a_f;
        MdlAMB::tangentialFeedforward(rho, thdd, Rp.col(2), 50.0, a_f);
        // The joint-space path's roll for this thdd, as a small displacement.
        const double h = 1e-5 * thdd;
        st.q[leg](0) += h / j;
        QuadrupedDynamics::terms_t t1;
        T_CHECK(wb.compute(st, t1));
        MdlAMB::input_t in1 = in;
        in1.q[leg](0) += h / j;
        MdlAMB::proj_t pr1;
        T_CHECK(MdlAMB::project(p, in1, legs, pr1));
        st.q[leg](0) -= h / j;
        const Eigen::Vector3d step = t1.foot[leg].p - t0.foot[leg].p;
        worst_cos = std::min(worst_cos, step.dot(a_f) / (step.norm() * a_f.norm()));
        // ... and that roll does move theta_j by h (j's definition).
        worst_dtheta = std::max(worst_dtheta, std::fabs((pr1.theta(1 + k) - pr.theta(1 + k)) - h) / std::fabs(h));
      }
    }
  }
  // With the stance feet on the body centreline the hip-roll axis (body x) is
  // the support line itself, so the roll step lies along e_t to first order
  // in h; a sign error would read -1.
  std::printf("  %-62s worst %.6f (min 0.999)\n", "cos(roll step of the old path, a_f), FR and RL, thdd +-1", worst_cos);
  T_CHECK(worst_cos > 0.999);
  report("that roll step moves theta_j by h (relative)", worst_dtheta, 1e-3);
}

// Static stand on the real Go2 model: at rest, with zero task commands and
// F_des = weight shared equally, the QP returns forces carrying the weight
// and torques inside the motors' limits, on four feet and on FL-RR.
static void testStaticStand() {
  g_case = "static stand";
  std::printf("static stand on the Go2 model\n");
  const QuadrupedKinematics kin(createGo2Config());
  const QuadrupedDynamics wb(createGo2WholeBodyConfig());
  const Eigen::Vector3d& tau_lim = wb.getParams().legs.torque_limit;
  // Four feet under the thighs at base_z 0.30, and the diagonal posture.
  const Eigen::Vector3d foot4[NL] = {{0.1934, 0.142, -0.278},
                                     {0.1934, -0.142, -0.278},
                                     {-0.1934, 0.142, -0.278},
                                     {-0.1934, -0.142, -0.278}};
  for (int c = 0; c < 2; c++) {
    const bool four = c == 0;
    QuadrupedDynamics::state_t st;
    for (int leg = 0; leg < NL; leg++) {
      T_CHECK(kin.inverseKinematics(leg, four ? foot4[leg] : FOOT_DIAG[leg], st.q[leg]));
      st.qd[leg].setZero();
    }
    terms_t dyn;
    T_CHECK(wb.compute(st, dyn));
    const double weight = dyn.mass * wb.getParams().gravity;
    input_t in;
    in.dyn = &dyn;
    const bool* feet = four ? FEET_ALL : FEET_FL_RR;
    const int nc = numContacts(feet);
    for (int leg = 0; leg < NL; leg++) {
      in.q[leg] = st.q[leg];
      in.contact[leg] = feet[leg];
      in.f_des[leg] = Eigen::Vector3d(0.0, 0.0, weight / nc);
    }
    // MdlAMB's task set with zero commands: orientation, position and, on
    // the diagonal, the two swing feet.
    in.num_tasks = four ? 2 : 4;
    in.task[0].J.block<3, 3>(0, 3) = st.R;
    in.task[1].J.block<3, 3>(0, 0).setIdentity();
    if (!four)
      for (int k = 0; k < 2; k++) {
        in.task[2 + k].J = dyn.foot[k == 0 ? 1 : 2].J;
        in.task[2 + k].Jdv = dyn.foot[k == 0 ? 1 : 2].Jdv;
      }
    QuadrupedWBC wbc;
    output_t out;
    T_CHECK(wbc.solve(in, out));
    T_CHECK(out.qp_status == 0);
    double fz = 0.0, worst_tau = 0.0;
    for (int leg = 0; leg < NL; leg++) {
      fz += out.f[leg].z();
      worst_tau = std::max(worst_tau, (out.tau.segment<3>(3 * leg).cwiseAbs() - tau_lim).maxCoeff());
    }
    std::printf("  %s: weight %.3f N, max |tau| %.2f Nm, base acceleration correction %.2e\n",
                four ? "four feet" : "FL-RR", weight, out.tau.cwiseAbs().maxCoeff(),
                maxAbs(out.vdot.head<6>()));
    // The floating-base weight trades a correction of the base acceleration
    // against force; on FL-RR the gravity moment about the support line that
    // no contact force can make lands in the former, and leaks into the
    // vertical force row only through M's coupling. Measured 7e-4 N (four
    // feet) and 3e-3 N (FL-RR); a wrong force or gravity sign leaves O(weight).
    report(four ? "four feet: sum f_z - weight [N]" : "FL-RR: sum f_z - weight [N]",
           std::fabs(fz - weight), 1e-2);
    report(four ? "four feet: |tau| - motor limit (never positive) [Nm]"
                : "FL-RR: |tau| - motor limit (never positive) [Nm]",
           std::max(worst_tau, 0.0), 0.0);
  }
}

int main() {
  testAchievable();
  testPriority();
  testKinematic();
  testQP();
  testFailure();
  testTiming();
  testPrincipalFrame();
  testFeedforwardSign();
  testStaticStand();
  std::printf("PASS test_wbc\n");
  return 0;
}
