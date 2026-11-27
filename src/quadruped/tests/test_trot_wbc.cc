/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

// MdlTrot::wbcSolve, the whole-body impulse-control trot tick (Kim, Di Carlo,
// Katz, Bledt, Kim 2019), on the real Go2 model, no simulator:
//   (i)   four feet at rest, statically consistent weight split: the QP keeps
//         the forces, nothing accelerates, the torques are the static ones;
//   (ii)  diagonal stance at random states: the hierarchy holds the contact
//         points, the solved forces obey the pyramid and normal bounds;
//   (iii) with the body off its reference, the swing joint references are
//         still the leg-relative ones, and the swing acceleration command is
//         the absolute acceleration of a point following the body-frame
//         reference (finite differences);
//   (iv)  the swing joints accelerate as the body-frame law asks, whatever the
//         body tasks command.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlTrot.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

// Linking MdlTrot.o pulls in its module layer, which references the hardware
// singletons; none is instantiated here (test_trot_gait.cc does the same).
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

typedef QuadrupedDynamics::VectorV VectorV;
static constexpr int NL = MdlTrot::NUM_LEGS;
static constexpr int TRIALS = 200;
// Hierarchy residuals relative to the size of the solution, as in test_wbc.cc:
// round-off grows with |vdot| and the stacked Jacobian's condition.
static constexpr double TOL_REL = 1e-9;
// qpOASES meets bounds and constraints to ~1e-10 of the data.
static constexpr double TOL_QP = 1e-6;

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
  std::printf("  %-66s worst %.2e (tol %.0e)\n", what, worst, tol);
  if (!(worst <= tol)) {
    std::fprintf(stderr, "FAIL [%s] %s: %.3e > %.0e\n", g_case, what, worst, tol);
    std::exit(1);
  }
}

static std::mt19937 g_rng(20261126);

static double uni(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(g_rng);
}

static Eigen::Vector3d uni3(double lo, double hi) {
  return Eigen::Vector3d(uni(lo, hi), uni(lo, hi), uni(lo, hi));
}

static Eigen::Matrix3d rpy(double roll, double pitch, double yaw) {
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

// Kim's locomotion weighting with mpc.toml's friction and normal bounds, as
// MdlTrot::activate() sets them.
static QuadrupedWBC::params_t kimParams() {
  QuadrupedWBC::params_t p;
  p.w_base = 0.1;
  p.w_force = 1.0;
  p.friction = 0.6;
  p.fz_min = 0.0;
  p.fz_max = 200.0;
  return p;
}

// trot.toml's nominal footprint: hip + [0, +-0.10, -0.28], body frame.
static Eigen::Vector3d nominalFoot(const QuadrupedKinematics& kin, int leg) {
  Eigen::Vector3d p = kin.getKinematicParams().hip_positions.row(leg).transpose();
  p += Eigen::Vector3d(0.0, leg % 2 == 0 ? 0.10 : -0.10, -0.28);
  return p;
}

static const bool FEET_ALL[NL] = {true, true, true, true};
static const bool FEET_DIAG[2][NL] = {{true, false, false, true}, {false, true, true, false}};
static const bool FEET_NONE[NL] = {false, false, false, false};

// A trot-like random state: tilted up to 0.2 rad with any heading, moving and
// turning, stance feet near the footprint, swing feet lifted, joints moving
// (swing joints at rest when swingAtRest).
static void randomState(const QuadrupedKinematics& kin, const bool stance[NL], bool swingAtRest,
                        MdlTrot::wbc_input_t& in) {
  in.state.R = rpy(uni(-0.2, 0.2), uni(-0.2, 0.2), uni(-M_PI, M_PI));
  in.state.v_base = uni3(-0.8, 0.8);
  in.state.w_body = uni3(-1.5, 1.5);
  in.p_base = Eigen::Vector3d(uni(-1.0, 1.0), uni(-1.0, 1.0), uni(0.25, 0.32));
  for (int leg = 0; leg < NL; leg++) {
    Eigen::Vector3d p = nominalFoot(kin, leg) + Eigen::Vector3d(uni(-0.04, 0.04), uni(-0.03, 0.03), 0.0);
    if (!stance[leg]) p.z() += uni(0.02, 0.10);
    T_CHECK(kin.inverseKinematics(leg, p, in.state.q[leg]));
    in.state.qd[leg] = (swingAtRest && !stance[leg]) ? Eigen::Vector3d::Zero() : uni3(-3.0, 3.0);
    in.stance[leg] = stance[leg];
  }
}

// Body references off the state by up to a few degrees and centimetres.
static void randomBodyReference(MdlTrot::wbc_input_t& in) {
  in.R_des = rpy(uni(-0.05, 0.05), uni(-0.05, 0.05), uni(-0.1, 0.1)) * in.state.R;
  in.w_des = uni3(-0.5, 0.5);
  in.p_des = in.p_base + uni3(-0.03, 0.03);
  in.v_des = in.state.v_base + uni3(-0.3, 0.3);
}

// MPC-like desired forces: a share of the weight with horizontal parts that
// sometimes leave the 0.6 pyramid, so the QP's bounds are exercised.
static void randomForces(MdlTrot::wbc_input_t& in) {
  for (int leg = 0; leg < NL; leg++) {
    in.f_des[leg].setZero();
    if (!in.stance[leg]) continue;
    const double fz = uni(30.0, 120.0);
    in.f_des[leg] = Eigen::Vector3d(uni(-0.9, 0.9) * fz, uni(-0.9, 0.9) * fz, fz);
  }
}

// The swing feet's measured body-frame position and velocity, from
// QuadrupedKinematics rather than the whole-body model under test.
static void measuredFoot(const QuadrupedKinematics& kin, const MdlTrot::wbc_input_t& in, int leg,
                         Eigen::Vector3d& p_B, Eigen::Vector3d& v_B) {
  Eigen::Matrix3d J;
  T_CHECK(kin.forwardKinematicsUnchecked(leg, in.state.q[leg], p_B));
  T_CHECK(kin.jacobian(leg, in.state.q[leg], J));
  v_B = J * in.state.qd[leg];
}

static double maxAbs(const Eigen::VectorXd& v) { return v.cwiseAbs().maxCoeff(); }

// (i) Level, at rest, all four feet down, F_des the weight split so that it
// balances gravity about the COM. Then the hierarchy's vdot is zero (no task
// asks for anything), the QP's equality is met with no correction, and the
// torque is the static one: per-leg gravity minus J^T f.
static void testStaticStand() {
  g_case = "static stand";
  std::printf("(i) four feet at rest, consistent weight split\n");
  const QuadrupedKinematics kin(createGo2Config());
  const QuadrupedDynamics model(createGo2WholeBodyConfig());
  const QuadrupedLegDynamics legs(createGo2DynamicsConfig());
  QuadrupedWBC wbc;
  wbc.setParams(kimParams());

  MdlTrot::wbc_input_t in;
  in.p_base = Eigen::Vector3d(0.0, 0.0, 0.30);
  for (int leg = 0; leg < NL; leg++) {
    T_CHECK(kin.inverseKinematics(leg, nominalFoot(kin, leg), in.state.q[leg]));
    in.state.qd[leg].setZero();
    in.stance[leg] = true;
  }
  QuadrupedDynamics::terms_t dyn;
  T_CHECK(model.compute(in.state, dyn));
  in.dyn = &dyn;
  in.p_des = in.p_base;

  // Vertical forces f_i with sum f_i = M g and sum f_i (p_i - c) x e_z = 0;
  // the minimum-norm one, about a quarter of the weight each.
  const double weight = dyn.mass * model.getParams().gravity;
  Eigen::Matrix<double, 3, NL> A;
  for (int leg = 0; leg < NL; leg++) {
    const Eigen::Vector3d d = dyn.contact[leg].p - dyn.com;
    A.col(leg) = Eigen::Vector3d(1.0, d.x(), d.y());
  }
  const Eigen::Vector4d fz =
      A.transpose() * (A * A.transpose()).ldlt().solve(Eigen::Vector3d(weight, 0.0, 0.0));
  for (int leg = 0; leg < NL; leg++) in.f_des[leg] = Eigen::Vector3d(0.0, 0.0, fz[leg]);

  MdlTrot::wbc_output_t out;
  T_CHECK(MdlTrot::wbcSolve(wbc, in, out));
  T_CHECK(out.result.qp_status == 0);
  T_CHECK(out.problem.num_tasks == 2);

  double worst_f = 0.0, worst_tau = 0.0;
  for (int leg = 0; leg < NL; leg++) {
    worst_f = std::max(worst_f, maxAbs(out.result.f[leg] - in.f_des[leg]));
    QuadrupedLegDynamics::terms_t lt;
    T_CHECK(legs.compute(leg, in.state.q[leg], Eigen::Vector3d::Zero(),
                         Eigen::Vector3d(0.0, 0.0, -model.getParams().gravity), lt));
    // Level body, so body and world axes agree; a vertical force at the
    // contact point and at the sphere centre load the joints alike.
    const Eigen::Vector3d tau = lt.bias - lt.foot_jacobian.transpose() * out.result.f[leg];
    worst_tau = std::max(worst_tau, maxAbs(out.result.tau.segment<3>(3 * leg) - tau));
  }
  std::printf("  weight %.3f N split %.2f / %.2f / %.2f / %.2f N\n", weight, fz[0], fz[1], fz[2],
              fz[3]);
  report("solved force - F_des [N]", worst_f, TOL_QP);
  report("|vdot| (base and joints)", maxAbs(out.result.vdot), TOL_REL);
  report("tau - (leg gravity - J^T f) [Nm]", worst_tau, 1e-8);
}

// (ii) Diagonal stance, random states and MPC-like forces. The hierarchy
// keeps the contact points still (J_c vdot_h + Jdv = 0). The QP corrects only
// the floating base, so the joint accelerations are the hierarchy's; the
// base correction is Kim's "the body gives way" and does move the contact
// points in the model, which is reported, not checked.
static void testDiagonalStance() {
  g_case = "diagonal stance";
  std::printf("(ii) diagonal stance, %d random states\n", TRIALS);
  const QuadrupedKinematics kin(createGo2Config());
  const QuadrupedDynamics model(createGo2WholeBodyConfig());
  QuadrupedWBC wbc;
  const QuadrupedWBC::params_t p = kimParams();
  wbc.setParams(p);

  double worst_contact = 0.0, worst_joint = 0.0, worst_pyramid = 0.0, worst_fz = 0.0;
  double max_slip = 0.0;
  for (int trial = 0; trial < TRIALS; trial++) {
    MdlTrot::wbc_input_t in;
    randomState(kin, FEET_DIAG[trial % 2], false, in);
    QuadrupedDynamics::terms_t dyn;
    T_CHECK(model.compute(in.state, dyn));
    in.dyn = &dyn;
    randomBodyReference(in);
    randomForces(in);
    for (int leg = 0; leg < NL; leg++) {
      in.foot_p[leg] = nominalFoot(kin, leg) + uni3(-0.05, 0.05);
      in.foot_v[leg] = uni3(-1.0, 1.0);
      in.foot_a[leg] = uni3(-30.0, 30.0);
    }
    MdlTrot::wbc_output_t out;
    T_CHECK(MdlTrot::wbcSolve(wbc, in, out));
    T_CHECK(out.result.qp_status == 0);
    T_CHECK(out.problem.num_tasks == 4);

    VectorV vdot_h;
    Eigen::Vector3d q_des[NL], qd_des[NL];
    T_CHECK(QuadrupedWBC::hierarchy(out.problem, vdot_h, q_des, qd_des));
    const double scale = 1.0 + maxAbs(vdot_h);
    worst_joint = std::max(worst_joint, maxAbs(out.result.vdot.tail<12>() - vdot_h.tail<12>()) / scale);
    for (int leg = 0; leg < NL; leg++) {
      const Eigen::Vector3d& f = out.result.f[leg];
      if (!in.stance[leg]) {
        T_CHECK(f.isZero(0.0));
        continue;
      }
      const QuadrupedDynamics::point_t& c = dyn.contact[leg];
      worst_contact = std::max(worst_contact, maxAbs(c.J * vdot_h + c.Jdv) / scale);
      max_slip = std::max(max_slip, maxAbs(c.J * out.result.vdot + c.Jdv));
      worst_pyramid = std::max({worst_pyramid, std::fabs(f.x()) - p.friction * f.z(),
                                std::fabs(f.y()) - p.friction * f.z()});
      worst_fz = std::max({worst_fz, p.fz_min - f.z(), f.z() - p.fz_max});
    }
  }
  report("hierarchy: |J_c vdot_h + Jdv| / (1 + |vdot_h|)", worst_contact, TOL_REL);
  report("QP leaves the joint accelerations: |vdot - vdot_h|_joints (rel)", worst_joint, TOL_REL);
  report("solved force outside the friction pyramid [N]", std::max(worst_pyramid, 0.0), TOL_QP);
  report("solved normal force outside [fz_min, fz_max] [N]", std::max(worst_fz, 0.0), TOL_QP);
  std::printf("  contact acceleration after the QP's base correction, max %.2f m/s^2 "
              "(Kim: the base gives way; not checked)\n", max_slip);
}

// (iii) Swing task errors and the finite-difference check of its command.
static void testSwingTask() {
  g_case = "swing task";
  std::printf("(iii) swing task, %d random moving states\n", TRIALS);
  const QuadrupedKinematics kin(createGo2Config());
  const QuadrupedDynamics model(createGo2WholeBodyConfig());
  QuadrupedWBC wbc;
  wbc.setParams(kimParams());

  double worst_q = 0.0, worst_qd = 0.0, worst_fd = 0.0;
  for (int trial = 0; trial < TRIALS; trial++) {
    MdlTrot::wbc_input_t in;
    randomState(kin, FEET_DIAG[trial % 2], false, in);
    QuadrupedDynamics::terms_t dyn;
    T_CHECK(model.compute(in.state, dyn));
    in.dyn = &dyn;
    randomBodyReference(in);
    randomForces(in);
    for (int leg = 0; leg < NL; leg++) {
      in.foot_p[leg] = nominalFoot(kin, leg) + uni3(-0.05, 0.05);
      in.foot_v[leg] = uni3(-1.0, 1.0);
      in.foot_a[leg] = uni3(-30.0, 30.0);
    }
    const Eigen::Matrix3d& R = in.state.R;

    // Joint references with the body references off the state: the body
    // levels move the base, and the swing joints still get the leg-relative
    // J_leg (q_des - q) = p_B_des - p_B and J_leg qd_des = v_B_des.
    MdlTrot::wbc_output_t out;
    T_CHECK(MdlTrot::wbcSolve(wbc, in, out));
    for (int leg = 0; leg < NL; leg++) {
      if (in.stance[leg]) continue;
      Eigen::Vector3d p_B, v_B;
      measuredFoot(kin, in, leg, p_B, v_B);
      Eigen::Matrix3d J;
      T_CHECK(kin.jacobian(leg, in.state.q[leg], J));
      worst_q = std::max(worst_q, maxAbs(J * (out.result.q_des[leg] - in.state.q[leg]) -
                                         (in.foot_p[leg] - p_B)));
      worst_qd = std::max(worst_qd, maxAbs(J * out.result.qd_des[leg] - in.foot_v[leg]));
    }

    // Finite differences: a zero-error reference (foot_p, foot_v = measured)
    // with a quadratic body-frame path, and body references equal to the
    // state, so the body tasks command no acceleration and the body keeps its
    // velocity and rate. The reference point is then
    //   x(t) = p_base + v_base t + exp([w] t) R (p_B + v_B t + a t^2 / 2).
    in.R_des = R;
    in.w_des = R * in.state.w_body;
    in.p_des = in.p_base;
    in.v_des = in.state.v_base;
    for (int leg = 0; leg < NL; leg++) measuredFoot(kin, in, leg, in.foot_p[leg], in.foot_v[leg]);
    T_CHECK(MdlTrot::wbcSolve(wbc, in, out));
    const Eigen::Vector3d w = R * in.state.w_body;
    int k = 2;  // swing tasks follow the body tasks in leg order
    for (int leg = 0; leg < NL; leg++) {
      if (in.stance[leg]) continue;
      auto x = [&](double t) -> Eigen::Vector3d {
        const Eigen::Matrix3d Rt = Eigen::AngleAxisd(w.norm() * t, w.normalized()).toRotationMatrix() * R;
        return in.p_base + in.state.v_base * t +
               Rt * (in.foot_p[leg] + in.foot_v[leg] * t + 0.5 * in.foot_a[leg] * t * t);
      };
      // h = 1e-4: truncation ~h^2/12 |x''''| ~ 1e-6, round-off ~eps |x| / h^2 ~ 1e-8.
      const double h = 1e-4;
      const Eigen::Vector3d fd = ((x(h) - x(0.0)) - (x(0.0) - x(-h))) / (h * h);
      const Eigen::Vector3d& acc_cmd = out.problem.task[k++].acc_cmd;
      worst_fd = std::max(worst_fd, maxAbs(acc_cmd - fd) / (1.0 + maxAbs(fd)));
    }
  }
  report("J_leg (q_des - q) - (p_B_des - p_B) [m]", worst_q, 1e-10);
  report("J_leg qd_des - v_B_des [m/s]", worst_qd, 1e-10);
  report("zero-error acc_cmd - finite-difference acceleration (rel)", worst_fd, 1e-5);
}

// (iv) Swing joints at rest, body tasks off their references so the body
// accelerates and turns, foot gains unlike the body gains: the swing joints
// accelerate as the body-frame law asks, J_leg qdd = a_B_des + kp e + kd e'
// (at rest p_B'' = J_leg qdd), whatever the body tasks command. Every third
// draw is a flight phase (duty below 0.5): all four feet swing, no force acts,
// so the floating-base rows of M vdot + h vanish (the base falls freely) while
// the legs still follow the law.
static void testSwingRidesBody() {
  g_case = "swing rides body";
  std::printf("(iv) body-frame swing law, body tasks commanding, %d random states\n", TRIALS);
  const QuadrupedKinematics kin(createGo2Config());
  const QuadrupedDynamics model(createGo2WholeBodyConfig());
  QuadrupedWBC wbc;
  wbc.setParams(kimParams());

  double worst = 0.0, worst_fall = 0.0, min_body = INFINITY;
  for (int trial = 0; trial < TRIALS; trial++) {
    MdlTrot::wbc_input_t in;
    const bool flight = trial % 3 == 2;
    randomState(kin, flight ? FEET_NONE : FEET_DIAG[trial % 2], true, in);
    QuadrupedDynamics::terms_t dyn;
    T_CHECK(model.compute(in.state, dyn));
    in.dyn = &dyn;
    randomBodyReference(in);
    randomForces(in);
    in.gains.kp_foot = 150.0;
    in.gains.kd_foot = 12.0;
    Eigen::Vector3d p_B[NL];
    for (int leg = 0; leg < NL; leg++) {
      Eigen::Vector3d v_B;  // zero for the swing legs
      measuredFoot(kin, in, leg, p_B[leg], v_B);
      in.foot_p[leg] = p_B[leg] + uni3(-0.02, 0.02);
      in.foot_v[leg] = uni3(-1.0, 1.0);
      in.foot_a[leg] = uni3(-30.0, 30.0);
    }
    MdlTrot::wbc_output_t out;
    T_CHECK(MdlTrot::wbcSolve(wbc, in, out));
    T_CHECK(out.result.qp_status == 0);
    const double scale = 1.0 + maxAbs(out.result.vdot);
    if (flight) {
      T_CHECK(out.problem.num_tasks == 6);
      const VectorV gen = dyn.M * out.result.vdot + dyn.h;
      worst_fall = std::max(worst_fall, maxAbs(gen.head<6>()) / (1.0 + maxAbs(dyn.h)));
      T_CHECK(maxAbs(gen.tail<12>() - out.result.tau) < 1e-12);
    }
    for (int leg = 0; leg < NL; leg++) {
      if (in.stance[leg]) continue;
      Eigen::Matrix3d J;
      T_CHECK(kin.jacobian(leg, in.state.q[leg], J));
      const Eigen::Vector3d law = in.foot_a[leg] + in.gains.kp_foot * (in.foot_p[leg] - p_B[leg]) +
                                  in.gains.kd_foot * in.foot_v[leg];
      worst = std::max(worst, maxAbs(J * out.result.vdot.segment<3>(6 + 3 * leg) - law) / scale);
    }
    min_body = std::min(min_body, out.problem.task[1].acc_cmd.norm() + out.problem.task[0].acc_cmd.norm());
  }
  std::printf("  smallest body task command |a_base| + |alpha| over the draws: %.2f\n", min_body);
  report("J_leg qdd - (a_B_des + kp e + kd e') / (1 + |vdot|)", worst, TOL_REL);
  report("flight: floating-base rows of M vdot + h (rel)", worst_fall, TOL_QP);
}

int main() {
  testStaticStand();
  testDiagonalStance();
  testSwingTask();
  testSwingRidesBody();
  std::printf("PASS test_trot_wbc\n");
  return 0;
}
