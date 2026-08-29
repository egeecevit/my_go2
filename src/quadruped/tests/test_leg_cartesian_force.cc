/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// Tests for MdlLegControl's Cartesian force command, equation (1) of the MIT
// Cheetah 3 convex MPC paper.
//
// The tests target the pure helper computeCartesianForceCommand() rather than
// the setCartesianForceCommand() wrapper, because the wrapper reads MotorHW and
// this repository has no mocking infrastructure and no fake MotorHW. That is
// also why the helper exists as a separate function: it is the largest part of
// the path that can be checked without a simulator. The wrapper's motor-ready
// gate and emit path are covered in simulation and by inspection.
//
// Two of these -- the J*qdot check and the J^T mapping -- are the ones worth
// having. A Cartesian controller that differentiates the wrong thing, or that
// maps force to torque with J instead of J^T, produces a leg that still moves
// and still looks roughly right, and the mistake only shows up as a gait that
// will not tune.
//
// They FAIL until the compute block inside the helper is implemented; the stub
// returns a zero torque, which is a valid shape and a wrong answer.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

// Linking pulls in MdlLegControl.o, whose module layer references the hardware
// singletons. Defining their statics satisfies the linker; none is ever
// instantiated, so MotorHW::instance() returns null and the wrapper refuses,
// which is itself checked below.
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

static const char* g_case = "(none)";

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                       \
  do {                                                                      \
    if (!(cond)) {                                                          \
      std::cerr << "FAIL [" << g_case << "] " << __FILE__ << ":" << __LINE__ \
                << ": " #cond << std::endl;                                 \
      std::exit(1);                                                         \
    }                                                                       \
  } while (0)

static void checkVecNear(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol,
                         const char* what, int line) {
  const double err = (a - b).cwiseAbs().maxCoeff();
  if (!(err <= tol)) {
    std::cerr << "FAIL [" << g_case << "] " << __FILE__ << ":" << line << ": " << what
              << " max abs difference " << err << " > " << tol << "\n"
              << "  got  [" << a.transpose() << "]\n"
              << "  want [" << b.transpose() << "]" << std::endl;
    std::exit(1);
  }
}

#define T_VEC_NEAR(a, b, tol, what) checkVecNear(a, b, tol, what, __LINE__)

// ---------------------------------------------------------------------------
//  Fixture
// ---------------------------------------------------------------------------

// An independent copy of the same kinematics the leg holds, so the expected
// values are computed from the model rather than from the code under test.
static const QuadrupedKinematics& reference() {
  static QuadrupedKinematics k(createGo2Config());
  return k;
}

// The trot.toml nominal footprint: hip position shifted by [0, +-0.10, -0.28].
static Eigen::Vector3d nominalFoot(int leg) {
  Eigen::Vector3d p = reference().getKinematicParams().hip_positions.row(leg).transpose();
  p[1] += 0.10 * (leg % 2 == 0 ? 1.0 : -1.0);
  p[2] += -0.28;
  return p;
}

static Eigen::Vector3d nominalAngles(int leg) {
  Eigen::Vector3d q;
  const bool ok = reference().inverseKinematics(leg, nominalFoot(leg), q);
  if (!ok) {
    std::cerr << "FAIL [" << g_case << "] fixture: IK failed for leg " << leg << std::endl;
    std::exit(1);
  }
  return q;
}

// What equation (1) says the answer is, computed straight from the reference
// kinematics with no shortcuts.
static Eigen::Vector3d expectedTorque(int leg, const Eigen::Vector3d& q,
                                      const Eigen::Vector3d& qdot,
                                      const Eigen::Vector3d& pref,
                                      const Eigen::Vector3d& vref,
                                      const Eigen::Vector3d& kp,
                                      const Eigen::Vector3d& kd,
                                      const Eigen::Vector3d& fff) {
  Eigen::Vector3d p;
  Eigen::Matrix3d J;
  reference().forwardKinematicsUnchecked(leg, q, p);
  reference().jacobian(leg, q, J);
  const Eigen::Vector3d v = J * qdot;
  const Eigen::Vector3d f = kp.cwiseProduct(pref - p) + kd.cwiseProduct(vref - v) + fff;
  return J.transpose() * f;
}

// A leg module with its kinematics built and no hardware behind it.
struct TestLeg {
  MdlLegControl leg;
  explicit TestLeg(int index) : leg(index) { leg.init(); }
  ~TestLeg() { leg.uninit(); }
};

// ---------------------------------------------------------------------------
//  1. Measured foot velocity comes from J*qdot
// ---------------------------------------------------------------------------

void test_velocity_from_jacobian() {
  g_case = "velocity_from_jacobian";
  std::cout << "test_velocity_from_jacobian..." << std::endl;

  const int leg = 0;
  TestLeg t(leg);

  const Eigen::Vector3d q = nominalAngles(leg);
  // Deliberately asymmetric, so an axis swap or a dropped column cannot hide.
  const Eigen::Vector3d qdot(0.5, -0.3, 0.7);
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const Eigen::Vector3d kd(10.0, 12.0, 14.0);

  // Pure damping about zero reference velocity, so the whole torque is the
  // velocity term and nothing else can compensate for getting it wrong.
  Eigen::Vector3d tau;
  T_CHECK(t.leg.computeCartesianForceCommand(q, qdot, zero, zero, zero, kd, zero, tau));

  Eigen::Matrix3d J;
  reference().jacobian(leg, q, J);
  const Eigen::Vector3d want = J.transpose() * (-kd.cwiseProduct(J * qdot));
  T_VEC_NEAR(tau, want, 1e-9, "torque from measured foot velocity");

  // And the plausible wrong answer -- damping the joint rate directly instead
  // of the foot velocity, which is what using the IK's desired qdot amounts to
  // -- is a genuinely different number, so the check above has teeth.
  const Eigen::Vector3d wrong = J.transpose() * (-kd.cwiseProduct(qdot));
  T_CHECK((want - wrong).cwiseAbs().maxCoeff() > 1.0);

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  2. The torque is J^T f_cmd, on every leg
// ---------------------------------------------------------------------------

void test_jacobian_transpose_mapping() {
  g_case = "jacobian_transpose_mapping";
  std::cout << "test_jacobian_transpose_mapping..." << std::endl;

  // All four legs: the right and left conventions differ by a joint direction
  // sign on abduction, and J^T carries it, so a leg-independent shortcut is
  // wrong on half the robot.
  for (int leg = 0; leg < 4; leg++) {
    TestLeg t(leg);

    const Eigen::Vector3d q = nominalAngles(leg);
    const Eigen::Vector3d qdot(0.2, -0.4, 0.15);
    const Eigen::Vector3d pref = nominalFoot(leg) + Eigen::Vector3d(0.01, -0.02, 0.03);
    const Eigen::Vector3d vref(0.05, 0.02, -0.03);
    const Eigen::Vector3d kp(300.0, 400.0, 500.0);
    const Eigen::Vector3d kd(5.0, 6.0, 7.0);
    const Eigen::Vector3d fff(3.0, -7.0, 40.0);

    Eigen::Vector3d tau;
    T_CHECK(t.leg.computeCartesianForceCommand(q, qdot, pref, vref, kp, kd, fff, tau));
    T_VEC_NEAR(tau, expectedTorque(leg, q, qdot, pref, vref, kp, kd, fff), 1e-9,
               "J^T f_cmd");

    // J, not J^T. The two agree only for a symmetric Jacobian, which this one
    // is not, and confusing them is the classic way to get a leg that pushes
    // sideways when it is asked to push down.
    Eigen::Matrix3d J;
    Eigen::Vector3d p;
    reference().jacobian(leg, q, J);
    reference().forwardKinematicsUnchecked(leg, q, p);
    const Eigen::Vector3d f =
        kp.cwiseProduct(pref - p) + kd.cwiseProduct(vref - J * qdot) + fff;
    T_CHECK((J.transpose() * f - J * f).cwiseAbs().maxCoeff() > 1.0);
  }

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  3. Zero Cartesian gains leave only the feedforward
// ---------------------------------------------------------------------------

void test_pure_feedforward() {
  g_case = "pure_feedforward";
  std::cout << "test_pure_feedforward..." << std::endl;

  // This is the stance case: no position loop on a planted foot at all, just
  // the ground reaction force the MPC asked for, mapped through J^T.
  for (int leg = 0; leg < 4; leg++) {
    TestLeg t(leg);

    const Eigen::Vector3d q = nominalAngles(leg);
    const Eigen::Vector3d qdot(0.3, 0.1, -0.2);
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    // The force the actuators apply at the foot for a supporting stance leg:
    // downwards in the body frame, which is -R^T times an upward world ground
    // reaction force. The caller has already flipped the sign, so the helper
    // must use it as given.
    const Eigen::Vector3d fff(2.0, -1.0, -75.0);

    Eigen::Matrix3d J;
    reference().jacobian(leg, q, J);

    Eigen::Vector3d tau;
    T_CHECK(t.leg.computeCartesianForceCommand(q, qdot, zero, zero, zero, zero, fff, tau));
    T_VEC_NEAR(tau, J.transpose() * fff, 1e-9, "feedforward only torque");

    // The references are genuinely unused, not merely small: a stance command
    // passes whatever reference happens to be lying around, and if that leaked
    // into the torque the stance force would depend on the swing trajectory.
    Eigen::Vector3d tau2;
    T_CHECK(t.leg.computeCartesianForceCommand(q, qdot, Eigen::Vector3d(9.0, -9.0, 9.0),
                                               Eigen::Vector3d(-4.0, 4.0, -4.0), zero,
                                               zero, fff, tau2));
    T_VEC_NEAR(tau2, tau, 0.0, "reference ignored at zero gain");
  }

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  4. Rejection
// ---------------------------------------------------------------------------

void test_rejection() {
  g_case = "rejection";
  std::cout << "test_rejection..." << std::endl;

  const int leg = 1;
  TestLeg t(leg);

  const Eigen::Vector3d q = nominalAngles(leg);
  const Eigen::Vector3d qdot(0.1, 0.2, 0.3);
  const Eigen::Vector3d pref = nominalFoot(leg);
  const Eigen::Vector3d vref = Eigen::Vector3d::Zero();
  const Eigen::Vector3d kp = Eigen::Vector3d::Constant(300.0);
  const Eigen::Vector3d kd = Eigen::Vector3d::Constant(6.0);
  const Eigen::Vector3d fff = Eigen::Vector3d::Zero();

  Eigen::Vector3d tau;
  T_CHECK(t.leg.computeCartesianForceCommand(q, qdot, pref, vref, kp, kd, fff, tau));

  // Every argument, one at a time, with both flavours of non-finite. A single
  // bad joint reading must not reach the motors through some intermediate that
  // happens to stay finite.
  const double bad[] = {std::nan(""), std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()};
  for (double v : bad) {
    for (int axis = 0; axis < 3; axis++) {
      Eigen::Vector3d a;

      a = q;    a[axis] = v;
      T_CHECK(!t.leg.computeCartesianForceCommand(a, qdot, pref, vref, kp, kd, fff, tau));
      a = qdot; a[axis] = v;
      T_CHECK(!t.leg.computeCartesianForceCommand(q, a, pref, vref, kp, kd, fff, tau));
      a = pref; a[axis] = v;
      T_CHECK(!t.leg.computeCartesianForceCommand(q, qdot, a, vref, kp, kd, fff, tau));
      a = vref; a[axis] = v;
      T_CHECK(!t.leg.computeCartesianForceCommand(q, qdot, pref, a, kp, kd, fff, tau));
      a = kp;   a[axis] = v;
      T_CHECK(!t.leg.computeCartesianForceCommand(q, qdot, pref, vref, a, kd, fff, tau));
      a = kd;   a[axis] = v;
      T_CHECK(!t.leg.computeCartesianForceCommand(q, qdot, pref, vref, kp, a, fff, tau));
      a = fff;  a[axis] = v;
      T_CHECK(!t.leg.computeCartesianForceCommand(q, qdot, pref, vref, kp, kd, a, tau));
    }
  }

  // Finite inputs whose product is not. This is the case the input checks
  // cannot catch and the reason the result has to be checked as well: nothing
  // here is infinite until the multiplication happens.
  {
    const Eigen::Vector3d hugeKp = Eigen::Vector3d::Constant(1e200);
    const Eigen::Vector3d hugeRef = Eigen::Vector3d::Constant(1e200);
    T_CHECK(!t.leg.computeCartesianForceCommand(q, qdot, hugeRef, vref, hugeKp, kd, fff,
                                                tau));
  }
  {
    const Eigen::Vector3d hugeKd = Eigen::Vector3d::Constant(1e200);
    const Eigen::Vector3d hugeVel = Eigen::Vector3d::Constant(1e200);
    T_CHECK(!t.leg.computeCartesianForceCommand(q, qdot, pref, hugeVel, kp, hugeKd, fff,
                                                tau));
  }

  // The wrapper refuses outright with no MotorHW behind it, which is the same
  // path a motor that is not READY takes. It is the only part of the wrapper
  // reachable without a simulator, and it is the part that matters: nothing is
  // emitted when the joint state cannot be trusted.
  T_CHECK(!t.leg.setCartesianForceCommand(pref, vref, kp, kd, fff, 0.2));

  std::cout << "  PASS" << std::endl;
}

void test_inverse_dynamics_swing_command() {
  g_case = "inverse_dynamics_swing_command";
  std::cout << "test_inverse_dynamics_swing_command..." << std::endl;

  const int leg = 0;
  TestLeg t(leg);
  T_CHECK(t.leg.hasSwingDynamics());

  const Eigen::Vector3d q = nominalAngles(leg);
  const Eigen::Vector3d qdot(0.35, -0.8, 1.15);
  QuadrupedLegDynamics dynamics(createGo2DynamicsConfig());
  QuadrupedLegDynamics::terms_t terms;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  T_CHECK(dynamics.compute(leg, q, qdot, gravity, terms));

  const Eigen::Vector3d pref = terms.foot_position + Eigen::Vector3d(0.006, -0.004, 0.008);
  const Eigen::Vector3d vref = terms.foot_jacobian * qdot +
                               Eigen::Vector3d(0.03, -0.02, 0.015);
  const Eigen::Vector3d aref(1.2, -0.7, 2.4);
  const Eigen::Vector3d omega(28.0, 31.0, 34.0);
  const Eigen::Vector3d zeta(0.8, 0.9, 1.0);
  const double joint_damping = 0.2;

  MdlLegControl::command_result_t result;
  T_CHECK(t.leg.computeSwingCommand(q, qdot, pref, vref, aref, gravity, omega, zeta,
                                    1.0, joint_damping, result));

  const Eigen::Vector3d kp = omega.array().square() *
                             terms.operational_inertia.diagonal().array();
  const Eigen::Vector3d kd = 2.0 * zeta.array() * omega.array() *
                             terms.operational_inertia.diagonal().array();
  const Eigen::Vector3d feedback =
      terms.foot_jacobian.transpose() *
      (kp.cwiseProduct(pref - terms.foot_position) +
       kd.cwiseProduct(vref - terms.foot_jacobian * qdot));
  const Eigen::Vector3d feedforward =
      terms.foot_jacobian.transpose() * terms.operational_inertia *
          (aref - terms.jdot_qdot) +
      terms.bias;
  const Eigen::Vector3d requested = feedback + feedforward - joint_damping * qdot;

  T_VEC_NEAR(result.kp_cartesian, kp, 1e-9, "apparent-mass kp");
  T_VEC_NEAR(result.kd_cartesian, kd, 1e-9, "apparent-mass kd");
  T_VEC_NEAR(result.feedback_torque, feedback, 1e-9, "equation 1 feedback");
  T_VEC_NEAR(result.feedforward_torque, feedforward, 1e-9, "equation 2 feedforward");
  T_VEC_NEAR(result.requested_torque, requested, 1e-9, "net requested torque");
  T_CHECK(result.torque_scale > 0.999999);
  T_VEC_NEAR(result.applied_torque, requested, 1e-9, "unsaturated torque");
  // The heavy-legged Go2 ships with this term gated off until the MPC models
  // the equal-and-opposite leg momentum. The gate must remove equation (2)
  // without removing the apparent-mass gains from equation (3).
  MdlLegControl::command_result_t feedbackOnly;
  T_CHECK(t.leg.computeSwingCommand(q, qdot, pref, vref, aref, gravity, omega, zeta,
                                    0.0, joint_damping, feedbackOnly));
  T_VEC_NEAR(feedbackOnly.kp_cartesian, result.kp_cartesian, 0.0,
             "feedforward gate preserves kp");
  T_VEC_NEAR(feedbackOnly.kd_cartesian, result.kd_cartesian, 0.0,
             "feedforward gate preserves kd");
  T_VEC_NEAR(feedbackOnly.feedforward_torque, Eigen::Vector3d::Zero(), 0.0,
             "feedforward gate");

  // A deliberately impossible acceleration must be scaled as one leg vector,
  // preserving direction rather than independently clipping three axes.
  MdlLegControl::command_result_t limited;
  T_CHECK(t.leg.computeSwingCommand(q, qdot, pref, vref,
                                    Eigen::Vector3d(2000.0, -1500.0, 3000.0), gravity,
                                    omega, zeta, 1.0, joint_damping, limited));
  T_CHECK(limited.torque_scale > 0.0 && limited.torque_scale < 1.0);
  T_VEC_NEAR(limited.applied_torque,
             limited.torque_scale * limited.requested_torque, 1e-9,
             "uniform torque scaling");
  const Eigen::Vector3d limits(23.7, 23.7, 45.43);
  T_CHECK((limited.applied_torque.cwiseAbs() - limits).maxCoeff() <= 1e-10);

  std::cout << "  PASS" << std::endl;
}

int main() {
  std::cout << "=== Leg Cartesian Force Tests ===" << std::endl;
  test_velocity_from_jacobian();
  test_jacobian_transpose_mapping();
  test_pure_feedforward();
  test_rejection();
  test_inverse_dynamics_swing_command();
  std::cout << "All leg Cartesian force tests passed." << std::endl;
  return 0;
}
