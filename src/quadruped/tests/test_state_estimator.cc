/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// Tests for the two-stage state estimator ported from MIT Cheetah-Software:
// MdlOrientationEstimator and the linear Kalman filter in MdlPosVelEstimator.
//
// The tests that matter most are the frame convention checks. MIT's rotation
// matrix is world to body and ours is body to world, so every transform in the
// port had to be flipped. A flip that was missed still runs and still produces
// a plausible looking trajectory, and it happens to be invisible while the robot
// is level, which is exactly the condition a casual test uses. So the attitude
// tests are all repeated at a nonzero roll, where a transposed rotation gives a
// visibly wrong answer.
//
// Both modules are driven through step(), never through update(). Module's
// constructor and destructor never touch the manager, so these can be stack
// allocated and exercised with no ModuleManager, no config and no hardware.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

#include "hardware/MotorHW.hh"
#include "quadruped/MdlOrientationEstimator.hh"
#include "quadruped/MdlPosVelEstimator.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

// Linking pulls in the whole estimator translation units, whose module layers
// reference the hardware singletons. Defining their statics satisfies the
// linker; none is ever instantiated here, so instance() simply returns null.
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                   \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << ": " #cond \
                << std::endl;                                           \
      std::exit(1);                                                     \
    }                                                                   \
  } while (0)

#define T_NEAR(a, b, tol)                                                      \
  do {                                                                         \
    double _d = std::fabs((a) - (b));                                          \
    if (!(_d <= (tol))) {                                                      \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b \
                << "| = " << _d << " > " << (tol) << std::endl;                \
      std::exit(1);                                                            \
    }                                                                          \
  } while (0)

static void checkMatrixNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                            double tol, const char* what, int line) {
  T_CHECK(a.rows() == b.rows() && a.cols() == b.cols());
  const double err = (a - b).cwiseAbs().maxCoeff();
  if (!(err <= tol)) {
    std::cerr << "FAIL " << __FILE__ << ":" << line << ": " << what
              << " max abs difference " << err << " > " << tol << std::endl;
    // Print the worst offending entry to make a sign slip obvious.
    int r = 0, c = 0;
    (a - b).cwiseAbs().maxCoeff(&r, &c);
    std::cerr << "  worst at (" << r << "," << c << "): got " << a(r, c)
              << " expected " << b(r, c) << std::endl;
    std::exit(1);
  }
}

#define T_MATRIX_NEAR(a, b, tol, what) checkMatrixNear(a, b, tol, what, __LINE__)

static const int NL = MdlPosVelEstimator::NUM_LEGS;

// Body to world rotation from roll, pitch and yaw in ZYX order, matching the
// convention MdlOrientationEstimator::rpyFromQuat inverts.
static Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

// A nominal footprint, roughly where a standing Go2 puts its feet relative to
// the body origin, at the given stance height.
static void nominalFootprint(double height, Eigen::Vector3d out[4]) {
  static const double x[4] = {0.19, 0.19, -0.19, -0.19};
  static const double y[4] = {0.14, -0.14, 0.14, -0.14};
  for (int i = 0; i < 4; i++) out[i] = Eigen::Vector3d(x[i], y[i], -height);
}

// ---------------------------------------------------------------------------
// 1. The filter matrices, against the MIT construction
// ---------------------------------------------------------------------------

static void test_kf_matrix_structure() {
  const double dt = 0.002;

  MdlPosVelEstimator est;
  MdlPosVelEstimator::params_t p;
  p.dt = dt;
  est.setParams(p);
  est.reset();

  const int D = MdlPosVelEstimator::DIM;
  const int M = MdlPosVelEstimator::MEAS;
  T_CHECK(D == 18 && M == 28);
  T_CHECK(est.getC().rows() == M && est.getC().cols() == D);

  const Eigen::MatrixXd A = est.getA();
  const Eigen::MatrixXd B = est.getB();
  const Eigen::MatrixXd C = est.getC();

  // A: body position integrates velocity, velocity and feet hold.
  T_MATRIX_NEAR(A.block(0, 0, 3, 3), Eigen::MatrixXd::Identity(3, 3), 1e-15, "A rr");
  T_MATRIX_NEAR(A.block(0, 3, 3, 3), dt * Eigen::MatrixXd::Identity(3, 3), 1e-15,
                "A rv");
  T_MATRIX_NEAR(A.block(3, 3, 3, 3), Eigen::MatrixXd::Identity(3, 3), 1e-15, "A vv");
  T_MATRIX_NEAR(A.block(6, 6, 12, 12), Eigen::MatrixXd::Identity(12, 12), 1e-15,
                "A pp");
  // Total mass of A, which catches any stray entry anywhere else in the matrix.
  T_NEAR(A.cwiseAbs().sum(), 3.0 + 3.0 * dt + 3.0 + 12.0, 1e-15);

  T_MATRIX_NEAR(B.block(3, 0, 3, 3), dt * Eigen::MatrixXd::Identity(3, 3), 1e-15, "B");
  T_NEAR(B.cwiseAbs().sum(), 3.0 * dt, 1e-15);

  // C: twelve body-to-foot rows, twelve velocity rows, four foot height rows.
  Eigen::MatrixXd C1(3, 6), C2(3, 6);
  C1 << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3);
  C2 << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3);
  for (int i = 0; i < 4; i++) {
    T_MATRIX_NEAR(C.block(3 * i, 0, 3, 6), C1, 1e-15, "C position row");
    T_MATRIX_NEAR(C.block(12 + 3 * i, 0, 3, 6), C2, 1e-15, "C velocity row");
    // The four height rows pick out the world z of each foot: states 8, 11,
    // 14 and 17. Written as a loop in the port; this is what checks the loop
    // reproduces MIT's four hand-written indices.
    T_NEAR(C(24 + i, 6 + 3 * i + 2), 1.0, 1e-15);
  }
  T_MATRIX_NEAR(C.block(0, 6, 12, 12), -Eigen::MatrixXd::Identity(12, 12), 1e-15,
                "C foot block");
  // 12 body position + 12 negated foot + 12 velocity + 4 height.
  T_NEAR(C.cwiseAbs().sum(), 40.0, 1e-15);

  // Noise shapes and the initial covariance.
  const Eigen::MatrixXd Q0 = est.getQ0();
  for (int i = 0; i < 3; i++) T_NEAR(Q0(i, i), dt / 20.0, 1e-15);
  for (int i = 3; i < 6; i++) T_NEAR(Q0(i, i), dt * 9.8 / 20.0, 1e-15);
  for (int i = 6; i < 18; i++) T_NEAR(Q0(i, i), dt, 1e-15);
  T_NEAR(Q0.cwiseAbs().sum() - Q0.diagonal().cwiseAbs().sum(), 0.0, 1e-15);

  T_MATRIX_NEAR(Eigen::MatrixXd(est.getR0()), Eigen::MatrixXd::Identity(M, M), 1e-15,
                "R0");
  T_MATRIX_NEAR(Eigen::MatrixXd(est.getCovariance()),
                100.0 * Eigen::MatrixXd::Identity(D, D), 1e-13, "P0");
  T_NEAR(est.getState().cwiseAbs().sum(), 0.0, 1e-15);

  std::cout << "  kf_matrix_structure OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 2. Initial yaw removal
// ---------------------------------------------------------------------------

static void test_orientation_yaw_zeroing() {
  const Eigen::Vector3d w = Eigen::Vector3d::Zero();
  const Eigen::Vector3d g(0.0, 0.0, 9.81);

  {
    MdlOrientationEstimator ori;
    MdlOrientationEstimator::params_t p;
    p.zero_initial_yaw = true;
    ori.setParams(p);
    ori.reset();

    ori.step(quatFromRPY(0.1, -0.2, 1.3), w, g, 0.001);
    T_NEAR(ori.getRPY().x(), 0.1, 1e-12);
    T_NEAR(ori.getRPY().y(), -0.2, 1e-12);
    T_NEAR(ori.getRPY().z(), 0.0, 1e-12);

    // The datum is captured once and held, so a later yaw reads as the change
    // since the first sample rather than being zeroed again.
    ori.step(quatFromRPY(0.1, -0.2, 1.7), w, g, 0.001);
    T_NEAR(ori.getRPY().z(), 0.4, 1e-12);
  }

  {
    // The check that distinguishes left from right multiplication. Correcting
    // about the world vertical leaves roll untouched; correcting about the
    // body's own z axis, which is what right multiplication does, would tilt
    // the correction by the roll and corrupt both roll and pitch. At a large
    // initial yaw the difference is unmissable.
    MdlOrientationEstimator ori;
    MdlOrientationEstimator::params_t p;
    p.zero_initial_yaw = true;
    ori.setParams(p);
    ori.reset();

    ori.step(quatFromRPY(0.3, 0.0, 2.0), w, g, 0.001);
    T_NEAR(ori.getRPY().x(), 0.3, 1e-12);
    T_NEAR(ori.getRPY().y(), 0.0, 1e-12);
    T_NEAR(ori.getRPY().z(), 0.0, 1e-12);

    // Rotation matrix stays a rotation.
    const Eigen::Matrix3d R = ori.getRotation();
    T_MATRIX_NEAR(Eigen::MatrixXd(R.transpose() * R),
                  Eigen::MatrixXd::Identity(3, 3), 1e-13, "R orthonormal");
    T_NEAR(R.determinant(), 1.0, 1e-13);
  }

  {
    // Off, which is the default in simulation, passes yaw straight through.
    MdlOrientationEstimator ori;
    MdlOrientationEstimator::params_t p;
    p.zero_initial_yaw = false;
    ori.setParams(p);
    ori.reset();

    ori.step(quatFromRPY(0.1, -0.2, 1.3), w, g, 0.001);
    T_NEAR(ori.getRPY().z(), 1.3, 1e-12);
  }

  std::cout << "  orientation_yaw_zeroing OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 3. Frame transforms out of the orientation stage
// ---------------------------------------------------------------------------

static void test_orientation_frames() {
  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p;
  p.zero_initial_yaw = false;
  ori.setParams(p);
  ori.reset();

  const Eigen::Quaterniond q = quatFromRPY(0.3, -0.15, 0.7);
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const Eigen::Vector3d gyro(0.11, -0.22, 0.33);

  // Level and at rest: the accelerometer reads gravity's reaction, straight up
  // in both frames, and adding gravity gives zero acceleration.
  {
    MdlOrientationEstimator level;
    level.setParams(p);
    level.reset();
    const Eigen::Vector3d up(0.0, 0.0, 9.81);
    level.step(Eigen::Quaterniond::Identity(), gyro, up, 0.001);
    T_NEAR((level.getAccelerationWorld() - up).norm(), 0.0, 1e-12);
    T_NEAR((level.getAccelerationWorld() + Eigen::Vector3d(0, 0, -9.81)).norm(), 0.0,
           1e-12);
  }

  // The same, rolled. This is the case that fails if the rotation is applied
  // the wrong way round; the level case above passes either way, which is
  // exactly why it cannot be trusted on its own.
  {
    const Eigen::Vector3d accBody = R.transpose() * Eigen::Vector3d(0.0, 0.0, 9.81);
    ori.step(q, gyro, accBody, 0.001);

    T_NEAR((ori.getAccelerationWorld() - Eigen::Vector3d(0.0, 0.0, 9.81)).norm(), 0.0,
           1e-12);
    T_NEAR((ori.getAccelerationWorld() + Eigen::Vector3d(0, 0, -9.81)).norm(), 0.0,
           1e-12);

    T_MATRIX_NEAR(Eigen::MatrixXd(ori.getRotation()), Eigen::MatrixXd(R), 1e-14, "Rbw");
    T_NEAR((ori.getAngularVelocity() - gyro).norm(), 0.0, 1e-15);
    T_NEAR((ori.getAngularVelocityWorld() - R * gyro).norm(), 0.0, 1e-14);
    T_NEAR((ori.getAccelerationWorld() - R * ori.getAcceleration()).norm(), 0.0, 1e-14);

    // rpyFromQuat inverts the construction it was handed.
    T_NEAR(ori.getRPY().x(), 0.3, 1e-12);
    T_NEAR(ori.getRPY().y(), -0.15, 1e-12);
    T_NEAR(ori.getRPY().z(), 0.7, 1e-12);
  }

  std::cout << "  orientation_frames OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 4. A robot standing still
// ---------------------------------------------------------------------------

static void runStatic(MdlPosVelEstimator& est, double height, int steps) {
  Eigen::Vector3d footPos[4], footVel[4];
  nominalFootprint(height, footPos);
  for (int i = 0; i < 4; i++) footVel[i].setZero();

  const double phase[4] = {MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE};
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d aWorld(0.0, 0.0, 9.81);
  const Eigen::Vector3d omega = Eigen::Vector3d::Zero();

  for (int k = 0; k < steps; k++)
    est.step(R, aWorld, omega, footPos, footVel, phase);
}

static void test_static_stance_convergence() {
  const double dt = 0.002;
  const double height = 0.30;

  // Cold start, point foot. Absolute height is observable through the foot
  // height rows, which is new: the filter this replaces could never have
  // recovered it.
  {
    MdlPosVelEstimator est;
    MdlPosVelEstimator::params_t p;
    p.dt = dt;
    p.foot_ground_height = 0.0;
    p.warm_start = false;
    est.setParams(p);
    est.reset();

    runStatic(est, height, 2000);

    Eigen::Vector3d r, v;
    T_CHECK(est.getBodyPosition(r));
    T_CHECK(est.getBodyVelocity(v));
    T_NEAR(r.z(), height, 5e-3);
    T_CHECK(r.head<2>().norm() < 1e-2);
    T_CHECK(v.norm() < 1e-3);

    Eigen::Vector3d footPos[4];
    nominalFootprint(height, footPos);
    for (int i = 0; i < 4; i++) {
      Eigen::Vector3d fp;
      T_CHECK(est.getFootPosition(i, fp));
      T_NEAR(fp.z(), 0.0, 2e-3);
      T_NEAR(fp.x(), footPos[i].x(), 2e-3);
      T_NEAR(fp.y(), footPos[i].y(), 2e-3);
    }
  }

  // With a real foot radius the whole estimate rises by exactly that much,
  // because the datum the height rows pull toward has moved.
  {
    MdlPosVelEstimator est;
    MdlPosVelEstimator::params_t p;
    p.dt = dt;
    p.foot_ground_height = 0.022;
    p.warm_start = false;
    est.setParams(p);
    est.reset();

    runStatic(est, height, 2000);

    Eigen::Vector3d r;
    T_CHECK(est.getBodyPosition(r));
    T_NEAR(r.z(), height + 0.022, 5e-3);

    Eigen::Vector3d fp;
    T_CHECK(est.getFootPosition(0, fp));
    T_NEAR(fp.z(), 0.022, 2e-3);
  }

  // The warm start puts the height where it belongs on the very first cycle,
  // rather than letting a quarter metre lurch into the error statistics.
  {
    MdlPosVelEstimator est;
    MdlPosVelEstimator::params_t p;
    p.dt = dt;
    p.foot_ground_height = 0.0;
    p.warm_start = true;
    est.setParams(p);
    est.reset();

    runStatic(est, height, 1);

    Eigen::Vector3d r;
    T_CHECK(est.getBodyPosition(r));
    // Not exact, because the seed is followed immediately by a full predict and
    // correct on the same cycle; but it is three orders of magnitude closer
    // than the cold start would be after one step.
    T_NEAR(r.z(), height, 1e-3);
  }

  std::cout << "  static_stance_convergence OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 5. A robot travelling at constant velocity
// ---------------------------------------------------------------------------

// Feet planted in the world while the body advances at v, held at a fixed
// attitude. In the body frame each foot therefore slides backwards at exactly
// -v, which is the measurement the velocity rows are built on.
static void runConstantVelocity(MdlPosVelEstimator& est, const Eigen::Vector3d& v,
                                const Eigen::Matrix3d& R, double height, double dt,
                                int steps) {
  Eigen::Vector3d nominal[4];
  nominalFootprint(height, nominal);

  const double phase[4] = {MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE};
  const Eigen::Vector3d aWorld(0.0, 0.0, 9.81);  // constant velocity, so a = 0
  const Eigen::Vector3d omega = Eigen::Vector3d::Zero();

  for (int k = 0; k < steps; k++) {
    const double t = k * dt;
    Eigen::Vector3d footPos[4], footVel[4];
    for (int i = 0; i < 4; i++) {
      // World foot positions are fixed; express them in the moving body frame.
      const Eigen::Vector3d rel = R * nominal[i] - v * t;
      footPos[i] = R.transpose() * rel;
      footVel[i] = R.transpose() * (-v);
    }
    est.step(R, aWorld, omega, footPos, footVel, phase);
  }
}

static void test_constant_velocity() {
  const double dt = 0.002;
  const double height = 0.30;
  const int steps = 2000;
  const Eigen::Vector3d v(0.25, -0.1, 0.0);

  // Level.
  {
    MdlPosVelEstimator est;
    MdlPosVelEstimator::params_t p;
    p.dt = dt;
    p.foot_ground_height = 0.0;
    est.setParams(p);
    est.reset();

    runConstantVelocity(est, v, Eigen::Matrix3d::Identity(), height, dt, steps);

    Eigen::Vector3d r, vw, vb;
    T_CHECK(est.getBodyVelocity(vw));
    T_CHECK(est.getBodyPosition(r));
    T_CHECK(est.getBodyVelocityInBody(vb));
    T_NEAR(vw.x(), v.x(), 5e-3);
    T_NEAR(vw.y(), v.y(), 5e-3);
    T_NEAR(vw.z(), 0.0, 5e-3);
    T_NEAR(r.x(), v.x() * steps * dt, 2e-2);
    T_NEAR(r.y(), v.y() * steps * dt, 2e-2);
    // Height stays pinned by the foot height rows while the horizontal states
    // run free.
    T_NEAR(r.z(), height, 5e-3);
  }

  // Rolled, so the body and world velocities genuinely differ and the frame
  // conversion has something to get wrong.
  {
    const Eigen::Matrix3d R = quatFromRPY(0.2, 0.0, 0.0).toRotationMatrix();

    MdlPosVelEstimator est;
    MdlPosVelEstimator::params_t p;
    p.dt = dt;
    p.foot_ground_height = 0.0;
    est.setParams(p);
    est.reset();

    runConstantVelocity(est, v, R, height, dt, steps);

    Eigen::Vector3d vw, vb;
    T_CHECK(est.getBodyVelocity(vw));
    T_CHECK(est.getBodyVelocityInBody(vb));
    T_NEAR(vw.x(), v.x(), 5e-3);
    T_NEAR(vw.y(), v.y(), 5e-3);
    T_CHECK((vb - vw).norm() > 1e-3);  // the check below is not vacuous
    T_NEAR((vb - R.transpose() * vw).norm(), 0.0, 1e-14);
  }

  // The same journey in reverse. Both the foot positions and the foot
  // velocities negate together, as they physically must, so this is a real
  // trajectory rather than two measurements contradicting each other. It exists
  // because a sign slip in the velocity measurement would make the two rows
  // disagree, and neither direction would then settle where it should.
  {
    MdlPosVelEstimator est;
    MdlPosVelEstimator::params_t p;
    p.dt = dt;
    p.foot_ground_height = 0.0;
    est.setParams(p);
    est.reset();

    runConstantVelocity(est, -v, Eigen::Matrix3d::Identity(), height, dt, steps);

    Eigen::Vector3d r, vw;
    T_CHECK(est.getBodyVelocity(vw));
    T_CHECK(est.getBodyPosition(r));
    T_NEAR(vw.x(), -v.x(), 5e-3);
    T_NEAR(vw.y(), -v.y(), 5e-3);
    T_NEAR(r.x(), -v.x() * steps * dt, 2e-2);
  }

  std::cout << "  constant_velocity OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 6. The angular velocity transport term
// ---------------------------------------------------------------------------

static void test_omega_cross_term() {
  const double dt = 0.002;
  const double height = 0.30;
  const int steps = 2000;
  const Eigen::Vector3d omega(0.0, 0.0, 1.0);  // spinning in place

  MdlPosVelEstimator est;
  MdlPosVelEstimator::params_t p;
  p.dt = dt;
  p.foot_ground_height = 0.0;
  est.setParams(p);
  est.reset();

  Eigen::Vector3d world[4];
  nominalFootprint(height, world);  // feet planted here, in the world

  const double phase[4] = {MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE};

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d dR =
      Eigen::AngleAxisd(omega.z() * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  for (int k = 0; k < steps; k++) {
    Eigen::Vector3d footPos[4], footVel[4];
    for (int i = 0; i < 4; i++) {
      footPos[i] = R.transpose() * world[i];
      // Differentiating p_rel = R^T (P - r) with the body turning in place
      // leaves exactly this. The filter adds omega x p_rel back before rotating
      // to the world, so a planted foot comes out with zero world velocity and
      // the body is correctly reported as not translating.
      footVel[i] = -omega.cross(footPos[i]);
    }
    est.step(R, Eigen::Vector3d(0.0, 0.0, 9.81), omega, footPos, footVel, phase);
    R = R * dR;
  }

  Eigen::Vector3d r, v;
  T_CHECK(est.getBodyPosition(r));
  T_CHECK(est.getBodyVelocity(v));
  // Without the transport term the filter reads the turn as translation and
  // both of these blow up. No other test in this file touches it.
  T_CHECK(v.norm() < 1e-2);
  T_CHECK(r.head<2>().norm() < 1e-2);
  T_NEAR(r.z(), height, 1e-2);

  std::cout << "  omega_cross_term OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 7. The contact trust ramp
// ---------------------------------------------------------------------------

static void test_trust_window_boundaries() {
  const double w = 0.2;

  T_NEAR(MdlPosVelEstimator::trustFromPhase(0.0, w), 0.0, 1e-15);
  T_NEAR(MdlPosVelEstimator::trustFromPhase(0.1, w), 0.5, 1e-15);
  T_NEAR(MdlPosVelEstimator::trustFromPhase(0.2, w), 1.0, 1e-15);
  T_NEAR(MdlPosVelEstimator::trustFromPhase(0.5, w), 1.0, 1e-15);
  // Exactly at the far edge the comparison is strict, so trust is still one.
  T_NEAR(MdlPosVelEstimator::trustFromPhase(0.8, w), 1.0, 1e-15);
  T_NEAR(MdlPosVelEstimator::trustFromPhase(0.9, w), 0.5, 1e-15);
  // The one that surprises people: a phase of one is the instant of liftoff,
  // not the middle of stance, and carries no trust at all. Anything turning a
  // boolean contact into a phase has to know this.
  T_NEAR(MdlPosVelEstimator::trustFromPhase(1.0, w), 0.0, 1e-15);
  T_NEAR(MdlPosVelEstimator::trustFromPhase(1.5, w), 0.0, 1e-15);

  // A zero window means no ramp, not a division by zero. The MIT sources would
  // produce NaN here and poison the covariance.
  const double t0 = MdlPosVelEstimator::trustFromPhase(0.0, 0.0);
  T_CHECK(std::isfinite(t0));
  T_NEAR(t0, 1.0, 1e-15);

  // The property the boolean-to-phase mapping relies on: mid stance is fully
  // trusted whatever the window is set to.
  const double windows[] = {0.05, 0.1, 0.2, 0.3, 0.5};
  for (double win : windows)
    T_NEAR(MdlPosVelEstimator::trustFromPhase(MdlPosVelEstimator::PLANTED_PHASE, win),
           1.0, 1e-15);

  std::cout << "  trust_window_boundaries OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 8. Covariance health over a long run
// ---------------------------------------------------------------------------

// Stance progress of a trotting leg, the same schedule TrotGait produces.
static double trotPhase(int leg, double t, double period, double duty) {
  static const double offset[4] = {0.0, 0.5, 0.5, 0.0};
  double phi = t / period + offset[leg];
  phi -= std::floor(phi);
  if (phi >= duty) return 0.0;
  return phi / duty;
}

static void test_covariance_health() {
  const double dt = 0.002;
  const double height = 0.30;
  const int steps = 10000;  // 20 s

  MdlPosVelEstimator est;
  MdlPosVelEstimator::params_t p;
  p.dt = dt;
  p.foot_ground_height = 0.0;
  est.setParams(p);
  est.reset();

  Eigen::Vector3d footPos[4], footVel[4];
  nominalFootprint(height, footPos);
  for (int i = 0; i < 4; i++) footVel[i].setZero();

  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d aWorld(0.0, 0.0, 9.81);
  const Eigen::Vector3d omega = Eigen::Vector3d::Zero();

  for (int k = 0; k < steps; k++) {
    double phase[4];
    for (int i = 0; i < 4; i++) phase[i] = trotPhase(i, k * dt, 0.5, 0.5);
    est.step(R, aWorld, omega, footPos, footVel, phase);

    if (k % 100 != 0) continue;

    const Eigen::MatrixXd P = est.getCovariance();
    T_CHECK(P.allFinite());
    T_CHECK((P - P.transpose()).cwiseAbs().maxCoeff() < 1e-12);

    // Deliberately not asserting strict positive definiteness. The update uses
    // the plain (I - KC)P form rather than the Joseph one, and the horizontal
    // clamp zeroes cross terms afterwards; neither promises an SPD result in
    // finite precision. A tolerance of zero here would be a flaky test
    // asserting something the algorithm does not claim. If this ever trips, it
    // is a real finding.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
    T_CHECK(es.eigenvalues().minCoeff() > -1e-9);

    // Starts at 1800 from the initial 100*I. Growth past this means divergence.
    T_CHECK(P.trace() < 1e4);
  }

  // The horizontal clamp is doing its job: unobservable though they are, the
  // first two diagonals are held down rather than growing without bound.
  T_CHECK(est.getCovariance()(0, 0) < 100.0);
  T_CHECK(est.getCovariance()(1, 1) < 100.0);

  std::cout << "  covariance_health OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 9. A swinging foot must not drag the body estimate
// ---------------------------------------------------------------------------

static void runWithBadLeg(MdlPosVelEstimator& est, double height, int steps,
                          bool corrupt) {
  Eigen::Vector3d footPos[4], footVel[4];
  nominalFootprint(height, footPos);
  for (int i = 0; i < 4; i++) footVel[i].setZero();

  // Leg 0 is airborne; the other three are solidly planted.
  const double phase[4] = {0.0, MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE,
                           MdlPosVelEstimator::PLANTED_PHASE};

  if (corrupt) {
    footPos[0] += Eigen::Vector3d(1.0, 1.0, 1.0);
    footVel[0] = Eigen::Vector3d(5.0, -5.0, 5.0);
  }

  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d aWorld(0.0, 0.0, 9.81);
  const Eigen::Vector3d omega = Eigen::Vector3d::Zero();

  for (int k = 0; k < steps; k++)
    est.step(R, aWorld, omega, footPos, footVel, phase);
}

static void test_swing_foot_trust_rejection() {
  const double dt = 0.002;
  const double height = 0.30;
  const int steps = 2000;

  MdlPosVelEstimator::params_t p;
  p.dt = dt;
  p.foot_ground_height = 0.0;
  p.warm_start = false;

  MdlPosVelEstimator clean, dirty;
  clean.setParams(p);
  clean.reset();
  dirty.setParams(p);
  dirty.reset();

  runWithBadLeg(clean, height, steps, false);
  runWithBadLeg(dirty, height, steps, true);

  Eigen::Vector3d rc, vc, rd, vd;
  T_CHECK(clean.getBodyPosition(rc) && clean.getBodyVelocity(vc));
  T_CHECK(dirty.getBodyPosition(rd) && dirty.getBodyVelocity(vd));

  // Velocity is untouched to the last bit, and by construction rather than by
  // good fortune: at zero trust the velocity measurement blends all the way to
  // the current estimate, so its residual is identically zero and the foot can
  // say nothing about velocity at all. Asserting this loosely would let a
  // regression turn an exact property into an approximate one unnoticed.
  T_CHECK((vd - vc).norm() < 1e-12);

  // Height likewise: its measurement blends the same way, and whatever leaks
  // past that is de-weighted a hundredfold.
  T_NEAR(rd.z(), rc.z(), 1e-6);

  // Horizontal position is the exception, and it is worth stating plainly
  // rather than choosing a tolerance that hides it. The body-to-foot rows are
  // deliberately left at full weight whatever the trust, because they are what
  // re-anchors a foot as it lands. So a foot reporting a position a metre out
  // does move the body estimate sideways, and because nothing in this filter
  // observes absolute horizontal position, that displacement is never
  // recovered.
  //
  // Measured here at roughly a fifth of the injected error. The bound is loose
  // because the exact split depends on how the covariance has settled by then;
  // what matters is that the foot state absorbs most of it, and that what does
  // leak stays in the plane where it cannot corrupt anything observable.
  const double injected = Eigen::Vector3d(1.0, 1.0, 1.0).head<2>().norm();
  const double leaked = (rd - rc).head<2>().norm();
  T_CHECK(leaked > 1e-3);            // it really does leak; this is not a no-op
  T_CHECK(leaked < 0.4 * injected);  // but most of it lands on the foot state

  // Which is the other half of the same statement: the mechanism is inflation,
  // not exclusion. The airborne foot's own state is free to follow the
  // nonsense, and that freedom is exactly what lets a kinematic measurement
  // re-anchor it the moment it touches down.
  Eigen::Vector3d fc, fd;
  T_CHECK(clean.getFootPosition(0, fc) && dirty.getFootPosition(0, fd));
  T_CHECK((fd - fc).norm() > 0.1);

  std::cout << "  swing_foot_trust_rejection OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 10. The horizontal datum survives a deactivation
// ---------------------------------------------------------------------------

// This module is switched off whenever the robot is off its feet, so a
// stand-trot-sit-stand-trot cycle runs reset() again with the robot no longer
// at the origin. Height and velocity are observable and get re-derived from the
// feet, but nothing here observes absolute x and y, so they have to be carried
// across. Drop them and each cycle reports the previous trot's whole distance
// as fresh position error.
static void test_origin_carry_across_activation() {
  const double dt = 0.002;
  const double height = 0.30;
  const int steps = 2000;
  const Eigen::Vector3d v(0.25, -0.1, 0.0);

  MdlPosVelEstimator est;
  MdlPosVelEstimator::params_t p;
  p.dt = dt;
  p.foot_ground_height = 0.0;
  est.setParams(p);
  est.reset();

  // Trot away from the origin.
  runConstantVelocity(est, v, Eigen::Matrix3d::Identity(), height, dt, steps);

  Eigen::Vector3d before;
  T_CHECK(est.getBodyPosition(before));
  // Far enough that carrying the datum and dropping it are unmistakably
  // different outcomes, so this cannot pass by tolerance.
  T_CHECK(before.head<2>().norm() > 0.1);

  // Sit, then stand: the Supervisor deactivates the module and activates it
  // again, and activate() resets. The accessors gate on readiness, so read the
  // raw state here.
  est.deactivate();
  est.reset();
  T_CHECK(!est.isReady());
  T_NEAR(est.getState()(0), before.x(), 1e-12);
  T_NEAR(est.getState()(1), before.y(), 1e-12);

  // Height and velocity are deliberately not carried. They are measured, so
  // assuming them would be guessing where the filter can simply look.
  T_NEAR(est.getState()(2), 0.0, 1e-12);
  T_NEAR(est.getState().segment<3>(3).norm(), 0.0, 1e-12);

  // Standing up takes a while to load the feet, so the warm start does not fire
  // on the first cycle. Reproduce that: no foot trusted, which leaves the
  // footholds at zero while r holds the datum, so rows 0-11 are inconsistent
  // and the filter drags r toward the footholds. This window is what makes the
  // seed read _originCarry rather than the current estimate -- without those
  // untrusted cycles the bug is invisible, because the seed fires before r has
  // had a chance to move.
  {
    Eigen::Vector3d footPos[4], footVel[4];
    nominalFootprint(height, footPos);
    const double swing[4] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 4; i++) footVel[i].setZero();
    for (int k = 0; k < 200; k++)
      est.step(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 9.81),
               Eigen::Vector3d::Zero(), footPos, footVel, swing);

    // The drag is real, so the assertions below are not trivially satisfied.
    T_CHECK(std::fabs(est.getState()(0) - before.x()) > 1e-3);
  }

  // Now the feet load and the seed fires, which must restore the datum rather
  // than preserve what the drag left behind.
  runStatic(est, height, 500);

  Eigen::Vector3d after, vw;
  T_CHECK(est.getBodyPosition(after));
  T_CHECK(est.getBodyVelocity(vw));
  T_NEAR(after.x(), before.x(), 1e-3);
  T_NEAR(after.y(), before.y(), 1e-3);
  T_NEAR(after.z(), height, 5e-3);
  T_NEAR(vw.norm(), 0.0, 5e-3);

  std::cout << "  origin_carry_across_activation OK" << std::endl;
}

// ---------------------------------------------------------------------------

int main() {
  std::cout << "test_state_estimator" << std::endl;

  test_kf_matrix_structure();
  test_orientation_yaw_zeroing();
  test_orientation_frames();
  test_static_stance_convergence();
  test_constant_velocity();
  test_omega_cross_term();
  test_trust_window_boundaries();
  test_covariance_health();
  test_swing_foot_trust_rejection();
  test_origin_carry_across_activation();

  std::cout << "test_state_estimator PASSED" << std::endl;
  return 0;
}
