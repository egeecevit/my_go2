/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// Tests for the Mahony attitude filter inside MdlOrientationEstimator, the path
// taken when attitude_source is "filter".
//
// Separate from test_state_estimator.cc, which exercises the passthrough path
// and the frame conventions on top of it. Nothing there ever turns the filter
// on, so until now its entire behaviour was established by trotting the
// simulator and reading a report line -- a measurement that takes thirty
// seconds, needs a gait, and cannot distinguish the filter being wrong from the
// gait being wrong.
//
// Every case here is a stationary or uniformly drifting body with an analytic
// answer, so the assertions are on numbers derived on paper rather than on
// whatever the code happened to produce when it was written:
//
//   1. an initial attitude error is driven out           (correction sign)
//   2. a constant gyro bias is identified                (the integral term)
//   3. with ki = 0 it is not, and the standing tilt      (equation 19 as printed)
//      is exactly bias/kp
//   4. accel_filter_tau = 0 is a passthrough, not a division by zero
//   5. an oscillating acceleration is rejected           (why the LPF is there)
//   6. yaw drifts at the injected rate and the z bias    (what gravity cannot
//      is never identified                                observe)
//   7. the integrator stops at mahony_bias_limit         (anti-windup)
//
// The module is driven through step(), which needs no ModuleManager, no config
// and no hardware. Note that step() seeds the estimate from its quaternion
// argument on the first call only; afterwards that argument is ignored on this
// path, so each test passes the attitude it wants to start wrong at and then
// feeds gyro and accelerometer readings consistent with the truth.

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hardware/MotorHW.hh"
#include "quadruped/MdlOrientationEstimator.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

// Linking pulls in the whole MdlOrientationEstimator translation unit, whose
// module layer references the hardware singletons. Defining their statics
// satisfies the linker; none is ever instantiated here, so instance() returns
// null and update() is never reached anyway.
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

static const double DT = 0.001;
static const double G = 9.81;

// Body to world rotation from roll, pitch and yaw in ZYX order, matching the
// convention MdlOrientationEstimator::rpyFromQuat inverts.
static Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

// Parameters common to every case: the filter on, the synthetic corruption off,
// since each test injects exactly the imperfection it is about.
static MdlOrientationEstimator::params_t baseParams(double kp, double ki) {
  MdlOrientationEstimator::params_t p;
  p.filter_attitude = true;
  p.noise_enable = false;
  p.zero_initial_yaw = false;
  p.mahony_kp = kp;
  p.mahony_ki = ki;
  return p;
}

// A stationary body: the readings never change, so the whole trajectory is one
// quaternion and one accelerometer vector repeated.
static void runStationary(MdlOrientationEstimator& ori, const Eigen::Quaterniond& q0,
                          const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc,
                          int steps) {
  for (int k = 0; k < steps; k++) ori.step(q0, gyro, acc, DT);
}

// ---------------------------------------------------------------------------
// 1. The correction drives an initial attitude error out
// ---------------------------------------------------------------------------

// Deliberately started at identity against a truth that is neither level nor
// symmetric in its two axes, so a correction with the wrong sign diverges and a
// correction with roll and pitch transposed lands on the wrong answer. Both
// would be invisible from a level start.
static void convergeFromIdentity(double tau, const char* what) {
  const double roll = 0.1, pitch = -0.05;
  const Eigen::Matrix3d Rtrue = quatFromRPY(roll, pitch, 0.0).toRotationMatrix();
  const Eigen::Vector3d acc = Rtrue.transpose() * Eigen::Vector3d(0.0, 0.0, G);

  // ki = 0 here on purpose. This case is about the proportional correction
  // alone, and with no bias to find the integral term only adds an eleven
  // second transient to a test that has nothing to say about it.
  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(2.0, 0.0);
  p.accel_filter_tau = tau;
  ori.setParams(p);
  ori.reset();

  runStationary(ori, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), acc,
                10000);  // 10 s, twenty time constants at kp = 2

  const Eigen::Vector3d rpy = ori.getRPY();
  T_CHECK(rpy.allFinite());
  std::cout << "    " << what << ": roll " << rpy.x() << " pitch " << rpy.y()
            << std::endl;
  T_NEAR(rpy.x(), roll, 2e-3);
  T_NEAR(rpy.y(), pitch, 2e-3);
  // Yaw ends near where it was seeded but NOT exactly there, and the difference
  // is the point rather than a tolerance. wcorr is perpendicular to the measured
  // gravity direction, not to body z, so while the estimate is tilted away from
  // truth the correction has a small component along the body vertical and
  // leaks a few milliradians into heading. It is bounded by the tilt being
  // driven out, and it is never recovered, because nothing here observes yaw.
  T_CHECK(std::fabs(rpy.z()) < 5e-3);
}

static void test_correction_convergence() {
  convergeFromIdentity(0.5, "tau = 0.5");
  std::cout << "  correction_convergence OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 2. A constant gyro bias is identified
// ---------------------------------------------------------------------------

// The whole reason mahony_ki exists. The body is level and still; the gyroscope
// claims it is turning. The integrator has to end up holding exactly that lie,
// because it is subtracted before the rate is passed to the position filter.
static void test_bias_learning() {
  const double bias = 0.002;
  const Eigen::Vector3d gyro(bias, bias, 0.0);
  const Eigen::Vector3d acc(0.0, 0.0, G);

  MdlOrientationEstimator ori;
  // zeta = kp / (2 sqrt(ki)) = 1.0, wn = 0.3 rad/s: critically damped, settled
  // to well under a percent by 60 s.
  ori.setParams(baseParams(0.6, 0.09));
  ori.reset();

  runStationary(ori, Eigen::Quaterniond::Identity(), gyro, acc, 60000);

  const Eigen::Vector3d b = ori.getGyroBias();
  const Eigen::Vector3d rpy = ori.getRPY();
  std::cout << "    bias " << b.transpose() << " rpy " << rpy.transpose()
            << std::endl;

  T_NEAR(b.x(), bias, 0.1 * bias);
  T_NEAR(b.y(), bias, 0.1 * bias);
  // With the bias found the attitude has nowhere left to settle but the truth.
  T_NEAR(rpy.x(), 0.0, 2e-3);
  T_NEAR(rpy.y(), 0.0, 2e-3);

  // And the point of finding it: the rate handed downstream is the true one,
  // not the sensor's.
  T_NEAR(ori.getAngularVelocity().x(), 0.0, 0.1 * bias);
  T_NEAR(ori.getAngularVelocity().y(), 0.0, 0.1 * bias);

  std::cout << "  bias_learning OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 3. Equation (19) as printed leaves a standing tilt
// ---------------------------------------------------------------------------

// The analytic reason mahony_ki was added, pinned so it cannot quietly change.
// With no integral term the loop settles where the proportional correction
// cancels the bias: w = b + kp * wcorr = 0. For a level body wcorr is, to first
// order, minus the attitude error in roll and pitch, so the error settles at
// exactly +b/kp -- 0.02 rad here, twenty times the milliradian the full filter
// achieves on the same input in the case above.
static void test_ki_zero_standing_error() {
  const double bias = 0.002, kp = 0.1;
  const Eigen::Vector3d gyro(bias, bias, 0.0);
  const Eigen::Vector3d acc(0.0, 0.0, G);

  MdlOrientationEstimator ori;
  ori.setParams(baseParams(kp, 0.0));
  ori.reset();

  // Ten time constants at 1/kp = 10 s.
  runStationary(ori, Eigen::Quaterniond::Identity(), gyro, acc, 100000);

  const Eigen::Vector3d rpy = ori.getRPY();
  const double expected = bias / kp;
  std::cout << "    tilt " << rpy.x() << ", " << rpy.y() << " expected "
            << expected << std::endl;

  T_NEAR(rpy.x(), expected, 0.2 * expected);
  T_NEAR(rpy.y(), expected, 0.2 * expected);

  // Nothing integrates, so the bias is not merely unconverged, it is untouched.
  T_NEAR(ori.getGyroBias().norm(), 0.0, 0.0);

  std::cout << "  ki_zero_standing_error OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 4. accel_filter_tau = 0
// ---------------------------------------------------------------------------

// A legitimate setting rather than a degenerate one: the coefficient becomes
// dt/(0 + dt) = 1, which makes the low pass a passthrough and recovers the
// unfiltered gravity reference of the paper. It is worth a test only because it
// is the one value that could have been a division by zero, and because a NaN
// here propagates into the rotation matrix and takes the position filter with
// it.
static void test_zero_tau_passthrough() {
  convergeFromIdentity(0.0, "tau = 0");
  std::cout << "  zero_tau_passthrough OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 5. An oscillating acceleration is rejected
// ---------------------------------------------------------------------------

// The case the low pass exists for, in miniature. A trotting trunk surges fore
// and aft at twice the gait frequency; here that is a 2 Hz, 2 m/s^2 sinusoid on
// body x, which tilts the raw gravity reference by 2/9.81 = 0.20 rad peak while
// the body does not move at all. A first order low pass at tau = 0.5 s attenuates
// 2 Hz by 1/sqrt(1 + (2 pi 2 tau)^2), about six fold, and the attitude loop at
// kp = 0.6 attenuates what survives by a further kp/omega. What must not happen
// is any of it rectifying into a mean tilt, because a mean tilt rotates gravity
// into the horizontal plane of the position filter's prediction and integrates.
static void test_dynamic_acceleration_rejection() {
  const double amp = 2.0, freq = 2.0;

  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(0.6, 0.09);
  p.accel_filter_tau = 0.5;
  ori.setParams(p);
  ori.reset();

  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();

  const int steps = 30000;      // 30 s
  const int settled = 25000;    // average over the last 5 s, ten full periods
  double sumPitch = 0.0, sumRoll = 0.0, peakPitch = 0.0;
  int n = 0;

  for (int k = 0; k < steps; k++) {
    const double t = k * DT;
    Eigen::Vector3d acc(amp * std::sin(2.0 * M_PI * freq * t), 0.0, G);
    ori.step(q0, gyro, acc, DT);
    if (k < settled) continue;
    sumPitch += ori.getRPY().y();
    sumRoll += ori.getRPY().x();
    peakPitch = std::max(peakPitch, std::fabs(ori.getRPY().y()));
    n++;
  }

  const double meanPitch = sumPitch / n, meanRoll = sumRoll / n;
  std::cout << "    mean pitch " << meanPitch << " mean roll " << meanRoll
            << " peak pitch " << peakPitch << std::endl;

  T_CHECK(ori.getRPY().allFinite());
  T_NEAR(meanPitch, 0.0, 2e-3);
  // Nothing excites the other axis at all, so this is a check that the
  // disturbance is not being smeared across axes by the cross product.
  T_NEAR(meanRoll, 0.0, 1e-6);
  // The oscillation is not vacuously small: it is what is left of a 0.20 rad
  // reference tilt after the low pass and the loop. Assert it is both present
  // and two orders below the raw disturbance.
  T_CHECK(peakPitch > 1e-4);
  T_CHECK(peakPitch < 5e-3);
  // The bias estimate must not have absorbed a zero mean disturbance.
  T_CHECK(ori.getGyroBias().norm() < 1e-4);

  std::cout << "  dynamic_acceleration_rejection OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 6. Yaw is unobservable
// ---------------------------------------------------------------------------

// Not a defect, and the reason it is not is worth pinning: wcorr is a cross
// product with the measured gravity direction, so for a level body it has no z
// component at all and the yaw bias gets no excitation. The heading therefore
// integrates the bias exactly, one for one, which is the 110 mrad the config
// file's sweep table reports over a 55 s run and accepts. Roll and pitch are
// untouched by it.
static void test_yaw_unobservability() {
  const double bias = 0.002;
  const Eigen::Vector3d gyro(0.0, 0.0, bias);
  const Eigen::Vector3d acc(0.0, 0.0, G);

  MdlOrientationEstimator ori;
  ori.setParams(baseParams(0.6, 0.09));
  ori.reset();

  const int steps = 60000;
  runStationary(ori, Eigen::Quaterniond::Identity(), gyro, acc, steps);

  const double T = steps * DT;
  const Eigen::Vector3d rpy = ori.getRPY();
  const Eigen::Vector3d b = ori.getGyroBias();
  std::cout << "    yaw " << rpy.z() << " expected " << bias * T << " bias z "
            << b.z() << std::endl;

  T_NEAR(rpy.z(), bias * T, 0.1 * bias * T);
  // Never identified, so never subtracted: the drift above is permanent.
  T_CHECK(std::fabs(b.z()) < 0.2 * bias);
  // Gravity still holds the two axes it does see.
  T_NEAR(rpy.x(), 0.0, 2e-3);
  T_NEAR(rpy.y(), 0.0, 2e-3);

  std::cout << "  yaw_unobservability OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 7. The integrator clamp
// ---------------------------------------------------------------------------

// mahony_bias_limit bounds what a sustained tilt of the gravity reference can
// do to the integrator. Tested by handing the filter a bias larger than the
// limit, which is the same situation from the integrator's point of view: it is
// fed a consistent error it cannot cancel. It must stop at the limit rather
// than continue, and the attitude must then settle at the standing tilt the
// uncancelled remainder implies, exactly as in the ki = 0 case above.
static void test_bias_clamp() {
  const double injected = 0.01, limit = 0.001, kp = 0.6;
  const Eigen::Vector3d gyro(injected, 0.0, 0.0);
  const Eigen::Vector3d acc(0.0, 0.0, G);

  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(kp, 0.09);
  p.mahony_bias_limit = limit;
  ori.setParams(p);
  ori.reset();

  runStationary(ori, Eigen::Quaterniond::Identity(), gyro, acc, 30000);

  const Eigen::Vector3d b = ori.getGyroBias();
  std::cout << "    clamped bias " << b.x() << " limit " << limit << " roll "
            << ori.getRPY().x() << std::endl;

  // Pinned at the limit, not merely below it: the drive is still pushing.
  T_NEAR(b.x(), limit, 1e-12);
  T_CHECK(b.x() <= limit);

  // Unclamped it would have reached the injected value, which is what makes the
  // assertion above non-vacuous.
  {
    MdlOrientationEstimator open;
    MdlOrientationEstimator::params_t q = baseParams(kp, 0.09);
    q.mahony_bias_limit = 0.0;  // no clamp
    open.setParams(q);
    open.reset();
    runStationary(open, Eigen::Quaterniond::Identity(), gyro, acc, 30000);
    std::cout << "    unclamped bias " << open.getGyroBias().x() << std::endl;
    T_NEAR(open.getGyroBias().x(), injected, 0.1 * injected);
  }

  // What the clamp costs: the remainder is left to the proportional term, so
  // the tilt settles at (injected - limit)/kp, the equation (19) behaviour.
  T_NEAR(ori.getRPY().x(), (injected - limit) / kp, 0.2 * (injected - limit) / kp);

  std::cout << "  bias_clamp OK" << std::endl;
}

// ---------------------------------------------------------------------------

int main() {
  std::cout << "test_mahony_filter" << std::endl;

  test_correction_convergence();
  test_bias_learning();
  test_ki_zero_standing_error();
  test_zero_tau_passthrough();
  test_dynamic_acceleration_rejection();
  test_yaw_unobservability();
  test_bias_clamp();

  std::cout << "test_mahony_filter PASSED" << std::endl;
  return 0;
}
