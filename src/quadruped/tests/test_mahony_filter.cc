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
  // Keep this analytic clamp test independent of rotating-reference dynamics.
  // With a nonzero tau, the deliberately uncorrected residual rate also
  // transports the stored gravity vector and adds residual * tau to the
  // standing tilt.  The clamp itself is what this case is intended to pin.
  p.accel_filter_tau = 0.0;
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
    q.accel_filter_tau = 0.0;
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
// 8. Shadow mode publishes truth while the filter keeps running
// ---------------------------------------------------------------------------

// use_ground_truth is a diagnostic contract for the consumers, not a bypass of
// the estimator: the accessors must reproduce every published simulator
// quaternion and body rate exactly, while the filter behind them keeps running
// on the IMU, so a shadow run measures the estimator the controller is not
// using. Otherwise "estimator error with the controller on truth" could never
// be measured at all.
static void test_shadow_publishes_truth_while_filter_runs() {
  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(0.6, 0.09);
  p.use_ground_truth = true;
  ori.setParams(p);
  ori.reset();

  // Filter seeded level against a tilted truth, so it has somewhere to go.
  const double roll = 0.1, pitch = -0.05;
  const Eigen::Quaterniond truth = quatFromRPY(roll, pitch, 0.0);
  const Eigen::Vector3d acc = truth.conjugate() * Eigen::Vector3d(0.0, 0.0, G);
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omegaTrue(0.01, -0.02, 0.03);
  const Eigen::Vector3d aWorldTrue = truth * acc;

  for (int k = 0; k < 20000; ++k) {
    ori.step(Eigen::Quaterniond::Identity(), gyro, acc, DT);
    ori.setPublishedTruth(truth, omegaTrue, acc);
    T_CHECK(ori.getOrientation().angularDistance(truth) < 1e-12);
    T_CHECK((ori.getAngularVelocity() - omegaTrue).norm() < 1e-12);
    T_CHECK((ori.getAccelerationWorld() - aWorldTrue).norm() < 1e-9);
  }

  // The filter ran: from an identity seed its roll and pitch have converged
  // onto the truth. (Not the full angular distance: two successive tilt
  // corrections leave a small yaw residual that gravity cannot observe.)
  const Eigen::Vector3d rpy = MdlOrientationEstimator::rpyFromQuat(ori.getFilterOrientation());
  T_NEAR(rpy.x(), roll, 2e-3);
  T_NEAR(rpy.y(), pitch, 2e-3);
  std::cout << "  shadow_publishes_truth_while_filter_runs OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 8b. A planted attitude error decays as the second-order loop predicts
// ---------------------------------------------------------------------------

// perturbAttitude() is the convergence probe behind tilt_probe_deg. For a
// stationary body with no gyro bias, a planted roll error e, the bias
// integrator b and the gravity reference's own error r obey, to first order,
//
//     e' = -kp (e - r) - b,     b' = ki (e - r),     r' = -r/tau - b.
//
// The third equation is the part a textbook Mahony analysis lacks: the filter
// transports its low-passed gravity reference with the bias-corrected rate, so
// while b is wrong the reference is dragged along at -b and the low-pass pulls
// it back at 1/tau. It makes the loop third-order. For kp = 1, ki = 0.25,
// tau = 0.5 that is 1/e at 0.84 s, zero at 1.76 s and an undershoot of
// -0.235 theta0 at 4.0 s, against -0.135 theta0 for the second-order loop
// alone. The test integrates the model rather than quoting those numbers, so
// it keeps pinning the loop if the gains change. The bias clamp is disabled:
// the integrator peaks near 0.19 theta0, which an 8 degree probe pushes past
// the 0.02 rad/s limit.
static void test_tilt_probe_convergence() {
  const double kp = 1.0, ki = 0.25, tau = 0.5;
  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(kp, ki);
  p.accel_filter_tau = tau;
  p.mahony_bias_limit = 0.0;
  ori.setParams(p);
  ori.reset();

  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc(0.0, 0.0, G);
  runStationary(ori, Eigen::Quaterniond::Identity(), gyro, acc, 2000);

  const double theta0 = 8.0 * M_PI / 180.0;
  ori.perturbAttitude(Eigen::Vector3d(theta0, 0.0, 0.0));

  double e = theta0, b = 0.0, r = 0.0;
  double tZero = -1.0, undershoot = 0.0, tUnder = 0.0, worst = 0.0;
  for (int k = 1; k <= 15000; ++k) {
    const double before = k == 1 ? theta0 : ori.getRPY().x();
    ori.step(Eigen::Quaterniond::Identity(), gyro, acc, DT);
    const double de = -kp * (e - r) - b, db = ki * (e - r), dr = -r / tau - b;
    e += de * DT;
    b += db * DT;
    r += dr * DT;

    const double t = k * DT;
    const double roll = ori.getRPY().x();
    // The published attitude refreshes on step(); one step in, the probe is intact.
    if (k == 1) T_NEAR(roll, theta0, 1e-3 * theta0);
    if (tZero < 0.0 && before > 0.0 && roll <= 0.0) tZero = t;
    if (roll < undershoot) {
      undershoot = roll;
      tUnder = t;
    }
    worst = std::max(worst, std::fabs(roll - e));
  }
  std::cout << "    zero crossing " << tZero << " s, undershoot " << undershoot / theta0
            << " theta0 at " << tUnder << " s, max |roll - model| " << worst / theta0
            << " theta0" << std::endl;
  T_CHECK(worst < 0.01 * theta0);
  T_NEAR(tZero, 1.76, 0.1);
  T_NEAR(undershoot / theta0, -0.235, 0.02);
  T_CHECK(std::fabs(ori.getRPY().x()) < 0.01 * theta0);
  T_CHECK(std::fabs(ori.getRPY().y()) < 1e-6);  // the error stays on its own axis
  std::cout << "  tilt_probe_convergence OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 8c. Bias learning is frozen while the body moves
// ---------------------------------------------------------------------------

// A trotting body pitches at a few tenths of a radian per second and, at a
// gait transition, accelerates fore-aft at ~0.3 m/s^2 for a second or two.
// The acceleration tilts the low-passed gravity reference by a/g without
// changing its magnitude, so the gravity band lets it through and the
// integrator reads it as bias: 0.25 * 0.03 rad * 1.5 s = 11 mrad/s on paper,
// 7 mrad/s measured in simulation. With the body-rate gate the integrator must
// not move at all, while the proportional path still tracks.
static void test_bias_gate_during_motion() {
  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(1.0, 0.25);
  p.accel_filter_tau = 0.5;
  ori.setParams(p);
  ori.reset();

  const double f = 3.33, amp = 0.03;
  double maxBias = 0.0, maxPitchErr = 0.0;
  for (int k = 0; k < 6000; ++k) {
    const double t = k * DT;
    const double pitch = amp * std::sin(2.0 * M_PI * f * t);
    const double rate = amp * 2.0 * M_PI * f * std::cos(2.0 * M_PI * f * t);
    const Eigen::Quaterniond truth = quatFromRPY(0.0, pitch, 0.0);
    const double ax = (t >= 2.0 && t < 3.5) ? 0.3 : 0.0;
    const Eigen::Vector3d acc = truth.conjugate() * Eigen::Vector3d(ax, 0.0, G);
    ori.step(truth, Eigen::Vector3d(0.0, rate, 0.0), acc, DT);
    maxBias = std::max(maxBias, ori.getGyroBias().norm());
    if (t > 1.0) maxPitchErr = std::max(maxPitchErr, std::fabs(ori.getRPY().y() - pitch));
  }
  std::cout << "    max |bias| " << maxBias << " rad/s, max pitch error " << maxPitchErr << " rad"
            << std::endl;
  // ki/20 while moving: the 7 mrad/s kick of the ungated loop becomes < 1 mrad/s.
  T_CHECK(maxBias < 1e-3);
  // The proportional path still follows the reference's a/g tilt: bounded, not divergent.
  T_CHECK(maxPitchErr < 0.04);
  std::cout << "  bias_gate_during_motion OK" << std::endl;
}

// The gate must reopen: after the motion stops and the hold-off has elapsed,
// a real bias is learned as before. During motion only the slow ki/20 path
// runs, so less than a tenth of the bias is learned in the first 3 s, which
// also pins that the gate never opens at the rate's zero crossings.
static void test_bias_gate_reopens_after_quiet() {
  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(1.0, 0.25);
  p.accel_filter_tau = 0.5;
  ori.setParams(p);
  ori.reset();

  const Eigen::Vector3d bias(0.002, 0.002, 0.0);
  const double f = 3.33, amp = 0.03;
  for (int k = 0; k < 25000; ++k) {
    const double t = k * DT;
    const double pitch = t < 2.0 ? amp * std::sin(2.0 * M_PI * f * t) : 0.0;
    const double rate = t < 2.0 ? amp * 2.0 * M_PI * f * std::cos(2.0 * M_PI * f * t) : 0.0;
    const Eigen::Quaterniond truth = quatFromRPY(0.0, pitch, 0.0);
    const Eigen::Vector3d acc = truth.conjugate() * Eigen::Vector3d(0.0, 0.0, G);
    ori.step(truth, Eigen::Vector3d(0.0, rate, 0.0) + bias, acc, DT);
    if (t < 2.95) T_CHECK(ori.getGyroBias().norm() < 0.1 * bias.norm());
    if (k == 3500) T_CHECK(ori.getGyroBias().norm() > 1e-5);  // reopened after 2 tau
  }
  T_NEAR(ori.getGyroBias().x(), bias.x(), 0.1 * bias.x());
  T_NEAR(ori.getGyroBias().y(), bias.y(), 0.1 * bias.y());
  std::cout << "  bias_gate_reopens_after_quiet OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 8d. Centripetal acceleration is removed from the gravity reference
// ---------------------------------------------------------------------------

// A body turning at omega_z while moving at v_x reads omega x v = 0.09 m/s^2
// of lateral specific force, which gravity alone cannot tell from a 0.53 deg
// roll; with ki > 0 the filter settles exactly there (trot_yaw: 0.70 deg tilt
// rms against 0.31 straight). With the velocity hint the term is subtracted
// before the reference low-pass and the roll error vanishes.
static void test_centripetal_compensation() {
  const double wz = 0.3, vx = 0.3;
  const Eigen::Vector3d omega(0.0, 0.0, wz), vBody(vx, 0.0, 0.0);
  // Level body yawing: body-frame specific force = g up + omega x v.
  const Eigen::Vector3d acc = Eigen::Vector3d(0.0, 0.0, G) + omega.cross(vBody);

  double rollWithout = 0.0, rollWith = 0.0;
  for (int pass = 0; pass < 2; ++pass) {
    MdlOrientationEstimator ori;
    ori.setParams(baseParams(1.0, 0.25));
    ori.reset();
    if (pass == 1) ori.setBodyVelocityHint(vBody);
    for (int k = 0; k < 20000; ++k) ori.step(Eigen::Quaterniond::Identity(), omega, acc, DT);
    (pass == 0 ? rollWithout : rollWith) = ori.getRPY().x();
  }
  std::cout << "    roll error without hint " << rollWithout << " rad, with hint " << rollWith
            << std::endl;
  // The uncompensated filter leans by about omega v / g (0.53 deg); the exact
  // steady value differs by the small-angle and transport terms, ~0.04 deg.
  T_NEAR(rollWithout, std::atan2(wz * vx, G), 1e-3);
  T_CHECK(std::fabs(rollWith) < 1e-4);
  std::cout << "  centripetal_compensation OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 9. Low-pass history must follow the rotating body frame
// ---------------------------------------------------------------------------

// A gravity vector stored at time k - 1 is expressed in B(k - 1), while the
// next accelerometer reading is expressed in B(k).  They cannot be blended as
// component triples when the body is rotating: doing so turns a true pitch
// motion into a delayed gravity vector, then into a fictitious y-axis gyro
// bias.  This is deliberately an ideal IMU case -- exact angular rate and
// exact gravity, with no translational acceleration -- so neither source of
// error can explain a nonzero correction.
//
// At 1 rad/s for 1.5 s the body pitches through 86 degrees.  That is much more
// attitude motion than a nominal gait, but makes the coordinate-frame error
// unambiguous and keeps the test short.  A frame-consistent filter transports
// the previous average from B(k - 1) to B(k) before blending it, therefore it
// follows this trajectory to numerical integration accuracy and learns no
// bias.  The old direct body-component average has about a half-second stale
// gravity direction and fails both checks by a wide margin.
static void test_rotating_frame_accel_filter_invariance() {
  const double pitchRate = 1.0;
  const int steps = 1500;

  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(0.6, 0.09);
  p.accel_filter_tau = 0.5;
  ori.setParams(p);
  ori.reset();

  for (int k = 0; k <= steps; ++k) {
    const double pitch = pitchRate * k * DT;
    const Eigen::Quaterniond truth = quatFromRPY(0.0, pitch, 0.0);
    const Eigen::Vector3d acc =
        truth.conjugate() * Eigen::Vector3d(0.0, 0.0, G);
    ori.step(truth, Eigen::Vector3d(0.0, pitchRate, 0.0), acc, DT);
  }

  const double truePitch = pitchRate * steps * DT;
  const Eigen::Vector3d rpy = ori.getRPY();
  const Eigen::Vector3d bias = ori.getGyroBias();
  std::cout << "    pitch " << rpy.y() << " true " << truePitch << " bias y "
            << bias.y() << std::endl;

  T_NEAR(rpy.y(), truePitch, 3e-3);
  T_NEAR(bias.y(), 0.0, 1e-4);
  std::cout << "  rotating_frame_accel_filter_invariance OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 10. A dynamic first accelerometer sample must not become the gravity history
// ---------------------------------------------------------------------------

// Filter mode explicitly trusts the first IMU quaternion as its attitude seed.
// The matching gravity vector is therefore already known.  Seeding the long
// acceleration low pass from the simultaneous raw sample instead lets a startup
// shove or settling transient masquerade as gravity for roughly tau seconds,
// corrupting both attitude and the much slower bias integrator long after the
// transient itself is gone.
static void test_dynamic_first_accel_does_not_poison_startup() {
  const Eigen::Quaterniond truth = quatFromRPY(0.0, -0.1, 0.0);
  const Eigen::Vector3d gravityBody =
      truth.conjugate() * Eigen::Vector3d(0.0, 0.0, G);

  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(0.6, 0.09);
  p.accel_filter_tau = 0.5;
  ori.setParams(p);
  ori.reset();

  // A horizontal 0.6 g transient is plausible while the simulated model drops
  // into its initial contacts.  It occurs only in the seed sample.
  ori.step(truth, Eigen::Vector3d::Zero(),
           gravityBody + Eigen::Vector3d(6.0, 0.0, 0.0), DT);

  double peakPitchError = 0.0;
  for (int k = 0; k < 2000; ++k) {
    ori.step(truth, Eigen::Vector3d::Zero(), gravityBody, DT);
    peakPitchError = std::max(
        peakPitchError, std::fabs(ori.getRPY().y() + 0.1));
  }

  std::cout << "    startup peak pitch error " << peakPitchError
            << " bias " << ori.getGyroBias().transpose() << std::endl;
  T_CHECK(peakPitchError < 1e-3);
  T_CHECK(ori.getGyroBias().norm() < 1e-4);
  std::cout << "  dynamic_first_accel_does_not_poison_startup OK" << std::endl;
}

// ---------------------------------------------------------------------------
// 11. Initial yaw is a datum, not a recursive correction
// ---------------------------------------------------------------------------

// zero_initial_yaw is an output-frame convention.  In filter mode the
// propagated state must remain in the IMU's original world frame and the fixed
// datum rotation must be applied only when publishing it.  Reapplying the same
// half-radian inverse yaw to the recursive quaternion every sample creates a
// spurious -0.5 rad heading step on every call while the robot is completely
// still.
static void test_zero_initial_yaw_is_applied_once() {
  const Eigen::Quaterniond initial = quatFromRPY(0.0, 0.0, 0.5);
  const Eigen::Vector3d acc(0.0, 0.0, G);

  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(0.6, 0.09);
  p.zero_initial_yaw = true;
  ori.setParams(p);
  ori.reset();

  for (int k = 0; k < 10; ++k)
    ori.step(initial, Eigen::Vector3d::Zero(), acc, DT);

  const Eigen::Vector3d rpy = ori.getRPY();
  std::cout << "    stationary relative yaw " << rpy.z() << std::endl;
  T_NEAR(rpy.x(), 0.0, 1e-10);
  T_NEAR(rpy.y(), 0.0, 1e-10);
  T_NEAR(rpy.z(), 0.0, 1e-10);
  std::cout << "  zero_initial_yaw_is_applied_once OK" << std::endl;
}

// The same datum must retain subsequent relative heading rather than pinning
// yaw to zero or accumulating the datum on every propagation step.
static void test_zero_initial_yaw_reports_relative_heading() {
  const double initialYaw = 0.5;
  const double yawRate = 0.2;
  const int steps = 1000;
  const Eigen::Vector3d acc(0.0, 0.0, G);

  MdlOrientationEstimator ori;
  MdlOrientationEstimator::params_t p = baseParams(0.6, 0.09);
  p.zero_initial_yaw = true;
  ori.setParams(p);
  ori.reset();

  const Eigen::Quaterniond initial = quatFromRPY(0.0, 0.0, initialYaw);
  ori.step(initial, Eigen::Vector3d(0.0, 0.0, yawRate), acc, DT);
  for (int k = 1; k <= steps; ++k) {
    const Eigen::Quaterniond truth =
        quatFromRPY(0.0, 0.0, initialYaw + yawRate * k * DT);
    ori.step(truth, Eigen::Vector3d(0.0, 0.0, yawRate), acc, DT);
  }

  const double expected = yawRate * steps * DT;
  const Eigen::Vector3d rpy = ori.getRPY();
  std::cout << "    relative yaw " << rpy.z() << " expected " << expected
            << std::endl;
  T_NEAR(rpy.x(), 0.0, 1e-10);
  T_NEAR(rpy.y(), 0.0, 1e-10);
  T_NEAR(rpy.z(), expected, 2e-3);
  std::cout << "  zero_initial_yaw_reports_relative_heading OK" << std::endl;
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
  test_shadow_publishes_truth_while_filter_runs();
  test_tilt_probe_convergence();
  test_bias_gate_during_motion();
  test_bias_gate_reopens_after_quiet();
  test_centripetal_compensation();
  test_zero_initial_yaw_is_applied_once();
  test_zero_initial_yaw_reports_relative_heading();
  test_rotating_frame_accel_filter_invariance();
  test_dynamic_first_accel_does_not_poison_startup();

  std::cout << "test_mahony_filter PASSED" << std::endl;
  return 0;
}
