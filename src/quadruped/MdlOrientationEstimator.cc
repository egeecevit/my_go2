/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "quadruped/MdlOrientationEstimator.hh"

#include <algorithm>
#include <cmath>
#include <string>

#include "rtcore/ConfigTable.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ModuleManager.hh"

#define DBGPRINT(...)  // printf(__VA_ARGS__)

using namespace rtcore;

// Simulation ground truth, defined in MdlSimDriver.cc. Only exists in the
// simulation target; on go1 and robot it resolves to null and every use below
// is guarded. Same convention as simPollKey().
extern bool simGetGroundTruth(double pos[3], double quat[4], double linvel[3],
                              double angvel[3]) __attribute__((weak));

MdlOrientationEstimator::MdlOrientationEstimator()
    : Module(ORIENTATIONMODULE_NAME, 0, MULTI_USER) {
  DBGPRINT("MdlOrientationEstimator::MdlOrientationEstimator\n");
}

MdlOrientationEstimator::~MdlOrientationEstimator() {
  DBGPRINT("MdlOrientationEstimator::~MdlOrientationEstimator\n");
}

// ---------------------------------------------------------------------------
//  Attitude derivation
// ---------------------------------------------------------------------------

Eigen::Vector3d MdlOrientationEstimator::rpyFromQuat(const Eigen::Quaterniond& q) {
  // ZYX order, returned as (roll, pitch, yaw), matching MIT's ori::quatToRPY.
  //
  // Written out rather than delegated to Eigen's eulerAngles(2,1,0), which uses
  // a different range convention and can return a representation with pitch
  // outside the usual half-open interval. Attitude here feeds a rotation matrix
  // and a report line, so the angles need to read the way a person expects.
  const double w = q.w(), x = q.x(), y = q.y(), z = q.z();

  Eigen::Vector3d rpy;
  rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));

  // Clamp guards against a normalization error of a few ulp pushing the
  // argument past one, where asin returns NaN and takes the whole estimate
  // with it.
  double s = 2.0 * (w * y - z * x);
  if (s > 1.0) s = 1.0;
  if (s < -1.0) s = -1.0;
  rpy[1] = std::asin(s);

  rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return rpy;
}

void MdlOrientationEstimator::setParams(const params_t& params) {
  _params = params;
  _injectedAccBias = Eigen::Vector3d::Constant(_params.acc_bias_init);
  _injectedGyroBias = Eigen::Vector3d::Constant(_params.gyro_bias_init);
  _rng.seed(_params.seed);
}

void MdlOrientationEstimator::reset() {
  _q = Eigen::Quaterniond::Identity();
  _Rbw.setIdentity();
  _rpy.setZero();
  _omegaBody.setZero();
  _omegaWorld.setZero();
  _aBody.setZero();
  _aWorld.setZero();
  _ready = false;
  _haveYawDatum = false;
  _qYawInv = Eigen::Quaterniond::Identity();
  _gyroBias.setZero();
  _accFilt.setZero();
  _wcorr.setZero();
  _gain = 0.0;
  _haveAttitude = false;
  _haveImu = false;
  _injectedAccBias = Eigen::Vector3d::Constant(_params.acc_bias_init);
  _injectedGyroBias = Eigen::Vector3d::Constant(_params.gyro_bias_init);
}

// One step of the attitude filter of equation (19) of the Cheetah 3 paper,
// which is Mahony's nonlinear complementary filter on SO(3) with the bias
// estimating second state added.
//
// The gyroscope carries the fast dynamics faithfully but integrates its own
// error without bound; the accelerometer cannot follow the fast dynamics but
// knows which way is down. Blending them at a crossover of roughly mahony_kp
// takes the honest half of each.
//
// Our _q is body to world, as is the paper's R-hat, so the expressions carry
// over without transposing anything -- unlike the position filter, which has to
// flip every one of MIT's.
void MdlOrientationEstimator::_filterAttitude(const Eigen::Vector3d& gyro,
                                              const Eigen::Vector3d& acc, double dt) {
  // The rotation that would bring the measured specific force into line with
  // where the estimate currently believes gravity to be. Zero when they already
  // agree, and by construction perpendicular to the vertical, which is why this
  // corrects roll and pitch but never yaw.
  // Low pass the specific force over about a gait period before using it as a
  // gravity reference. On a legged robot there is no instant at which the
  // accelerometer reads gravity alone: the trunk surges fore and aft at twice
  // the gait frequency, about 0.2 g on this robot, and every footfall rings on
  // top. Averaged over a whole cycle that acceleration very nearly cancels,
  // because steady locomotion has no mean acceleration, and what is left is
  // gravity.
  //
  // Filtering rather than gating, and this is the part that is easy to get
  // wrong. A gate on |a| alone does not even see the disturbance -- a specific
  // force tilted by 0.2 g has almost exactly the magnitude of one that is not --
  // and worse, it opens and closes at fixed points in the stride, so it samples
  // an oscillating tilt error phase selectively and rectifies it into a constant
  // one. Measured, that cost 68 mrad of pitch and drove the integrator to twice
  // the true bias.
  const double a = dt / (_params.accel_filter_tau + dt);
  _accFilt += a * (acc - _accFilt);

  _wcorr.setZero();
  _gain = 0.0;

  const double anorm = _accFilt.norm();
  if (anorm > 1e-6) {
    const Eigen::Vector3d upBody = _q.conjugate() * Eigen::Vector3d::UnitZ();
    _wcorr = (_accFilt / anorm).cross(upBody);

    // The magnitude test still earns its place, but as a backstop rather than as
    // the main defence: it catches a genuinely sustained acceleration, a fall or
    // a shove, which no amount of averaging removes because it does not average
    // to zero. The paper's note that kappa is decreased "during highly-dynamic
    // portions of the gait". After filtering it is open nearly all the time.
    _gain = std::max(std::min(1.0, 1 - std::fabs(anorm - _params.gravity_magnitude) /
                                       _params.gravity_magnitude),
                     0.0);
  }

  // The integral path, absent from equation (19) as printed. Without it a
  // constant gyro bias leaves a standing tilt error, because the estimate
  // settles wherever kp*wcorr happens to cancel the bias and the bias itself is
  // never named. Integrating the same error signal absorbs the constant, in the
  // way the I term of a PI loop does, and leaves the integrator holding the
  // quantity worth having.
  _gyroBias -= _params.mahony_ki * _gain * _wcorr * dt;

  // Anti-windup. The gate above rejects a specific force whose *magnitude* has
  // left the band, but a sustained acceleration that merely tilts the reference
  // -- a shove, a slope, a long fall -- keeps the gate open while feeding the
  // integrator an error that never averages away. Nothing else in the loop
  // stops it, since to the filter that is indistinguishable from a gyroscope
  // drifting. The limit is set far above any bias a flyable MEMS gyro has, so
  // in normal operation this never binds; it exists so that a few seconds of
  // abuse cannot leave a corrupted rate correction behind afterwards.
  if (_params.mahony_bias_limit > 0.0)
    _gyroBias = _gyroBias.cwiseMax(-_params.mahony_bias_limit)
                    .cwiseMin(_params.mahony_bias_limit);

  const Eigen::Vector3d w = gyro - _gyroBias + _params.mahony_kp * _gain * _wcorr;

  // qdot = 0.5 * q * (0, w) for a body-to-world Hamilton quaternion, with w in
  // the body frame. Explicit Euler is ample at 1 kHz against a body turning at
  // a few rad/s; the renormalization below absorbs what it loses.
  const Eigen::Quaterniond qw(0.0, w.x(), w.y(), w.z());
  const Eigen::Quaterniond qdot = _q * qw;
  _q.coeffs() += 0.5 * dt * qdot.coeffs();
  _q.normalize();
}

void MdlOrientationEstimator::step(const Eigen::Quaterniond& q,
                                   const Eigen::Vector3d& gyro,
                                   const Eigen::Vector3d& acc, double dt) {
  if (_params.filter_attitude) {
    // Seeded from the sensor rather than started at identity, so the filter
    // begins already level instead of spending its first seconds rotating there
    // and dragging the position estimate along with it.
    if (!_haveAttitude) {
      _q = q.normalized();
      // Seeded too, so the low pass starts from the current reading instead of
      // sweeping up from zero and tilting the estimate over its first second.
      _accFilt = acc;
      _haveAttitude = true;
    }
    _filterAttitude(gyro, acc, dt);
  } else {
    _q = q.normalized();
  }

  if (_params.zero_initial_yaw) {
    if (!_haveYawDatum) {
      // Only the yaw is removed. Roll and pitch are observable through gravity
      // and are meaningful in absolute terms, so cancelling them would discard
      // real information; yaw has no such reference and its origin is
      // arbitrary. MIT reaches this by zeroing the roll and pitch entries of
      // the initial rpy before inverting, which amounts to the same rotation.
      const double yaw0 = rpyFromQuat(_q).z();
      _qYawInv = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw0, Eigen::Vector3d::UnitZ()));
      _haveYawDatum = true;
    }
    // Left multiplication, so the correction is applied about the *world*
    // vertical. Multiplying on the right would rotate about the body's own z
    // axis, which for any non level attitude mixes the correction into roll and
    // pitch and corrupts both.
    _q = (_qYawInv * _q).normalized();
  }

  _rpy = rpyFromQuat(_q);
  _Rbw = _q.toRotationMatrix();

  // Bias corrected, which matters downstream rather than here: this rate drives
  // the position filter's foot velocity transport term, omega x p_rel, where an
  // uncorrected bias becomes a body velocity error of about bias times leg
  // length and integrates into position. _gyroBias is zero unless the attitude
  // filter is running, so this is a no-op on the passthrough path.
  _omegaBody = gyro - _gyroBias;
  _omegaWorld = _Rbw * _omegaBody;

  // Specific force, gravity included: at rest and level the accelerometer reads
  // (0, 0, +9.81) in both frames. The position filter is what subtracts gravity,
  // since it is the stage that needs an inertial acceleration.
  _aBody = acc;
  _aWorld = _Rbw * _aBody;

  _ready = true;
}

// ---------------------------------------------------------------------------
//  Module interface
// ---------------------------------------------------------------------------

void MdlOrientationEstimator::init() {
  DBGPRINT("MdlOrientationEstimator::init\n");

  _dt = CLOCK_TO_SEC(_mgr->getStepPeriod());
  _readConfig();

  _logserver = (LogServer*)_mgr->findModule(LOGSERVER_NAME, 0);
  if (_logserver) {
    _logserver->registerVar(LOG_DOUBLE, 16, ORIENTATIONMODULE_NAME, "state",
                            (unsigned char*)_logState);
    // Separate from "state" rather than widening it: the offline plotting
    // scripts index that variable by hand, so its width is part of its
    // interface.
    _logserver->registerVar(LOG_DOUBLE, 10, ORIENTATIONMODULE_NAME, "filter",
                            (unsigned char*)_logFilter);
  }
}

void MdlOrientationEstimator::uninit() {
  DBGPRINT("MdlOrientationEstimator::uninit\n");

  if (_logserver) {
    _logserver->deleteVar(ORIENTATIONMODULE_NAME, "state");
    _logserver->deleteVar(ORIENTATIONMODULE_NAME, "filter");
    _logserver = nullptr;
  }
}

void MdlOrientationEstimator::_readConfig() {
  ConfigTable config;
  if (!_mgr->getConfigTable("orientationestimator", config)) {
    _mgr->warning(ORIENTATIONMODULE_NAME,
                  "No [orientationestimator] config table; using built-in defaults");
    return;
  }

  _enable = config.getBool("enable", true);

  params_t p;
  p.zero_initial_yaw = config.getBool("zero_initial_yaw", p.zero_initial_yaw);
  p.use_ground_truth = config.getBool("use_ground_truth", p.use_ground_truth);

  const std::string src = config.getString("attitude_source", "imu");
  if (src == "filter") {
    p.filter_attitude = true;
  } else if (src != "imu") {
    _mgr->warning(ORIENTATIONMODULE_NAME,
                  "Unknown attitude_source '%s'; using 'imu'", src.c_str());
  }
  p.mahony_kp = config.getDouble("mahony_kp", p.mahony_kp);
  p.mahony_ki = config.getDouble("mahony_ki", p.mahony_ki);
  p.mahony_bias_limit = config.getDouble("mahony_bias_limit", p.mahony_bias_limit);
  p.accel_filter_tau = config.getDouble("accel_filter_tau", p.accel_filter_tau);
  p.gravity_magnitude = config.getDouble("gravity_magnitude", p.gravity_magnitude);
  // Silently ignoring a stale override left in a version or robot directory
  // would mean running with settings nobody intended.
  if (config.getDouble("accel_gate", -1.0) >= 0.0)
    _mgr->warning(ORIENTATIONMODULE_NAME,
                  "accel_gate is obsolete; the gravity correction weight is "
                  "normalized by gravity_magnitude, which sets the band width");

  if (p.use_ground_truth && !simGetGroundTruth) {
    _mgr->warning(ORIENTATIONMODULE_NAME,
                  "use_ground_truth needs the simulation target; using the IMU");
    p.use_ground_truth = false;
  }

  ConfigTable noise;
  if (config.getTable("simnoise", noise)) {
    p.noise_enable = noise.getBool("enable", p.noise_enable);
    p.acc_noise_std = noise.getDouble("acc_noise_std", p.acc_noise_std);
    p.gyro_noise_std = noise.getDouble("gyro_noise_std", p.gyro_noise_std);
    p.acc_bias_walk = noise.getDouble("acc_bias_walk", p.acc_bias_walk);
    p.gyro_bias_walk = noise.getDouble("gyro_bias_walk", p.gyro_bias_walk);
    p.acc_bias_init = noise.getDouble("acc_bias_init", p.acc_bias_init);
    p.gyro_bias_init = noise.getDouble("gyro_bias_init", p.gyro_bias_init);
    p.seed = (unsigned int)noise.getInt("seed", p.seed);
  }

  setParams(p);
}

void MdlOrientationEstimator::activate() {
  DBGPRINT("MdlOrientationEstimator::activate\n");
  reset();
}

void MdlOrientationEstimator::deactivate() {
  DBGPRINT("MdlOrientationEstimator::deactivate\n");
}

void MdlOrientationEstimator::update() {
  if (!_enable) return;

  IMUHW* imuhw = IMUHW::instance();
  if (imuhw && imuhw->getLastReading(0, _imu)) _haveImu = true;
  if (!_haveImu) return;

  // IMUHW reports w first, which is also Eigen's constructor order, so unlike
  // the MIT sources there is no index shuffle here.
  Eigen::Quaterniond q(_imu.q.v[0], _imu.q.v[1], _imu.q.v[2], _imu.q.v[3]);
  Eigen::Vector3d gyro(_imu.gyro.v[0], _imu.gyro.v[1], _imu.gyro.v[2]);
  Eigen::Vector3d acc(_imu.acc.v[0], _imu.acc.v[1], _imu.acc.v[2]);

  const double norm = q.norm();
  if (!std::isfinite(norm) || norm < 0.5 || !gyro.allFinite() || !acc.allFinite()) {
    _mgr->warning(ORIENTATIONMODULE_NAME, "Non-finite IMU reading; skipping cycle");
    return;
  }

  if (_params.use_ground_truth && simGetGroundTruth) {
    double pos[3], quat[4], linvel[3], angvel[3];
    if (simGetGroundTruth(pos, quat, linvel, angvel)) {
      q = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
      // angvel is already body frame, matching what the gyroscope reports.
      gyro = Eigen::Vector3d(angvel[0], angvel[1], angvel[2]);
      // Acceleration stays with the IMU: the simulator reports none, and
      // leaving that path exercised is the point of isolating attitude.
    }
  } else {
    _applySensorNoise(acc, gyro);
  }

  step(q, gyro, acc, _dt);
  _refreshLogBuffer();
}

void MdlOrientationEstimator::_applySensorNoise(Eigen::Vector3d& acc,
                                                Eigen::Vector3d& gyro) {
  if (!_params.noise_enable) return;

  // Note that this corrupts only the rates and forces, not the quaternion. The
  // IMU's attitude output is taken as given by this design, so an honest
  // simulation of a poor IMU has to reach the position filter through the
  // accelerometer and the gyroscope, which is where the corruption actually
  // hurts: through aWorld driving the prediction, and through omegaBody in the
  // foot velocity cross term.
  const double sqrtDt = std::sqrt(_dt);

  for (int i = 0; i < 3; i++) {
    _injectedAccBias[i] += _params.acc_bias_walk * sqrtDt * _randn();
    _injectedGyroBias[i] += _params.gyro_bias_walk * sqrtDt * _randn();
  }

  for (int i = 0; i < 3; i++) {
    acc[i] += _injectedAccBias[i] + _params.acc_noise_std * _randn();
    gyro[i] += _injectedGyroBias[i] + _params.gyro_noise_std * _randn();
  }
}

void MdlOrientationEstimator::_refreshLogBuffer() {
  _logState[0] = _q.w();
  _logState[1] = _q.x();
  _logState[2] = _q.y();
  _logState[3] = _q.z();
  for (int i = 0; i < 3; i++) {
    _logState[4 + i] = _rpy[i];
    _logState[7 + i] = _omegaBody[i];
    _logState[10 + i] = _aWorld[i];
    // Recorded because whether the attitude filter is working is exactly the
    // question of whether this converges, and to what.
    _logState[13 + i] = _gyroBias[i];
  }

  for (int i = 0; i < 3; i++) {
    // The correction path, which used to be reconstructible only backwards from
    // the attitude it produced. wcorr is the raw gravity disagreement and gain
    // is how much of it was believed, so the two together say whether a bias
    // estimate that went somewhere odd was fed a bad reference or simply not
    // fed at all.
    _logFilter[i] = _wcorr[i];
    // The low passed specific force, since it and not the raw accelerometer is
    // what the correction actually sees.
    _logFilter[4 + i] = _accFilt[i];
    // And the injected bias, which turns the question above from a judgement
    // into a subtraction: with simnoise on, this is the exact quantity
    // _logState[13 + i] is trying to find, so the true estimation error is a
    // difference of two logged columns rather than an eyeball on a curve.
    _logFilter[7 + i] = _injectedGyroBias[i];
  }
  _logFilter[3] = _gain;
}
