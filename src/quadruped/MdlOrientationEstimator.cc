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

namespace {

// Return the quaternion exponential of a body-frame rotation vector.
Eigen::Quaterniond rotationIncrement(const Eigen::Vector3d& rotationVector) {
  const double angle = rotationVector.norm();
  if (angle < 1e-12) {
    // Use the normalized first-order form for tiny rotations.
    return Eigen::Quaterniond(1.0, 0.5 * rotationVector.x(),
                              0.5 * rotationVector.y(),
                              0.5 * rotationVector.z())
        .normalized();
  }

  const double scale = std::sin(0.5 * angle) / angle;
  return Eigen::Quaterniond(std::cos(0.5 * angle), scale * rotationVector.x(),
                            scale * rotationVector.y(),
                            scale * rotationVector.z());
}

}  // namespace

// Optional simulation-only ground-truth hook; all uses are null-safe.
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
  // Return ZYX roll, pitch, yaw using the repository's angle convention.
  const double w = q.w(), x = q.x(), y = q.y(), z = q.z();

  Eigen::Vector3d rpy;
  rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));

  // Clamp the asin input against normalization roundoff.
  double s = 2.0 * (w * y - z * x);
  if (s > 1.0) s = 1.0;
  if (s < -1.0) s = -1.0;
  rpy[1] = std::asin(s);

  rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return rpy;
}

void MdlOrientationEstimator::setParams(const params_t& params) {
  _params = params;
  _resetInjectedBias();
  _rng.seed(_params.seed);
}

// The injected bias is also what the error log compares the estimate with, so
// it must read zero when the synthetic corruption is off.
void MdlOrientationEstimator::_resetInjectedBias() {
  const double acc = _params.noise_enable ? _params.acc_bias_init : 0.0;
  const double gyro = _params.noise_enable ? _params.gyro_bias_init : 0.0;
  _injectedAccBias = Eigen::Vector3d::Constant(acc);
  _injectedGyroBias = Eigen::Vector3d::Constant(gyro);
}

void MdlOrientationEstimator::reset() {
  _qFilter = Eigen::Quaterniond::Identity();
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
  _quietTime = 0.0;
  _vBodyHint.setZero();
  _wcorr.setZero();
  _gain = 0.0;
  _haveAttitude = false;
  _haveImu = false;
  _haveTruth = false;
  _elapsed = 0.0;
  _tiltProbePlanted = false;
  _resetInjectedBias();
}

// Propagate the body-to-world attitude and apply the gravity correction.
void MdlOrientationEstimator::_filterAttitude(const Eigen::Vector3d& gyro,
                                              const Eigen::Vector3d& acc, double dt) {
  // Propagate with the bias-corrected rate and transport the stored reference into the new body frame.
  const Eigen::Vector3d omegaPrediction = gyro - _gyroBias;
  const Eigen::Quaterniond delta = rotationIncrement(omegaPrediction * dt);
  const Eigen::Quaterniond qPred = (_qFilter * delta).normalized();
  _accFilt = delta.conjugate() * _accFilt;

  // A turning body reads its centripetal acceleration omega x v as a tilt of
  // gravity: at 0.3 rad/s and 0.3 m/s that is 0.09 m/s^2, i.e. 0.5 deg of
  // roll the filter would settle on. The velocity stage knows v, so remove it.
  const Eigen::Vector3d accGravity = acc - omegaPrediction.cross(_vBodyHint);

  // Low-pass the frame-consistent specific force before using it as gravity.
  const double a = dt / (_params.accel_filter_tau + dt);
  _accFilt += a * (accGravity - _accFilt);

  _wcorr.setZero();
  _gain = 0.0;

  const double anorm = _accFilt.norm();
  if (anorm > 1e-6) {
    // Skip direction normalization when the filtered force is near zero.
    const Eigen::Vector3d upBody = qPred.conjugate() * Eigen::Vector3d::UnitZ();
    _wcorr = (_accFilt / anorm).cross(upBody);

    // Fade the correction when filtered force magnitude leaves the gravity band.
    _gain = std::max(std::min(1.0, 1 - std::fabs(anorm - _params.gravity_magnitude) /
                                       _params.gravity_magnitude),
                     0.0);
  }

  // Integrate the gravity innovation into the gyro-bias estimate, but only
  // while the body is quiet. A fore-aft acceleration of 0.3 m/s^2 at a gait
  // transition tilts the low-passed gravity reference by a/g = 0.03 rad while
  // changing its magnitude by a^2/2g = 0.005 m/s^2, so the gravity band above
  // never notices and with ki = 0.25 the innovation integrated into a 7 mrad/s
  // bias kick that took ~8 s to unwind. Body rates separate the two regimes
  // cleanly: standing reads 0.01-0.04 rad/s including noise, trotting ~0.6.
  // The hold-off of 2 tau lets a polluted reference decay before learning
  // resumes, otherwise the stop transient would still be integrated. While
  // moving, learning is slowed 20x rather than stopped: the kick scales with
  // ki, so 1/20 keeps it under 0.5 mrad/s, while a frozen integrator could not
  // follow the bias random walk through a long trot (1e-4 rad/s/sqrt(s) is
  // 0.8 mrad/s per minute); at ki/20 the walk is still tracked with an ~80 s
  // time constant.
  const bool gateOff = !(_params.bias_gate_rate > 0.0);
  _quietTime = (gateOff || omegaPrediction.norm() < _params.bias_gate_rate) ? _quietTime + dt : 0.0;
  const bool quiet = gateOff || _quietTime >= 2.0 * _params.accel_filter_tau;
  const double ki = quiet ? _params.mahony_ki : 0.05 * _params.mahony_ki;
  _gyroBias -= ki * _gain * _wcorr * dt;

  // Clamp the integrated bias when a positive limit is configured.
  if (_params.mahony_bias_limit > 0.0)
    _gyroBias = _gyroBias.cwiseMax(-_params.mahony_bias_limit)
                    .cwiseMin(_params.mahony_bias_limit);

  // Apply the body-frame innovation after the gyro prediction.
  const Eigen::Vector3d correction = _params.mahony_kp * _gain * _wcorr;
  _qFilter = (qPred * rotationIncrement(correction * dt)).normalized();
}

void MdlOrientationEstimator::step(const Eigen::Quaterniond& q,
                                   const Eigen::Vector3d& gyro,
                                   const Eigen::Vector3d& acc, double dt) {
  // Shadow mode (use_ground_truth) does not bypass the filter: update()
  // publishes truth separately, so the filter is always evaluated.
  if (_params.filter_attitude) {
    // Seed recursive state from the first supplied quaternion.
    if (!_haveAttitude) {
      _qFilter = q.normalized();
      // Initialize the filtered gravity vector from the startup attitude.
      _accFilt = _qFilter.conjugate() *
                 (_params.gravity_magnitude * Eigen::Vector3d::UnitZ());
      _wcorr.setZero();
      _gain = 0.0;
      _haveAttitude = true;
    } else {
      _filterAttitude(gyro, acc, dt);
    }
  } else {
    _qFilter = q.normalized();
  }

  if (_params.zero_initial_yaw) {
    if (!_haveYawDatum) {
      // Capture only the initial yaw; roll and pitch remain in the world frame.
      const double yaw0 = rpyFromQuat(_qFilter).z();
      _qYawInv = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw0, Eigen::Vector3d::UnitZ()));
      _haveYawDatum = true;
    }
    // Left multiplication applies the datum about the world vertical.
    _q = (_qYawInv * _qFilter).normalized();
  } else {
    _q = _qFilter;
  }

  _rpy = rpyFromQuat(_q);
  _Rbw = _q.toRotationMatrix();

  // Publish the bias-corrected body rate and its world-frame rotation.
  _omegaBody = gyro - _gyroBias;
  _omegaWorld = _Rbw * _omegaBody;

  // Publish specific force in body and world frames; gravity remains included.
  _aBody = acc;
  _aWorld = _Rbw * _aBody;

  _ready = true;
}

void MdlOrientationEstimator::setPublishedTruth(const Eigen::Quaterniond& q,
                                                const Eigen::Vector3d& omegaBody,
                                                const Eigen::Vector3d& accBody) {
  _truth.q = q.normalized();
  _truth.Rbw = _truth.q.toRotationMatrix();
  _truth.rpy = rpyFromQuat(_truth.q);
  _truth.omegaBody = omegaBody;
  _truth.omegaWorld = _truth.Rbw * omegaBody;
  _truth.aWorld = _truth.Rbw * accBody;
  _haveTruth = true;
}

void MdlOrientationEstimator::perturbAttitude(const Eigen::Vector3d& rotvecBody) {
  _qFilter = (_qFilter * rotationIncrement(rotvecBody)).normalized();
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
    // Keep filter internals in a separate fixed-width log variable.
    _logserver->registerVar(LOG_DOUBLE, 10, ORIENTATIONMODULE_NAME, "filter",
                            (unsigned char*)_logFilter);
    // Error against simulator truth; stays zero on hardware.
    _logserver->registerVar(LOG_DOUBLE, 11, ORIENTATIONMODULE_NAME, "error",
                            (unsigned char*)_logError);
  }
}

void MdlOrientationEstimator::uninit() {
  DBGPRINT("MdlOrientationEstimator::uninit\n");

  if (_logserver) {
    _logserver->deleteVar(ORIENTATIONMODULE_NAME, "state");
    _logserver->deleteVar(ORIENTATIONMODULE_NAME, "filter");
    _logserver->deleteVar(ORIENTATIONMODULE_NAME, "error");
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
  p.bias_gate_rate = config.getDouble("bias_gate_rate", p.bias_gate_rate);
  p.accel_filter_tau = config.getDouble("accel_filter_tau", p.accel_filter_tau);
  p.gravity_magnitude = config.getDouble("gravity_magnitude", p.gravity_magnitude);
  // Warn when an obsolete gate override is present.
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
    p.tilt_probe_deg = noise.getDouble("tilt_probe_deg", p.tilt_probe_deg);
    p.tilt_probe_time = noise.getDouble("tilt_probe_time", p.tilt_probe_time);
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

  // IMUHW and Eigen both use w-first quaternion order.
  Eigen::Quaterniond q(_imu.q.v[0], _imu.q.v[1], _imu.q.v[2], _imu.q.v[3]);
  Eigen::Vector3d gyro(_imu.gyro.v[0], _imu.gyro.v[1], _imu.gyro.v[2]);
  Eigen::Vector3d acc(_imu.acc.v[0], _imu.acc.v[1], _imu.acc.v[2]);

  const double norm = q.norm();
  // Reject invalid norms and sensor vectors before propagation.
  if (!std::isfinite(norm) || norm < 0.5 || !gyro.allFinite() || !acc.allFinite()) {
    _mgr->warning(ORIENTATIONMODULE_NAME, "Non-finite IMU reading; skipping cycle");
    return;
  }

  // The filter always sees the corrupted IMU, shadow mode included, so the
  // logs measure the estimator a hardware run would have.
  _applySensorNoise(acc, gyro);
  step(q, gyro, acc, _dt);

  // Convergence probe: plant the configured roll error once the robot has had
  // tilt_probe_time seconds to settle, so the decay is measured while standing.
  _elapsed += _dt;
  if (!_tiltProbePlanted && _params.tilt_probe_deg != 0.0 &&
      _elapsed >= _params.tilt_probe_time) {
    perturbAttitude(Eigen::Vector3d(_params.tilt_probe_deg * M_PI / 180.0, 0.0, 0.0));
    _tiltProbePlanted = true;
  }

  if (simGetGroundTruth) {
    double pos[3], quat[4], linvel[3], angvel[3];
    if (simGetGroundTruth(pos, quat, linvel, angvel)) {
      Eigen::Quaterniond qTrue(quat[0], quat[1], quat[2], quat[3]);
      qTrue.normalize();
      // Simulation angular velocity is already in the body frame.
      const Eigen::Vector3d wTrue(angvel[0], angvel[1], angvel[2]);
      if (_params.use_ground_truth) setPublishedTruth(qTrue, wTrue, acc);
      _refreshErrorLog(qTrue, wTrue);
    }
  }
  _refreshLogBuffer();
}

void MdlOrientationEstimator::_refreshErrorLog(const Eigen::Quaterniond& qTrue,
                                               const Eigen::Vector3d& wTrue) {
  // Body-frame rotation vector taking the true attitude to the filter's. Uses
  // _qFilter rather than _q so a zero_initial_yaw datum never reads as error.
  const Eigen::AngleAxisd err(qTrue.conjugate() * _qFilter);
  const Eigen::Vector3d rotvec = err.angle() * err.axis();

  // Split the error the way the sensors see it, not by body axis: gravity
  // observes the angle between the true and estimated up vectors, and nothing
  // observes the heading about the world vertical. Splitting by body-frame
  // components instead leaks heading drift into "roll" whenever the body is
  // pitched (6.7 degrees in the stand pose puts 12 % of the yaw error there).
  const Eigen::Vector3d upTrue = qTrue.conjugate() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d upEst = _qFilter.conjugate() * Eigen::Vector3d::UnitZ();
  const double tilt = std::atan2(upTrue.cross(upEst).norm(), upTrue.dot(upEst));
  Eigen::Vector3d hTrue = qTrue * Eigen::Vector3d::UnitX();
  Eigen::Vector3d hEst = _qFilter * Eigen::Vector3d::UnitX();
  hTrue.z() = 0.0;
  hEst.z() = 0.0;
  const double heading = std::atan2(hTrue.cross(hEst).z(), hTrue.dot(hEst));

  for (int i = 0; i < 3; i++) {
    _logError[i] = rotvec[i];
    _logError[5 + i] = _omegaBody[i] - wTrue[i];
    _logError[8 + i] = _gyroBias[i] - _injectedGyroBias[i];
  }
  _logError[3] = tilt;
  _logError[4] = heading;
}

void MdlOrientationEstimator::_applySensorNoise(Eigen::Vector3d& acc,
                                                Eigen::Vector3d& gyro) {
  if (!_params.noise_enable) return;

  // Apply synthetic noise and bias to rates and specific force, not orientation.
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
    // Estimated gyro bias.
    _logState[13 + i] = _gyroBias[i];
  }

  for (int i = 0; i < 3; i++) {
    // Raw gravity correction and its weight.
    _logFilter[i] = _wcorr[i];
    // Low-passed specific force used by the correction.
    _logFilter[4 + i] = _accFilt[i];
    // Injected gyro bias used by simulation noise.
    _logFilter[7 + i] = _injectedGyroBias[i];
  }
  _logFilter[3] = _gain;
}
