/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "quadruped/MdlOrientationEstimator.hh"

#include <cmath>

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
  _haveImu = false;
  _injectedAccBias = Eigen::Vector3d::Constant(_params.acc_bias_init);
  _injectedGyroBias = Eigen::Vector3d::Constant(_params.gyro_bias_init);
}

void MdlOrientationEstimator::step(const Eigen::Quaterniond& q,
                                   const Eigen::Vector3d& gyro,
                                   const Eigen::Vector3d& acc, double dt) {
  (void)dt;

  _q = q.normalized();

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

  _omegaBody = gyro;
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
  if (_logserver)
    _logserver->registerVar(LOG_DOUBLE, 13, ORIENTATIONMODULE_NAME, "state",
                            (unsigned char*)_logState);
}

void MdlOrientationEstimator::uninit() {
  DBGPRINT("MdlOrientationEstimator::uninit\n");

  if (_logserver) {
    _logserver->deleteVar(ORIENTATIONMODULE_NAME, "state");
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
  }
}
