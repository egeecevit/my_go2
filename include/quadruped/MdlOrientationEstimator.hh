/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef MDLORIENTATIONESTIMATOR_HH
#define MDLORIENTATIONESTIMATOR_HH

#include <Eigen/Dense>
#include <random>

#include "hardware/IMUHW.hh"
#include "rtcore/Module.hh"

#define ORIENTATIONMODULE_NAME "MdlOrientationEstimator"

namespace rtcore {
class LogServer;
}

/** \brief First stage of the state estimator: body attitude.

  The module either passes through the IMU quaternion or runs the configured
  complementary filter. It publishes a body-to-world quaternion, its
  body-to-world rotation matrix, ZYX roll/pitch/yaw, body and world angular
  velocity, and body and world specific force.

  Shadow mode (simulation only): with use_ground_truth the filter keeps running
  on the corrupted IMU and is what the logs record, while the accessors hand
  consumers the simulator's attitude and body rate. That separates "how good is
  the estimator" from "how well does the controller cope with it".

  Configuration lives in the [orientationestimator] table in stateestimator.toml.
 */
class MdlOrientationEstimator : public rtcore::Module {
 public:
  /** \brief Runtime estimator parameters. */
  struct params_t {
    /** \brief Apply a one-time inverse initial yaw to the published attitude. */
    bool zero_initial_yaw = false;

    /** \brief Publish simulator attitude to consumers while the filter keeps
        running on the IMU. Shadow mode, simulation only. */
    bool use_ground_truth = false;

    /** \brief Run the complementary attitude filter after seeding from the first quaternion. */
    bool filter_attitude = false;

    /** \brief Proportional complementary-filter gain [1/s]. */
    double mahony_kp = 1.0;

    /** \brief Integral complementary-filter gain [1/s^2]; zero disables bias learning. */
    double mahony_ki = 0.25;

    /** \brief Time constant for the specific-force low-pass gravity reference [s]. */
    double accel_filter_tau = 0.5;

    /** \brief Symmetric per-axis gyro-bias clamp [rad/s]; nonpositive disables it. */
    double mahony_bias_limit = 0.02;

    /** \brief Gyro-bias learning runs at full ki only while the bias-corrected
        body rate has stayed below this [rad/s] for 2 * accel_filter_tau, and at
        ki/20 otherwise. Nonpositive disables the gate. */
    double bias_gate_rate = 0.1;

    /** \brief Gravity magnitude and center of the correction-weight band [m/s^2]. */
    double gravity_magnitude = 9.81;

    // Synthetic IMU noise and bias parameters for simulation tests.
    bool noise_enable = false;
    double acc_noise_std = 0.05;      // [m/s^2]
    double gyro_noise_std = 0.005;    // [rad/s]
    double acc_bias_walk = 0.001;     // [m/s^2 / sqrt(s)]
    double gyro_bias_walk = 0.0001;   // [rad/s / sqrt(s)]
    double acc_bias_init = 0.02;      // constant initial bias, every axis
    // Applied on all three axes.
    double gyro_bias_init = 0.002;
    unsigned int seed = 12345;

    /** \brief Convergence probe: a roll error [deg] planted on the filter
        attitude once, tilt_probe_time seconds after activation. Applied
        regardless of noise_enable; zero disables it. */
    double tilt_probe_deg = 0.0;
    double tilt_probe_time = 0.0;
  };

  MdlOrientationEstimator();
  ~MdlOrientationEstimator();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  void setParams(const params_t& params);
  const params_t& getParams() const { return _params; }

  /** \brief Clears the estimate and the captured yaw datum. */
  void reset();

  /** \brief Derives the full attitude state from one IMU sample.

    Separated from update() so the derivation can be driven directly, without a
    ModuleManager or any hardware.

    @param q Body to inertial orientation, as reported by the IMU
    @param gyro Angular rate in the body frame [rad/s]
    @param acc Specific force in the body frame [m/s^2], gravity not removed
    @param dt Timestep [s]: the attitude propagation and bias integration step
           of the filter, and the interval the synthetic bias random walk is
           scaled by. Ignored on the passthrough path, which does not
           integrate anything. */
  void step(const Eigen::Quaterniond& q, const Eigen::Vector3d& gyro,
            const Eigen::Vector3d& acc, double dt);

  // Estimate accessors return initialized defaults until the first step().
  // In shadow mode (setPublishedTruth) they return the simulator's attitude;
  // the filter members behind them are what the logs record either way.
  const Eigen::Quaterniond& getOrientation() const { return _haveTruth ? _truth.q : _q; }
  /** \brief Body-to-world rotation matrix. */
  const Eigen::Matrix3d& getRotation() const { return _haveTruth ? _truth.Rbw : _Rbw; }
  /** \brief Roll, pitch and yaw in ZYX order [rad]. */
  const Eigen::Vector3d& getRPY() const { return _haveTruth ? _truth.rpy : _rpy; }
  const Eigen::Vector3d& getAngularVelocity() const {
    return _haveTruth ? _truth.omegaBody : _omegaBody;
  }
  const Eigen::Vector3d& getAngularVelocityWorld() const {
    return _haveTruth ? _truth.omegaWorld : _omegaWorld;
  }
  /** \brief Specific force in the body frame; reads (0,0,+9.81) at rest. */
  const Eigen::Vector3d& getAcceleration() const { return _aBody; }
  /** \brief Specific force rotated to the world frame by the published
      attitude, so it stays consistent with getRotation() in shadow mode. Add
      gravity to get the inertial acceleration. */
  const Eigen::Vector3d& getAccelerationWorld() const {
    return _haveTruth ? _truth.aWorld : _aWorld;
  }

  /** \brief The filter's own attitude, regardless of shadow mode. */
  const Eigen::Quaterniond& getFilterOrientation() const { return _q; }

  /** \brief Shadow mode: publish this attitude and body rate to consumers while
      the filter keeps running. update() calls it with simulator truth; public
      so tests can drive it without a simulator. */
  void setPublishedTruth(const Eigen::Quaterniond& q, const Eigen::Vector3d& omegaBody,
                         const Eigen::Vector3d& accBody);

  /** \brief Rotate the filter attitude by a body-frame rotation vector [rad].
      The convergence probe behind tilt_probe_deg; tests call it directly. */
  void perturbAttitude(const Eigen::Vector3d& rotvecBody);

  /** \brief Body-frame velocity estimate from the position/velocity stage,
      used to remove the centripetal term omega x v from the specific force
      before it serves as the gravity reference. Zero when unknown. */
  void setBodyVelocityHint(const Eigen::Vector3d& vBody) { _vBodyHint = vBody; }

  /** \brief True once at least one IMU sample has been processed. */
  bool isReady() const { return _ready; }

  /** \brief Estimated gyro bias [rad/s], already subtracted from angular velocity. */
  const Eigen::Vector3d& getGyroBias() const { return _gyroBias; }

  /** \brief Return roll, pitch, and yaw from a quaternion in ZYX order. */
  static Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond& q);

 private:
  params_t _params;

  // Recursive attitude state; keep it separate from the published yaw-adjusted output.
  Eigen::Quaterniond _qFilter = Eigen::Quaterniond::Identity();

  // Published attitude, optionally left-multiplied by the one-time yaw datum.
  Eigen::Quaterniond _q = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d _Rbw = Eigen::Matrix3d::Identity();
  Eigen::Vector3d _rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d _omegaBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d _omegaWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d _aBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d _aWorld = Eigen::Vector3d::Zero();
  bool _ready = false;

  // Shadow mode: simulator truth handed to consumers while the filter above
  // keeps running on the IMU. See setPublishedTruth().
  struct attitude_t {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d Rbw = Eigen::Matrix3d::Identity();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d omegaBody = Eigen::Vector3d::Zero();
    Eigen::Vector3d omegaWorld = Eigen::Vector3d::Zero();
    Eigen::Vector3d aWorld = Eigen::Vector3d::Zero();
  } _truth;
  bool _haveTruth = false;

  // Convergence probe bookkeeping: seconds since activation, planted once.
  double _elapsed = 0.0;
  bool _tiltProbePlanted = false;

  // Yaw datum, captured on the first sample when zero_initial_yaw is set.
  bool _haveYawDatum = false;
  Eigen::Quaterniond _qYawInv = Eigen::Quaterniond::Identity();

  // Filter state; _accFilt is transported into the latest body frame before blending.
  Eigen::Vector3d _gyroBias = Eigen::Vector3d::Zero();
  Eigen::Vector3d _accFilt = Eigen::Vector3d::Zero();
  bool _haveAttitude = false;
  // Seconds the body rate has stayed under bias_gate_rate; see _filterAttitude().
  double _quietTime = 0.0;
  // Previous-tick body velocity from MdlPosVelEstimator; see setBodyVelocityHint().
  Eigen::Vector3d _vBodyHint = Eigen::Vector3d::Zero();

  // Last correction and its weight, retained for logging.
  Eigen::Vector3d _wcorr = Eigen::Vector3d::Zero();
  double _gain = 0.0;

  // Persistent IMU sample state for the edge-triggered getLastReading() API.
  IMUHW::imudata_t _imu;
  bool _haveImu = false;

  bool _enable = true;
  double _dt = 0.001;

  // Injected bias state, integrated across cycles.
  Eigen::Vector3d _injectedAccBias = Eigen::Vector3d::Zero();
  Eigen::Vector3d _injectedGyroBias = Eigen::Vector3d::Zero();
  std::mt19937 _rng;
  std::normal_distribution<double> _gauss{0.0, 1.0};

  rtcore::LogServer* _logserver = nullptr;
  // quat(w,x,y,z), rpy, omegaBody, aWorld, gyroBias
  double _logState[16] = {0};
  // Filter internals: wcorr, gain, accFilt, and injectedGyroBias.
  double _logFilter[10] = {0};
  // Error against simulator truth at the same tick: rotation vector of
  // R_true^T R_filter, the gravity-observable tilt angle, the heading error
  // about the world vertical, body-rate error, gyro-bias error.
  double _logError[11] = {0};

  void _readConfig();
  void _refreshErrorLog(const Eigen::Quaterniond& qTrue, const Eigen::Vector3d& wTrue);
  /** \brief One step of the Mahony complementary filter; see params_t. */
  void _filterAttitude(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc,
                       double dt);
  void _applySensorNoise(Eigen::Vector3d& acc, Eigen::Vector3d& gyro);
  void _resetInjectedBias();
  void _refreshLogBuffer();
  double _randn() { return _gauss(_rng); }
};

#endif  // MDLORIENTATIONESTIMATOR_HH
