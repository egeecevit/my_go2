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

  Ported from the orientation estimator of MIT Cheetah-Software
  (common/src/Controllers/OrientationEstimator.cpp). The approach is
  deliberately not a filter. The IMU already runs an attitude filter of its own,
  fusing its gyroscope against gravity, and it does so with far more information
  about its own noise characteristics than anything downstream could
  reconstruct. So this module takes that quaternion as given and spends its
  effort on deriving the quantities the rest of the estimator needs from it.

  That is the substantive difference from the EKF this replaces, which estimated
  attitude jointly with position and carried gyroscope bias states to do it.
  Splitting the two apart is what makes the second stage a plain linear Kalman
  filter rather than an extended one: with the rotation known, the relationship
  between body position, foot positions and their velocities is linear.

  Outputs, all in the conventions of this code base rather than MIT's:

    - orientation, an Eigen quaternion mapping body to inertial
    - _Rbw = q.toRotationMatrix(), which takes a body frame vector to the world
    - rpy, in ZYX order, returned as (roll, pitch, yaw)
    - angular velocity and specific force, in both the body and world frames

  On the rotation convention: MIT's StateEstimate::rBody is world to body, since
  their ori::quaternionToRotationMatrix transposes before returning. Ours is the
  opposite, which is what Eigen gives. Anywhere MIT writes rBody we write
  _Rbw.transpose() and anywhere they write rBody.transpose() we write _Rbw. The
  member is named _Rbw rather than rBody precisely so that reading this file
  next to theirs does not quietly suggest the two are the same thing.

  Configuration lives in the [orientationestimator] table; see
  stateestimator.toml.
 */
class MdlOrientationEstimator : public rtcore::Module {
 public:
  /** \brief Tunable parameters, settable without a ModuleManager so the module
      can be exercised from a unit test. */
  struct params_t {
    /** \brief Rotate the estimate so the initial yaw reads zero.

      MIT does this unconditionally, and on real hardware it is right: the IMU's
      yaw origin is whatever it happened to be at power-on and carries no
      meaning. In simulation it is wrong by default, because it puts the
      estimator's world frame at an angle to the simulator's and would make the
      ground truth comparison in MdlPosVelEstimator report a large error for a
      filter that is tracking perfectly. */
    bool zero_initial_yaw = false;

    /** \brief Take attitude from the simulator instead of the IMU.

      Simulation only. This is the "cheater" estimator of the MIT sources,
      reduced to a flag: with a perfect attitude, any remaining error belongs to
      the position and velocity filter, which is the first thing worth knowing
      when the estimate looks wrong. */
    bool use_ground_truth = false;

    // Synthetic IMU corruption. The simulated IMU has no noise and no bias and
    // its orientation comes straight from the simulator state, so an estimator
    // fed that data is never tested. These inject a plausible IMU instead.
    // Never enabled on real hardware, which supplies its own imperfections.
    bool noise_enable = false;
    double acc_noise_std = 0.05;      // [m/s^2]
    double gyro_noise_std = 0.005;    // [rad/s]
    double acc_bias_walk = 0.001;     // [m/s^2 / sqrt(s)]
    double gyro_bias_walk = 0.0001;   // [rad/s / sqrt(s)]
    double acc_bias_init = 0.02;      // constant initial bias, every axis
    double gyro_bias_init = 0.002;
    unsigned int seed = 12345;
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
    @param dt Timestep, used only by the synthetic bias random walk */
  void step(const Eigen::Quaterniond& q, const Eigen::Vector3d& gyro,
            const Eigen::Vector3d& acc, double dt);

  // Estimate accessors. All return zero-ish defaults until the first step().
  const Eigen::Quaterniond& getOrientation() const { return _q; }
  /** \brief Body to world rotation. MIT's rBody is this transposed. */
  const Eigen::Matrix3d& getRotation() const { return _Rbw; }
  /** \brief Roll, pitch and yaw in ZYX order [rad]. */
  const Eigen::Vector3d& getRPY() const { return _rpy; }
  const Eigen::Vector3d& getAngularVelocity() const { return _omegaBody; }
  const Eigen::Vector3d& getAngularVelocityWorld() const { return _omegaWorld; }
  /** \brief Specific force in the body frame; reads (0,0,+9.81) at rest. */
  const Eigen::Vector3d& getAcceleration() const { return _aBody; }
  /** \brief Specific force rotated to the world frame. Add gravity to get the
      inertial acceleration. */
  const Eigen::Vector3d& getAccelerationWorld() const { return _aWorld; }

  /** \brief True once at least one IMU sample has been processed. */
  bool isReady() const { return _ready; }

  /** \brief Roll, pitch and yaw of a quaternion, ZYX order.

    Static because MdlPosVelEstimator needs the same conversion for its ground
    truth comparison, and because a test can then check it against known
    angles without constructing a module. */
  static Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond& q);

 private:
  params_t _params;

  Eigen::Quaterniond _q = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d _Rbw = Eigen::Matrix3d::Identity();
  Eigen::Vector3d _rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d _omegaBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d _omegaWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d _aBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d _aWorld = Eigen::Vector3d::Zero();
  bool _ready = false;

  // Yaw datum, captured on the first sample when zero_initial_yaw is set.
  bool _haveYawDatum = false;
  Eigen::Quaterniond _qYawInv = Eigen::Quaterniond::Identity();

  // Persistent, because IMUHW::getLastReading() is edge triggered: it reports
  // no new data when the timestamp it is handed matches the one it holds. A
  // stack local would be zeroed each cycle and would therefore always appear to
  // succeed, silently reprocessing stale samples.
  IMUHW::imudata_t _imu;
  bool _haveImu = false;

  bool _enable = true;
  double _dt = 0.001;

  // Injected corruption, integrated across cycles.
  Eigen::Vector3d _injectedAccBias = Eigen::Vector3d::Zero();
  Eigen::Vector3d _injectedGyroBias = Eigen::Vector3d::Zero();
  std::mt19937 _rng;
  std::normal_distribution<double> _gauss{0.0, 1.0};

  rtcore::LogServer* _logserver = nullptr;
  // quat(w,x,y,z), rpy, omegaBody, aWorld
  double _logState[13] = {0};

  void _readConfig();
  void _applySensorNoise(Eigen::Vector3d& acc, Eigen::Vector3d& gyro);
  void _refreshLogBuffer();
  double _randn() { return _gauss(_rng); }
};

#endif  // MDLORIENTATIONESTIMATOR_HH
