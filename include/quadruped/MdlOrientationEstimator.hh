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

    /** \brief Estimate attitude here rather than taking the IMU's quaternion.

      Off by default, which is what Cheetah-Software does: its
      VectorNavOrientationEstimator copies the quaternion straight out of the
      sensor, on the reasoning that the VN-100's own EKF has far better
      knowledge of its noise than anything downstream could reconstruct.

      Turned on, this runs the filter of equation (19) of the Cheetah 3 paper --
      a Mahony nonlinear complementary filter on SO(3) -- which is what the
      VN-100 is doing internally. Wanted when the IMU reports rates but no
      attitude, when its attitude is not trusted, and in simulation, where the
      quaternion is the simulator's own and passing it through means nothing is
      ever tested. See _filterAttitude(). */
    bool filter_attitude = false;

    /** \brief Proportional gain of the attitude filter, kappa in eq (19) [1/s].
        The de-drifting time constant is roughly its reciprocal. Larger pulls
        roll and pitch back to gravity faster and lets more of the gait's own
        acceleration into the estimate. Bounded above by the gait: the trunk
        genuinely rolls and pitches through a stride, and a loop fast enough to
        argue with that leaves a gait locked attitude error. Choose it against
        mahony_ki by damping rather than alone; see there and the config. */
    double mahony_kp = 0.6;

    /** \brief Integral gain of the attitude filter [1/s^2], zero for eq (19)
        exactly.

      Equation (19) is proportional only, so a constant gyro bias leaves a
      standing orientation error: the estimate settles wherever k_P * w_corr
      happens to cancel the bias, and the bias is never identified. This is the
      second state of Mahony's explicit complementary filter, bdot = -k_I w_corr,
      which absorbs the constant exactly as the integral term of a PI loop does.
      What it buys beyond a truer attitude is a gyro bias estimate to subtract
      before the rate is handed on, and that rate drives the position filter's
      foot velocity transport term, where an uncorrected bias becomes a lateral
      velocity error of the order of bias times leg length.

      Only roll and pitch converge. w_corr is a cross product with the vertical
      so it never has a component along it, leaving the yaw bias unobservable --
      the same reason heading drifts, and not a defect. The heading error that
      follows is accepted here; nothing in this module tries to remove it.

      Pick this against mahony_kp by the damping of the pair, zeta =
      mahony_kp / (2 sqrt(mahony_ki)), rather than either alone: the settling
      time of the bias estimate is what shows up downstream, because everything
      before it settles is a transient the position filter integrates. */
    double mahony_ki = 0.09;

    /** \brief Time constant for low passing the specific force before it is
        used as a gravity reference [s]. Should span at least one gait period.

      This, not the gate, is what makes the accelerometer usable on a legged
      robot. There is no instant at which it reads gravity alone, but over a
      whole gait cycle the trunk's own acceleration averages to nearly nothing.
      See _filterAttitude() for why gating cannot substitute. */
    double accel_filter_tau = 0.5;

    /** \brief Symmetric per-axis clamp on the bias estimate [rad/s], zero or
        negative to leave it unbounded.

      Bounds one failure, and only one: a sustained acceleration that averaging
      does not remove -- a long fall, a shove held for seconds, a run down a
      slope -- tilts the filtered gravity reference for as long as it lasts, and
      the integrator, having no way to tell that from a gyroscope drifting, walks
      off to a value no real gyro bias could have. Since the bias is subtracted
      from the rate handed to the position filter, that walk outlives the event
      that caused it. The clamp costs nothing in normal operation, where the
      estimate sits near the true 0.002 rad/s, two orders below the default. */
    double mahony_bias_limit = 0.02;

    /** \brief Magnitude of gravity [m/s^2].

      Doubles as the width of the band around g over which the gravity
      correction is believed: both gains fade linearly to zero as the filtered
      specific force magnitude departs from g by this much. A backstop against
      a sustained acceleration that averaging will not remove, not the main
      defence, which is accel_filter_tau. The paper's note that kappa is
      "heuristically decreased during highly-dynamic portions of the gait where
      |a_b| >> g". */
    double gravity_magnitude = 9.81;

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
    // Applied on all three axes, which is what a raw gyroscope looks like. Note
    // that this is only realistic alongside filter_attitude: an IMU reporting a
    // trustworthy quaternion has necessarily found its own roll and pitch bias,
    // since that is how it holds attitude, and would report rates with that part
    // already removed. Injecting it here while passing the quaternion through
    // models a unit that knows its attitude perfectly and has made no attempt to
    // correct its rates, which is not a device that exists.
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
    @param dt Timestep [s]: the attitude propagation and bias integration step
           of the filter, and the interval the synthetic bias random walk is
           scaled by. Ignored on the passthrough path, which does not
           integrate anything. */
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

  /** \brief Gyroscope bias the attitude filter has found [rad/s].

    Zero unless filter_attitude is set. Roll and pitch are the axes gravity
    observes and the only ones that converge; the z component is not pinned to
    zero, because wcorr is perpendicular to the *measured* gravity direction
    rather than to body z, so any tilt cross-couples a little of the correction
    onto that axis. It stays small and means nothing: global yaw is unobservable
    from gravity whatever lands there. Already subtracted from
    getAngularVelocity(). */
  const Eigen::Vector3d& getGyroBias() const { return _gyroBias; }

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

  // Attitude filter state. _gyroBias is the integral term of the complementary
  // filter and stays at zero unless filter_attitude is set, so the subtraction
  // in step() is a no-op on the passthrough path and needs no branch.
  Eigen::Vector3d _gyroBias = Eigen::Vector3d::Zero();
  Eigen::Vector3d _accFilt = Eigen::Vector3d::Zero();
  bool _haveAttitude = false;

  // The last step's correction and its weight. Members rather than locals only
  // so they can be logged: they are the whole correction path, and without them
  // a bias estimate going somewhere unexpected can only be reasoned about
  // backwards from the attitude it produced.
  Eigen::Vector3d _wcorr = Eigen::Vector3d::Zero();
  double _gain = 0.0;

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
  // quat(w,x,y,z), rpy, omegaBody, aWorld, gyroBias
  double _logState[16] = {0};
  // Attitude filter internals: wcorr, gain, accFilt, injectedGyroBias. Kept in
  // a second variable rather than appended to the one above, whose width the
  // offline plotting scripts index into by hand.
  double _logFilter[10] = {0};

  void _readConfig();
  /** \brief One step of the Mahony complementary filter; see params_t. */
  void _filterAttitude(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc,
                       double dt);
  void _applySensorNoise(Eigen::Vector3d& acc, Eigen::Vector3d& gyro);
  void _refreshLogBuffer();
  double _randn() { return _gauss(_rng); }
};

#endif  // MDLORIENTATIONESTIMATOR_HH
