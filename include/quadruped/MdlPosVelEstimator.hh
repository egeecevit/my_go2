/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef MDLPOSVELESTIMATOR_HH
#define MDLPOSVELESTIMATOR_HH

#include <Eigen/Dense>
#include <string>

#include "rtcore/Module.hh"

#define POSVELMODULE_NAME "MdlPosVelEstimator"

class QuadrupedKinematics;
class MdlOrientationEstimator;
class MdlTrot;

namespace rtcore {
class LogServer;
}

/** \brief Second stage of the state estimator: body position and velocity.

  With attitude supplied by MdlOrientationEstimator, this linear Kalman filter
  estimates the world-frame state

      x = ( r  v  p_1 ... p_4 )

  containing body position r, body velocity v, and four foot positions. The
  prediction is a double integrator driven by world-frame specific force plus
  gravity. Measurement rows are:

    - rows  0-11  relative body/foot positions from leg kinematics
    - rows 12-23  body velocity from negated foot velocity, plus the rolling
                  of the foot sphere with its calf (radius foot_ground_height)
    - rows 24-27  world height of each foot above the ground datum

  Each leg's contact phase in [0, 1] sets a trust weight. Trust ramps at both
  stance boundaries; reduced trust inflates that leg's process and measurement
  noise. Position rows remain active to re-anchor a foot at touchdown, while
  velocity and height rows fade toward the current estimate as trust falls.
  Outside an active gait schedule, callers use PLANTED_PHASE for fully planted
  legs.

  Rbw is the body-to-world rotation. Optional simulator ground truth and
  visualization use weak hooks and remain inactive when those hooks are absent.
  Shadow mode (simulation only): with use_ground_truth the filter keeps running
  and is what the logs record, while the accessors hand consumers the
  simulator's position and velocity.
  The module runs after the orientation estimator and before behavioral
  controllers, so consumers see the latest available attitude and estimate.

  Configuration lives in the [posvelestimator] table; see stateestimator.toml.
 */
class MdlPosVelEstimator : public rtcore::Module {
 public:
  // Keep fixed-size Eigen members aligned for vectorized allocations.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int NUM_LEGS = 4;
  /** \brief State dimension: body position, body velocity, four foot positions. */
  static constexpr int DIM = 18;
  /** \brief Measurement dimension: 12 body-to-foot, 12 velocity, 4 foot height. */
  static constexpr int MEAS = 28;

  /** \brief Minimum horizontal path length before drift_percent is reported. */
  static constexpr double MIN_DRIFT_DISTANCE = 0.25;  // metres

  /** \brief Contact phase representing fully planted when no schedule is known.
      This lies at the center of the full-trust plateau. */
  static constexpr double PLANTED_PHASE = 0.5;

  typedef Eigen::Matrix<double, DIM, 1> StateVector;
  typedef Eigen::Matrix<double, DIM, DIM> StateMatrix;

  /** \brief Tunable filter parameters.

    Noise entries are dimensionless multipliers on fixed covariance templates. */
  struct params_t {
    double dt = 0.001;

    double imu_process_noise_position = 0.02;
    double imu_process_noise_velocity = 0.02;
    double foot_process_noise_position = 0.002;
    double foot_sensor_noise_position = 0.001;
    double foot_sensor_noise_velocity = 0.1;
    double foot_height_sensor_noise = 0.001;

    /** \brief Fraction of stance over which trust ramps up after touchdown. */
    double trust_window = 0.5;
    /** \brief Fraction of stance over which trust ramps down before liftoff. */
    double trust_window_liftoff = 0.1;
    /** \brief Additional covariance scale at zero trust; total scale is 1 plus
        this value. */
    double high_suspect_number = 1000.0;

    /** \brief World height of a contacting foot center [m]; on flat ground
        this is the foot-sphere radius, which also sets how fast the centre
        rolls with the calf. Zero gives a point foot. */
    double foot_ground_height = 0.022;

    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

    /** \brief Seed the state mean from leg kinematics on the first trusted cycle.
        The covariance remains at its initialized value. */
    bool warm_start = true;

    /** \brief Publish simulator position and velocity to consumers while the
        filter keeps running. Shadow mode, simulation only. */
    bool use_ground_truth = false;

    /** \brief Take contact trust from the simulator's foot forces instead of
        the gait schedule: a diagnostic tier that separates contact-timing error
        from filter error. Simulation only. */
    bool use_contact_oracle = false;
  };

  /** \brief Result of comparing the estimate with optional simulator truth. */
  struct comparison_t {
    /** \brief False when simulator truth is unavailable. */
    bool valid = false;

    // Instantaneous errors, estimate minus truth. Position is in the world
    // frame; velocity is each estimate's own body-frame velocity minus the
    // true body-frame velocity, because heading is unobservable and a
    // world-frame velocity difference grows with yaw drift.
    Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_error = Eigen::Vector3d::Zero();

    // Running per-axis RMS since activation.
    Eigen::Vector3d position_rms = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_rms = Eigen::Vector3d::Zero();

    /** \brief Norm of the current position error, in metres. */
    double position_error_norm = 0.0;
    /** \brief Low-pass filtered horizontal path length of the true body [m]. */
    double distance_travelled = 0.0;
    /** \brief Horizontal displacement from the true-body start position [m]. */
    double net_displacement = 0.0;
    /** \brief Position error as a percentage of distance travelled, or zero
        below MIN_DRIFT_DISTANCE. */
    double drift_percent = 0.0;
    /** \brief Seconds since the module was activated. */
    double elapsed = 0.0;

    /** \brief Distance from each estimated foot position to simulator truth [m].
        Simulation only; negative means unavailable or untrusted. */
    double foothold_error[NUM_LEGS] = {-1.0, -1.0, -1.0, -1.0};

    // Ground-truth state for callers that need it directly.
    Eigen::Vector3d true_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d true_velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond true_orientation = Eigen::Quaterniond::Identity();
  };

  MdlPosVelEstimator();
  ~MdlPosVelEstimator();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  void setParams(const params_t& params);
  const params_t& getParams() const { return _params; }

  /** \brief Builds the filter matrices and clears the state.
      Call after setParams(), since the matrices depend on the timestep. */
  void reset();

  /** \brief Runs one filter cycle without ModuleManager or hardware access.

    @param Rbw Body-to-world rotation
    @param aWorld Specific force in the world frame; gravity is added here
    @param omegaBody Angular rate in the body frame
    @param footPos Foot positions in the body frame, from leg kinematics
    @param footVel Foot velocities in the body frame, J*qdot
    @param omegaCalf Angular velocity of each calf relative to the body, body frame
    @param phase Per-leg contact phase in [0, 1], zero while airborne */
  void step(const Eigen::Matrix3d& Rbw, const Eigen::Vector3d& aWorld,
            const Eigen::Vector3d& omegaBody, const Eigen::Vector3d footPos[NUM_LEGS],
            const Eigen::Vector3d footVel[NUM_LEGS], const Eigen::Vector3d omegaCalf[NUM_LEGS],
            const double phase[NUM_LEGS]);

  /** \brief Trust weight for a contact phase, in [0, 1].

    Trust ramps over windowTouchdown after touchdown and over windowLiftoff
    before liftoff and is one between them. A phase of exactly 1.0 is the
    liftoff boundary and has zero trust. */
  static double trustFromPhase(double phase, double windowTouchdown, double windowLiftoff);
  /** \brief Symmetric ramp: the same window at both boundaries. */
  static double trustFromPhase(double phase, double window) {
    return trustFromPhase(phase, window, window);
  }

  /** \brief Compares the current estimate with simulator ground truth.
      Returns false and leaves result.valid false when truth is unavailable. */
  bool compareWithGroundTruth(comparison_t& result) const;

  /** \brief Shadow mode: publish this position and velocity to consumers while
      the filter keeps running. update() calls it with simulator truth; public
      so tests can drive it without a simulator. */
  void setPublishedTruth(const Eigen::Vector3d& pos, const Eigen::Vector3d& velWorld,
                         const Eigen::Quaterniond& qTrue);

  // Estimate accessors for other modules.
  bool getBodyPosition(Eigen::Vector3d& pos) const;
  bool getBodyVelocity(Eigen::Vector3d& vel) const;
  bool getBodyVelocityInBody(Eigen::Vector3d& vel) const;
  bool getFootPosition(int leg, Eigen::Vector3d& pos) const;

  /** \brief Trust the filter actually applied to a leg on the last cycle. */
  double getContactTrust(int leg) const;

  /** \brief True once the filter holds a usable estimate. */
  bool isReady() const { return _ready; }

  // Filter internals, exposed for diagnostics.
  const StateVector& getState() const { return _xhat; }
  const StateMatrix& getCovariance() const { return _P; }
  const Eigen::Matrix<double, DIM, DIM>& getA() const { return _A; }
  const Eigen::Matrix<double, DIM, 3>& getB() const { return _B; }
  const Eigen::Matrix<double, MEAS, DIM>& getC() const { return _C; }
  const Eigen::Matrix<double, DIM, DIM>& getQ0() const { return _Q0; }
  const Eigen::Matrix<double, MEAS, MEAS>& getR0() const { return _R0; }

 private:
  params_t _params;

  // Filter state and the fixed model matrices.
  StateVector _xhat = StateVector::Zero();
  StateMatrix _P = StateMatrix::Zero();
  Eigen::Matrix<double, DIM, DIM> _A = Eigen::Matrix<double, DIM, DIM>::Zero();
  Eigen::Matrix<double, DIM, 3> _B = Eigen::Matrix<double, DIM, 3>::Zero();
  Eigen::Matrix<double, MEAS, DIM> _C = Eigen::Matrix<double, MEAS, DIM>::Zero();
  Eigen::Matrix<double, DIM, DIM> _Q0 = Eigen::Matrix<double, DIM, DIM>::Zero();
  Eigen::Matrix<double, MEAS, MEAS> _R0 = Eigen::Matrix<double, MEAS, MEAS>::Zero();

  Eigen::Matrix3d _Rbw = Eigen::Matrix3d::Identity();
  Eigen::Vector3d _vBody = Eigen::Vector3d::Zero();
  double _trust[NUM_LEGS] = {0, 0, 0, 0};
  // Per-leg trust sums used for interval averages.
  double _trustSum[NUM_LEGS] = {0, 0, 0, 0};
  long _trustSamples = 0;
  bool _needSeed = true;
  bool _ready = false;

  // Shadow mode: simulator truth handed to consumers while the filter above
  // keeps running. See setPublishedTruth().
  Eigen::Vector3d _truthPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d _truthVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d _truthVBody = Eigen::Vector3d::Zero();
  bool _haveTruth = false;

  // Innovation diagnostics over fully trusted legs, refreshed by step().
  Eigen::Vector3d _innovVel = Eigen::Vector3d::Zero();  // body frame
  double _innovZ = 0.0;
  double _nis = 0.0;
  int _nTrusted = 0;

  // Horizontal world-position datum retained across activations; zero initially.
  Eigen::Vector2d _originCarry = Eigen::Vector2d::Zero();

  MdlOrientationEstimator* _orientation = nullptr;
  MdlTrot* _trot = nullptr;
  bool _trotSearched = false;
  QuadrupedKinematics* _kinematics = nullptr;

  // Per-cycle sensor snapshot, in body-frame coordinates.
  Eigen::Vector3d _jointAngles[NUM_LEGS];
  Eigen::Vector3d _jointVel[NUM_LEGS];
  Eigen::Vector3d _footPosBody[NUM_LEGS];
  Eigen::Vector3d _footVelBody[NUM_LEGS];
  Eigen::Vector3d _calfOmegaBody[NUM_LEGS];
  Eigen::Matrix3d _footJacobian[NUM_LEGS];
  double _contactPhase[NUM_LEGS] = {0, 0, 0, 0};
  bool _oracleContact[NUM_LEGS] = {false, false, false, false};

  double _dt = 0.001;
  double _startTime = 0.0;

  // -- Configuration --------------------------------------------------------
  bool _enable = true;

  // Visualization state.
  bool _vizEnable = true;
  bool _vizTrail = true;
  int _vizTrailLength = 200;
  int _vizTrailDecimation = 20;
  double _vizMarkerSize = 0.03;

  static constexpr int MAX_TRAIL = 512;
  Eigen::Vector3d _trailEst[MAX_TRAIL];
  Eigen::Vector3d _trailTrue[MAX_TRAIL];
  int _trailCount = 0;
  int _trailHead = 0;
  int _trailTick = 0;

  // Optional ground-truth comparison state.
  bool _truthEnable = true;
  double _truthReportPeriod = 2.0;
  double _lastReportTime = 0.0;
  // Running sums of squared position and velocity error.
  Eigen::Vector3d _sumSqPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d _sumSqVel = Eigen::Vector3d::Zero();
  long _errorSamples = 0;

  // Low-pass filtered horizontal path accumulator and sampling state.
  double _pathSamplePeriod = 0.05;  // [s]
  double _pathFilterTau = 0.5;      // [s]
  double _distanceTravelled = 0.0;
  Eigen::Vector3d _pathFilt1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d _pathFilt2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d _lastPathSample = Eigen::Vector3d::Zero();
  Eigen::Vector3d _startTruePos = Eigen::Vector3d::Zero();
  double _lastPathTime = 0.0;
  bool _havePathSample = false;

  // Optional simulator foot positions, in the world frame.
  bool _haveFootTruth = false;
  Eigen::Vector3d _footTruePos[NUM_LEGS];
  // Sums and counts for trusted foothold-error samples.
  double _footholdErrSum[NUM_LEGS] = {0, 0, 0, 0};
  long _footholdErrCount[NUM_LEGS] = {0, 0, 0, 0};

  // Logging buffers, refreshed each cycle and registered with LogServer.
  rtcore::LogServer* _logserver = nullptr;
  double _logState[9] = {0};       // position, vWorld, vBody
  double _logFootholds[12] = {0};  // foot positions, world frame
  // Position/velocity error, drift percentage, path length, and displacement.
  double _logError[9] = {0};
  double _logCov[DIM] = {0};      // diagonal of P
  double _logContacts[4] = {0};   // per-leg contact trust
  double _logFootErr[4] = {0};    // per-leg foothold error, negative if untrusted
  // Simulator foot positions in the world frame; simulation only.
  double _logFootTruth[12] = {0};
  // Mean velocity innovation (body), height innovation, NIS, trusted-leg count.
  double _logInnov[6] = {0};

  // Comparison result reused by statistics, logging, and visualization.
  comparison_t _comparison;

  void _readConfig();
  void _readSensors();
  void _readContactPhase();
  void _readFootTruth();
  void _seedFromKinematics(const Eigen::Matrix3d& Rbw,
                           const Eigen::Vector3d footPos[NUM_LEGS],
                           const double phase[NUM_LEGS]);
  void _updateVisualization();
  void _updateGroundTruthStats();
  void _refreshLogBuffers();
};

#endif  // MDLPOSVELESTIMATOR_HH
