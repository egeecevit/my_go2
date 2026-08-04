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

  Ported from the linear Kalman filter of MIT Cheetah-Software
  (common/src/Controllers/PositionVelocityEstimator.cpp), which is the algorithm
  used on Cheetah 3 and Mini Cheetah.

  With attitude supplied by MdlOrientationEstimator, what remains is linear, so
  this is an ordinary Kalman filter rather than an extended one. The state is

      x = ( r  v  p_1 ... p_4 )

  with the body position r and velocity v and the four foot positions p_i, all
  in the world frame: eighteen states. Prediction is a double integrator driven
  by the accelerometer. Three measurements correct it, all of them expressions
  of the single assumption that a foot on the ground does not move:

    - rows  0-11  the body-to-foot vector, from leg kinematics
    - rows 12-23  the body velocity, from the negated foot velocity
    - rows 24-27  the height of each foot above flat ground

  The velocity rows are what this filter has and its predecessor did not. The
  Bloesch EKF measured only foot *positions* and inferred velocity by
  differentiating through the filter; here J*qdot supplies it directly, which is
  why velocity converges in a handful of cycles rather than integrating up error.

  The other substantive idea is trust. Each leg carries a contact phase in
  [0, 1], and its measurements are weighted by a ramp that rises over the first
  fifth of stance and falls over the last. A foot that has just landed is still
  arriving and one about to lift is already breaking traction; neither is the
  fixed point the measurement model claims. Rather than being included or
  excluded, such a foot has its process and measurement noise inflated by up to
  a hundredfold, so it contributes in proportion to how stationary it plausibly
  is. That is a better match to the physics than a boolean contact flag, and it
  is what the gait schedule is for: an open loop trot knows exactly where in
  stance each foot is.

  One asymmetry in that scheme is worth knowing about. The velocity and height
  rows of an untrusted foot fade all the way to the current estimate, so their
  residuals go to zero and such a foot cannot influence velocity or height at
  all. The body-to-foot rows are deliberately not faded, because they are what
  re-anchors a foot as it touches down. A foot reporting a badly wrong position
  therefore does shift the body estimate sideways, and since nothing here
  observes absolute horizontal position, that shift is permanent. Velocity and
  height, the quantities a controller actually closes a loop around, are
  unaffected. See test_swing_foot_trust_rejection, which measures it.

  Rotation convention: MIT's rBody is world to body, ours is body to world. See
  the note in MdlOrientationEstimator.hh. Anywhere MIT writes Rbod we write _Rbw.

  Beyond the filter this module carries the assessment machinery inherited from
  the estimator it replaces: a MuJoCo overlay, a running comparison against
  simulator ground truth, and per-stance foot slip statistics. All of it reaches
  the simulator through weak symbols and is silently inert on the go1 and robot
  targets.

  Scheduling note: registered one order past MdlOrientationEstimator so it reads
  a fresh attitude from the same cycle, and both run ahead of the behavioral
  controllers that consume the estimate. MdlSimDriver runs last, so in
  simulation the sensor values read here were produced at the end of the
  previous cycle. That is a uniform one step delay affecting the IMU and the
  encoders equally, which the filter absorbs without difficulty.

  Configuration lives in the [posvelestimator] table; see stateestimator.toml.
 */
class MdlPosVelEstimator : public rtcore::Module {
 public:
  // Several of the fixed size members below have byte counts that are multiples
  // of sixteen, which Eigen wants aligned. C++17 makes new honour that on its
  // own and the macro expands to nothing, but it is left in so the requirement
  // survives a future change of standard.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int NUM_LEGS = 4;
  /** \brief State dimension: body position, body velocity, four foot positions. */
  static constexpr int DIM = 18;
  /** \brief Measurement dimension: 12 body-to-foot, 12 velocity, 4 foot height. */
  static constexpr int MEAS = 28;

  /** \brief Distance the robot must travel before drift_percent is meaningful. */
  static constexpr double MIN_DRIFT_DISTANCE = 0.25;  // metres

  /** \brief Contact phase standing for "solidly planted, no schedule known".

    The centre of the trust plateau, so it yields full trust for any trust
    window up to a half. Emphatically not 1.0, which is the *end* of stance and
    yields zero trust. */
  static constexpr double PLANTED_PHASE = 0.5;

  typedef Eigen::Matrix<double, DIM, 1> StateVector;
  typedef Eigen::Matrix<double, DIM, DIM> StateMatrix;

  /** \brief Tunable parameters, settable without a ModuleManager so the filter
      can be exercised from a unit test.

    The six noise entries are MIT's, and are dimensionless multipliers on fixed
    covariance shapes rather than physical densities. Their defaults are the
    values from Cheetah-Software's mini-cheetah-defaults.yaml. */
  struct params_t {
    double dt = 0.001;

    double imu_process_noise_position = 0.02;
    double imu_process_noise_velocity = 0.02;
    double foot_process_noise_position = 0.002;
    double foot_sensor_noise_position = 0.001;
    double foot_sensor_noise_velocity = 0.1;
    double foot_height_sensor_noise = 0.001;

    /** \brief Fraction of stance over which trust ramps in and out.
        Hardcoded at 0.2 in the MIT sources; exposed here because it is a real
        tuning knob and its effect is easy to see on the report line. */
    double trust_window = 0.2;
    /** \brief Noise multiplier applied to a fully untrusted foot, minus one.
        MIT hardcodes 100, so an airborne foot's covariance entries are scaled
        by 101. */
    double high_suspect_number = 100.0;

    /** \brief Height at which a foot in contact sits, in metres.

      Leg kinematics report the centre of the foot sphere, which rests one
      radius above flat ground. This matters more than the corresponding
      parameter did in the filter this replaces. There, absolute height was
      unobservable and this only chose a datum that was never revisited; here
      rows 24-27 are an active height measurement, so a wrong value is a
      permanent, continuously maintained bias in reported body height.

      Set to zero to recover the point-foot form of the MIT sources exactly. */
    double foot_ground_height = 0.022;

    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

    /** \brief Seed the state mean from leg kinematics on the first trusted
        cycle, rather than starting at the origin as MIT does.

      Only the mean is seeded; the covariance stays at its large initial value,
      so a wrong seed is discarded within a few cycles instead of being locked
      in. It buys no steady state accuracy, since the height measurement pulls
      the estimate into place within ten milliseconds regardless. What it buys
      is that the resulting quarter metre lurch does not land in the error
      statistics that the whole assessment layer exists to report. */
    bool warm_start = true;

    /** \brief Take position and velocity from the simulator instead of
        filtering. Simulation only; the "cheater" estimator of the MIT sources,
        reduced to a flag. */
    bool use_ground_truth = false;
  };

  /** \brief Result of comparing the estimate against simulation ground truth. */
  struct comparison_t {
    /** \brief False when no ground truth is available, e.g. on real hardware. */
    bool valid = false;

    // Instantaneous errors, estimate minus truth
    Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_error = Eigen::Vector3d::Zero();
    /** \brief Attitude error as a rotation vector, in roll/pitch/yaw order. */
    Eigen::Vector3d attitude_error = Eigen::Vector3d::Zero();

    // Running RMS since activation, per axis
    Eigen::Vector3d position_rms = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_rms = Eigen::Vector3d::Zero();
    Eigen::Vector3d attitude_rms = Eigen::Vector3d::Zero();

    /** \brief Norm of the current position error, in metres. */
    double position_error_norm = 0.0;
    /** \brief Length of the true body's horizontal path, in metres.

      Horizontal and low pass filtered, both deliberately. Summing the full
      three dimensional displacement every millisecond counts the gait's own
      vertical bounce and lateral sway as distance travelled, which rivals the
      speed the robot is actually making good. The drift figure below divides by
      this, so inflating it flatters the filter. */
    double distance_travelled = 0.0;
    /** \brief Straight line horizontal distance from where the filter started,
        in metres. Together with distance_travelled it separates progress from
        wandering. */
    double net_displacement = 0.0;
    /** \brief Position error as a percentage of distance travelled. Reported as
        zero until the robot has travelled MIN_DRIFT_DISTANCE, below which the
        ratio is dominated by any fixed offset and means nothing. */
    double drift_percent = 0.0;
    /** \brief Seconds since the module was activated. */
    double elapsed = 0.0;

    /** \brief Distance from each foot position estimate to the true one, in
        metres. Simulation only, and zero for legs not in contact. */
    double foothold_error[NUM_LEGS] = {0, 0, 0, 0};

    // Ground truth, for callers that want it directly
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

    The counterpart of setup() in the MIT sources. Must be called after
    setParams(), since the matrices depend on the timestep. */
  void reset();

  /** \brief Runs one filter cycle.

    Separated from update() so the filter can be driven directly, without a
    ModuleManager or any hardware.

    @param Rbw Body to world rotation
    @param aWorld Specific force in the world frame; gravity is added here
    @param omegaBody Angular rate in the body frame
    @param footPos Foot positions in the body frame, from leg kinematics
    @param footVel Foot velocities in the body frame, J*qdot
    @param phase Per-leg contact phase in [0, 1], zero while airborne */
  void step(const Eigen::Matrix3d& Rbw, const Eigen::Vector3d& aWorld,
            const Eigen::Vector3d& omegaBody, const Eigen::Vector3d footPos[NUM_LEGS],
            const Eigen::Vector3d footVel[NUM_LEGS], const double phase[NUM_LEGS]);

  /** \brief How far a foot at this contact phase is to be believed, in [0, 1].

    Rises over the first trust_window of stance and falls over the last, flat at
    one in between. Note that a phase of exactly 1.0 gives *zero* trust: it is
    the instant of liftoff, not the middle of stance. Anything synthesising a
    phase from a boolean contact signal must therefore use PLANTED_PHASE and not
    1.0.

    Static and public so the boundaries can be checked directly by a test. */
  static double trustFromPhase(double phase, double window);

  /** \brief Compares the current estimate against simulation ground truth.

    Returns false, leaving result.valid false, when no ground truth is
    available. Roll and pitch errors are reported alongside, but separately
    from, the yaw error: yaw has no absolute reference here, so folding it into
    a single attitude figure would make a correctly behaving estimate look
    broken. */
  bool compareWithGroundTruth(comparison_t& result) const;

  // Estimate accessors for other modules.
  bool getBodyPosition(Eigen::Vector3d& pos) const;
  bool getBodyVelocity(Eigen::Vector3d& vel) const;
  bool getBodyVelocityInBody(Eigen::Vector3d& vel) const;
  bool getFootPosition(int leg, Eigen::Vector3d& pos) const;

  /** \brief True when leg is currently believed to be on the ground. This is
      the boolean force detector's opinion, which drives the diagnostics; the
      filter itself uses the graded contact phase. */
  bool getContactState(int leg) const;

  /** \brief Trust the filter actually applied to a leg on the last cycle. */
  double getContactTrust(int leg) const;

  /** \brief Normal ground reaction force estimated for a leg, in newtons.

    This is the quantity the contact thresholds are compared against, so it is
    what to look at when tuning them. Both contact sources report a real force:
    "torque" infers it through the leg Jacobian, "simtruth" reads the solver's
    own contact force. */
  double getFootForce(int leg) const;

  /** \brief True once the filter holds a usable estimate. */
  bool isReady() const { return _ready; }

  // Filter internals, exposed for unit testing.
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
  // Averaged between report lines rather than sampled at one. The report period
  // is a whole number of gait cycles for any sensible pair of settings, so an
  // instantaneous reading lands at the same point in the stride every time and
  // says nothing about the rest of it. The same aliasing that the path length
  // filtering further down exists to avoid.
  double _trustSum[NUM_LEGS] = {0, 0, 0, 0};
  long _trustSamples = 0;
  bool _needSeed = true;
  bool _ready = false;

  MdlOrientationEstimator* _orientation = nullptr;
  MdlTrot* _trot = nullptr;
  bool _trotSearched = false;
  QuadrupedKinematics* _kinematics = nullptr;

  // Per-cycle sensor snapshot
  Eigen::Vector3d _jointAngles[NUM_LEGS];
  Eigen::Vector3d _jointVel[NUM_LEGS];
  Eigen::Vector3d _jointTorques[NUM_LEGS];
  Eigen::Vector3d _footPosBody[NUM_LEGS];
  Eigen::Vector3d _footVelBody[NUM_LEGS];
  Eigen::Matrix3d _footJacobian[NUM_LEGS];
  double _contactPhase[NUM_LEGS] = {0, 0, 0, 0};

  bool _contacts[NUM_LEGS] = {false, false, false, false};
  int _contactCount[NUM_LEGS] = {0, 0, 0, 0};
  double _footForce[NUM_LEGS] = {0, 0, 0, 0};

  double _dt = 0.001;
  double _startTime = 0.0;

  // -- Configuration --------------------------------------------------------
  bool _enable = true;

  // Where the filter's contact phase comes from. "gait" reads MdlTrot's
  // schedule, which is exact while it applies because the gait is open loop.
  // "force" derives it from the boolean detector below. "auto" prefers the gait
  // and falls back to the detector.
  std::string _phaseSource = "auto";

  // Contact detection. "torque" estimates ground reaction force from joint
  // torques and works on any target; "simtruth" reads the solver's own contact
  // force out of MuJoCo, which is useful for separating filter error from
  // detector error. Both produce a normal force in newtons and both then latch
  // through the same thresholds.
  std::string _contactSource = "torque";
  double _contactOnThreshold = 30.0;
  double _contactOffThreshold = 15.0;
  int _contactDebounce = 3;
  double _jacobianDamping = 0.01;
  /** \brief Sign relating joint torque to ground reaction force, tau = -s J^T f.
    Whether the measured torque opposes or follows the contact force depends on
    the actuator sign convention of the target, so it is left configurable
    rather than hard coded. A standing robot must yield a positive normal force;
    if contacts never latch, invert this. */
  double _contactForceSign = 1.0;

  // Visualization
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

  // Ground truth comparison
  bool _truthEnable = true;
  double _truthReportPeriod = 2.0;
  double _lastReportTime = 0.0;
  // Running sums of squared error, for the RMS figures
  Eigen::Vector3d _sumSqPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d _sumSqVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d _sumSqAtt = Eigen::Vector3d::Zero();
  long _errorSamples = 0;

  // Path length accumulator; see the note on comparison_t::distance_travelled.
  //
  // The body is low pass filtered before any of this, because a gait moves the
  // trunk in a circle as well as forwards. Decimating the raw signal does not
  // help: sampling a 2Hz sway every 50ms just aliases it, and sampling it at
  // exactly the gait period cancels it only by luck of phase. Two poles at a
  // time constant of about one gait cycle removes the oscillation whatever its
  // phase, then the sampling interval below keeps each increment comfortably
  // above the simulator's numerical noise.
  double _pathSamplePeriod = 0.05;  // [s]
  double _pathFilterTau = 0.5;      // [s]
  double _distanceTravelled = 0.0;
  Eigen::Vector3d _pathFilt1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d _pathFilt2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d _lastPathSample = Eigen::Vector3d::Zero();
  Eigen::Vector3d _startTruePos = Eigen::Vector3d::Zero();
  double _lastPathTime = 0.0;
  bool _havePathSample = false;

  // -- Error budget diagnostics (simulation only) ---------------------------
  //
  // The filter's measurement model asserts that a foot in contact is a fixed
  // point in the world. Every millimetre a stance foot actually travels is
  // model error the filter has no way to see, and it is not zero mean, so it
  // biases velocity and integrates into position. Measuring it says how much of
  // the residual error is the filter's and how much is the gait's.
  bool _haveFootTruth = false;
  Eigen::Vector3d _footTruePos[NUM_LEGS];
  Eigen::Vector3d _stanceStartPos[NUM_LEGS];
  bool _slipTracking[NUM_LEGS] = {false, false, false, false};
  double _slipSum[NUM_LEGS] = {0, 0, 0, 0};
  double _slipMax[NUM_LEGS] = {0, 0, 0, 0};
  long _stanceCount[NUM_LEGS] = {0, 0, 0, 0};
  // Sum of squared slip per body axis, pooled over all legs, and the total
  // stance time it accumulated over.
  Eigen::Vector3d _slipSumSqAxis = Eigen::Vector3d::Zero();
  double _stanceTimeTotal = 0.0;
  double _stanceStartTime[NUM_LEGS] = {0, 0, 0, 0};
  // How often the configured detector agrees with the simulator's own contact
  // state. Low agreement makes every other number on the report suspect.
  long _contactAgree = 0;
  long _contactSamples = 0;

  // Logging buffers, refreshed each cycle and registered with the LogServer.
  rtcore::LogServer* _logserver = nullptr;
  double _logState[9] = {0};       // position, vWorld, vBody
  double _logFootholds[12] = {0};  // foot positions, world frame
  // pos, vel and attitude errors, then drift percent, path length and net
  // displacement.
  double _logError[12] = {0};
  double _logCov[DIM] = {0};  // diagonal of P
  // Four contact trusts followed by the four estimated normal forces, so a
  // recorded run carries what is needed to re-tune the contact thresholds.
  double _logContacts[8] = {0};
  // Per-leg foot position error followed by per-leg mean slip per stance, the
  // two halves of the error budget above.
  double _logSlip[8] = {0};

  // Comparison against ground truth, recomputed once per cycle and reused by
  // the statistics, the log buffers and the visualization.
  comparison_t _comparison;

  void _readConfig();
  void _readSensors();
  void _readContactPhase();
  void _detectContacts();
  /** \brief Applies the Schmitt trigger and debounce to one leg's normal force.
      Both contact sources feed through here, so they latch alike. */
  void _latchContact(int leg, double normalForce);
  void _seedFromKinematics(const Eigen::Matrix3d& Rbw,
                           const Eigen::Vector3d footPos[NUM_LEGS],
                           const double phase[NUM_LEGS]);
  void _updateSlipStats();
  void _updateVisualization();
  void _updateGroundTruthStats();
  void _refreshLogBuffers();
};

#endif  // MDLPOSVELESTIMATOR_HH
