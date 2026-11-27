/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef MDLTROT_HH
#define MDLTROT_HH

#include <Eigen/Dense>

#include "rtcore/ClockHW.hh"
#include "rtcore/Module.hh"

#define TROTMODULE_NAME "MdlTrot"

#include "quadruped/MdlConvexMPC.hh"
#include "quadruped/QuadrupedWBC.hh"

class MdlLegControl;
class MdlOrientationEstimator;
class MdlPosVelEstimator;
class QuadrupedKinematics;

/** Integer-clock schedule for MPC solve deadlines.

  Deadlines use integer microseconds. Contact changes may trigger an early solve;
  that event starts a fresh period. */
class PeriodicSolveSchedule {
 public:
  bool reset(rtcore::CLOCK now, double period_seconds);
  bool due(rtcore::CLOCK now) const { return _valid && now >= _next; }
  void consumed(rtcore::CLOCK now, bool asynchronous);

  rtcore::CLOCK period() const { return _period; }
  rtcore::CLOCK next() const { return _next; }

 private:
  bool _valid = false;
  rtcore::CLOCK _period = 0;
  rtcore::CLOCK _next = 0;
};

/** One immutable, acceleration-continuous swing segment in body coordinates.

  Reset once at liftoff and sample until touchdown. The endpoint states remain
  fixed while the segment is active. */
class SwingTrajectory {
 public:
  bool reset(const Eigen::Vector3d& position0, const Eigen::Vector3d& velocity0,
             const Eigen::Vector3d& position1, const Eigen::Vector3d& velocity1,
             double duration, double height);

  /** Sample with a possibly time-varying clearance scale.

    scale_dot and scale_ddot preserve the derivative relationship between the
    returned position, velocity, and acceleration while clearance changes. */
  void sample(double time, double scale, double scale_dot, double scale_ddot,
              Eigen::Vector3d& position, Eigen::Vector3d& velocity,
              Eigen::Vector3d& acceleration) const;

  bool isValid() const { return _valid; }
  double duration() const { return _duration; }
  const Eigen::Vector3d& endPosition() const { return _position1; }
  const Eigen::Vector3d& endVelocity() const { return _velocity1; }

 private:
  bool _valid = false;
  double _duration = 0.0;
  double _height = 0.0;
  Eigen::Vector3d _position1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d _velocity1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d _coefficient[6] = {Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero()};
};

/** \brief Foot trajectory scheduler for a periodic diagonal gait.

  Pure geometry with no hardware or framework dependency; MdlTrot supplies the
  nominal footprint. Given a leg, time, and planted-foot velocity u in body
  coordinates, it returns the body-frame offset and velocity. Phase runs in
  [0, 1); stance is phase < duty with constant velocity u, while swing uses a
  cubic Hermite segment with matching end tangents and a raised-cosine lift.
  Position and velocity are continuous at both transitions. Phase offsets select
  the gait pattern.
 */
struct TrotGait {
  /** \brief Leg count, indexed as QuadrupedKinematics::LegIndex. */
  static constexpr int NUM_LEGS = 4;

  struct params_t {
    /** \brief Duration of one full gait cycle [s]. */
    double period = 0.5;
    /** \brief Fraction of the cycle a leg spends in stance. Values at least
        0.5 make diagonal support intervals overlap. */
    double duty = 0.5;
    /** \brief Peak foot clearance at mid swing [m]. */
    double swing_height = 0.05;
    /** \brief Phase offset per leg, in cycles. Trot is {0, 0.5, 0.5, 0} for
        FL, FR, RL, RR, which puts the diagonals in step with each other. */
    double phase_offset[NUM_LEGS] = {0.0, 0.5, 0.5, 0.0};
  };

  /** \brief Installs parameters, clamping any that would make the schedule
      degenerate. */
  void setParams(const params_t& params);
  const params_t& getParams() const { return _params; }

  /** \brief Phase of a leg in [0, 1). Wraps for any real t, negative included. */
  double legPhase(int leg, double t) const;

  /** \brief True while the leg is scheduled to be on the ground. */
  bool inStance(int leg, double t) const;

  /** \brief Progress through stance in [0, 1); returns 0 while swinging and
      at the instant stance begins. */
  double stanceProgress(int leg, double t) const;

  /** \brief Foot offset from the nominal footprint and its time derivative.
      \param u Stance velocity, the foot's body frame velocity while planted. */
  void sample(int leg, double t, const Eigen::Vector3d& u, Eigen::Vector3d& dp,
              Eigen::Vector3d& dv) const;

 private:
  params_t _params;
};

/** \brief Behavioral module that walks the robot with a diagonal trot, with
    stance carried by a convex MPC ground reaction force.

  MdlTrot owns state transitions, gait timing, contact timing, body references,
  and swing references. MdlConvexMPC supplies world-frame ground reaction
  forces; MdlLegControl converts those forces and swing trajectories to motor
  torques. Swing uses a frozen body-frame C2 segment; stance uses zero Cartesian
  gains and the MPC force rotated into the body frame.

  WAIT and PREP hold or blend a pose, TROT runs swing/stance control, STOPPING
  ramps speed and clearance down, and CENTERING returns to the nominal footprint.
  Estimator, MPC, and command failures transition to ERROR; stance has no
  position-control fallback.

  With [trot] wbc = true, TROT and STOPPING command the legs through
  whole-body impulse control instead (Kim, Di Carlo, Katz, Bledt, Kim 2019):
  the MPC's first-step forces, the body reference and the swing samples above
  become a task hierarchy and WBIC QP (QuadrupedWBC), and every joint gets the
  WBIC torque plus a joint PD on the hierarchy's joint references. A tick the
  WBC cannot solve falls back to the per-leg commands. See wbcSolve().

  Configuration lives in the [trot] table; see trot.toml, and in [mpc] for the
  solver; see mpc.toml.
 */
class MdlTrot : public rtcore::Module {
 public:
  static constexpr int NUM_LEGS = TrotGait::NUM_LEGS;

  /** \brief SETTLED means centering finished and motion has come to rest. */
  enum Status { IDLE, ACTIVE, SETTLED, ERROR };

  MdlTrot();
  ~MdlTrot();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  /** \brief Asks the module to stop walking and bring the feet back to the
      nominal footprint with zero velocity. Safe to call from any state. */
  void stopTrotting();

  /** \brief True once centering is complete and motion has come to rest. */
  bool isStopped() const { return _state == _state_t::DONE; }

  Status getStatus() const { return _status; }

  /** \brief Per-leg stance progress for the state estimator.

    Returns false and leaves phase untouched outside TROT and STOPPING. False
    means no gait schedule is active; the caller should treat the robot as fully
    planted. Values are recomputed from the gait datum rather than cached state. */
  bool getStancePhase(double phase[NUM_LEGS]) const;

  /** \brief Horizontal body-frame touchdown correction from velocity error and
      optional yaw-rate skew. Both input z components are ignored.

    centrifugalEnable disables the yaw-rate term. */
  static Eigen::Vector3d touchdownFeedback(const Eigen::Vector3d& vMeasured,
                                           const Eigen::Vector3d& vCommanded,
                                           double raibertGain, double omega_z,
                                           double height, double gravity,
                                           bool centrifugalEnable);

  /** \brief Task gains of the whole-body trot: stiffness [1/s^2] and damping
      [1/s] of each task's acceleration command (Kim et al. 2019, Table I). */
  struct wbc_gains_t {
    double kp_ori = 100.0, kd_ori = 10.0;
    double kp_pos = 100.0, kd_pos = 10.0;
    double kp_foot = 100.0, kd_foot = 10.0;
  };

  /** \brief One whole-body tick's input, free of the module framework. */
  struct wbc_input_t {
    /** Model terms at `state`; must outlive the call. */
    const QuadrupedDynamics::terms_t* dyn = nullptr;
    /** Measured R, base-origin velocity (world axes), body rate (body axes)
        and joints, in QuadrupedDynamics' layout. */
    QuadrupedDynamics::state_t state;
    /** Measured base origin, world [m]. */
    Eigen::Vector3d p_base = Eigen::Vector3d::Zero();
    /** Scheduled contacts and the MPC's ground reaction on the robot, world
        axes [N]; f_des is read for stance legs only. */
    bool stance[NUM_LEGS] = {true, true, true, true};
    Eigen::Vector3d f_des[NUM_LEGS];
    /** Body reference, reference acceleration zero (Kim): attitude, world
        angular velocity, base-origin position and velocity, world axes. */
    Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity();
    Eigen::Vector3d w_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_des = Eigen::Vector3d::Zero();
    /** Swing reference in body coordinates: foot centre from the base origin
        and its first two derivatives taken in the body frame. Read for swing
        legs only. */
    Eigen::Vector3d foot_p[NUM_LEGS], foot_v[NUM_LEGS], foot_a[NUM_LEGS];
    wbc_gains_t gains;
  };

  struct wbc_output_t {
    /** The problem handed to QuadrupedWBC. Tasks in Kim's order: body
        orientation, base position, then one per swing foot in leg order. */
    QuadrupedWBC::input_t problem;
    QuadrupedWBC::output_t result;
  };

  /** \brief Builds the Kim task set from the scheduled contacts and solves it.

    Contacts are the stance legs' rolling contact points with the MPC forces
    as F_des. Each swing foot is a task on its full 3x18 Jacobian, and both
    stages leave the swing joints the leg-relative error e = p_B_des - p_B
    the footstep law was tuned for. Kinematic stage: the reference is the
    body-frame sample carried on the desired body, so after the body levels
    the joint references are q + J_leg^-1 e and J_leg^-1 v_B_des. Acceleration
    stage: the command is the absolute acceleration of the body-frame sample
    when the body moves as the two body tasks command, so the body-frame error
    obeys e'' + kd e' + kp e = 0. In a flight phase (duty below 0.5) all four
    feet are swing tasks: the QP lets the base fall freely and the legs keep
    following their swing references in the body frame. False, with out not
    meaningful, when QuadrupedWBC::solve fails. */
  static bool wbcSolve(QuadrupedWBC& wbc, const wbc_input_t& in, wbc_output_t& out);

 private:
  enum class _state_t { WAIT, PREP, TROT, STOPPING, CENTERING, DONE };

  MdlLegControl* _legs[NUM_LEGS] = {};
  QuadrupedKinematics* _kinematics = nullptr;
  MdlOrientationEstimator* _orientation = nullptr;
  MdlPosVelEstimator* _posvel = nullptr;
  MdlConvexMPC* _mpc = nullptr;
  TrotGait _gait;

  _state_t _state = _state_t::WAIT;
  Status _status = IDLE;

  // Current per-leg reference trajectory, in the body frame.
  Eigen::Vector3d _footpos[NUM_LEGS];
  Eigen::Vector3d _footvel[NUM_LEGS];
  Eigen::Vector3d _footacc[NUM_LEGS];
  bool _stance[NUM_LEGS] = {true, true, true, true};

  SwingTrajectory _swingTrajectory[NUM_LEGS];
  double _swingStart[NUM_LEGS] = {0};
  double _swingEnd[NUM_LEGS] = {0};

  // Pose captured at activation and the PREP blend target.
  Eigen::Vector3d _footpos_start[NUM_LEGS];
  Eigen::Vector3d _footpos_end[NUM_LEGS];

  // Snapshot at stop time; CENTERING uses the position and velocity as its
  // cubic-Hermite initial state.
  Eigen::Vector3d _center_pos0[NUM_LEGS];
  Eigen::Vector3d _center_vel0[NUM_LEGS];
  Eigen::Vector3d _center_pos1[NUM_LEGS];

  double _mark = 0.0;       // entry time of the current state
  double _trot_mark = 0.0;  // entry time of TROT, the gait phase datum
  double _stop_mark = 0.0;  // entry time of STOPPING, the ramp-down datum

  // -- Command ramp ----------------------------------------------------------
  //
  // _speedScale scales commanded twist; _liftScale scales swing clearance.
  // Clearance stays full during ramp-up and fades with speed during ramp-down.
  double _speedScale = 0.0;
  double _liftScale = 1.0;
  double _liftScaleDot = 0.0;  // d/dt, for the swing velocity's product rule
  double _liftScaleDDot = 0.0;
  // Speed scale captured when STOPPING begins.
  double _stopSpeedScale0 = 0.0;

  int _cmdFailures[NUM_LEGS] = {0, 0, 0, 0};

  // Logged ramp scales: speed and lift.
  rtcore::LogServer* _logserver = nullptr;
  double _logScales[2] = {0};  // speed scale, lift scale
  double _logFootReference[NUM_LEGS * 9] = {0};  // per-leg p, v, a
  double _logContact[NUM_LEGS] = {0};
  double _logTorqueScale[NUM_LEGS] = {1, 1, 1, 1};
  // Requested torque before the per-leg saturation scale, in the same
  // leg-major layout as MotorHW. With [trot] wbc: WBIC torque plus joint PD at
  // the measured state, before a per-joint clip; torquescale is then the
  // uniform scale the per-leg path would have applied.
  double _logTorqueRequest[NUM_LEGS * 3] = {0};
  // Body reference and ramp/clamp state, laid out as:
  //   [0] _desPos.x()      [4] desVel_world.x()   [8]  _yawUnwrapped
  //   [1] _desPos.y()      [5] desVel_world.y()   [9]  _speedScale
  //   [2] _desPos.z()      [6] desVel_world.z()   [10] _liftScale
  //   [3] _desYaw          [7] _yawRateCommand()  [11] _yawClamp
  // Entries are world position/velocity, desired yaw/yaw rate, unwrapped yaw,
  // ramp scales, and the yaw clamp.
  double _logBodyReference[12] = {0};

  // -- Measured body state, refreshed once per TROT cycle --------------------
  //
  // COM state in world coordinates; the position estimator reports the body
  // origin and _readBodyState converts it using the configured COM offset.
  Eigen::Matrix3d _Rbw = Eigen::Matrix3d::Identity();
  Eigen::Vector3d _comPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d _comVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d _rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d _omegaWorld = Eigen::Vector3d::Zero();
  bool _stateValid = false;

  // Yaw carried on a continuous branch rather than wrapped into (-pi, pi]. The
  // QP sees the difference between the current and reference yaw directly, so a
  // wrap between the two would read as a full turn of heading error.
  double _yawUnwrapped = 0.0;
  double _yawWrapped = 0.0;  // last raw estimate, for the unwrap increment
  bool _haveYaw = false;

  // -- Body reference, integrated at the behavior rate -----------------------
  Eigen::Vector3d _desPos = Eigen::Vector3d::Zero();  // world x, y and hold height
  double _desYaw = 0.0;

  // Filtered body-frame velocity used as a bounded correction to the commanded
  // stance sweep and touchdown geometry.
  Eigen::Vector3d _vfilt = Eigen::Vector3d::Zero();
  bool _vfiltValid = false;
  double _stanceVelocityFeedback = 0.25;

  // Raibert touchdown feedback gain and the centrifugal term's on/off switch.
  // See touchdownFeedback() and its call in _startSwing.
  double _raibertGain = 0.03;
  bool _centrifugalTerm = true;

  // -- MPC cadence and result ------------------------------------------------
  MdlConvexMPC::output_t _mpcOut;
  bool _mpcHave = false;   // a valid force has been solved for at least once
  PeriodicSolveSchedule _mpcSchedule;
  int _mpcFailures = 0;    // consecutive failed solves
  bool _mpcMask[NUM_LEGS] = {true, true, true, true};  // contact mask at that solve

  // -- Configuration, all from the [trot] table

  Eigen::Vector3d _vcmd = Eigen::Vector3d(0.15, 0.0, 0.0);
  double _yawRate = 0.0;
  double _velocityFilterTau = 0.1;  // [s]

  double _origin[3] = {0.0, 0.10, -0.28};

  double _wait_duration = 0.3;
  double _prep_duration = 1.5;
  double _rampup_duration = 2.0;
  double _rampdown_duration = 1.0;
  double _centering_duration = 1.5;

  // Joint-space gains, used only by the states that hold a pose through IK.
  Eigen::Vector3d _prep_kp = Eigen::Vector3d::Constant(150.0);
  Eigen::Vector3d _prep_kd = Eigen::Vector3d::Constant(4.0);

  // Cartesian swing gains [N/m] and [N/(m/s)]; joint damping is applied to every
  // TROT command.
  Eigen::Vector3d _swing_kp = Eigen::Vector3d::Constant(500.0);
  Eigen::Vector3d _swing_kd = Eigen::Vector3d::Constant(8.0);
  Eigen::Vector3d _swingNaturalFrequency = Eigen::Vector3d::Constant(28.0);
  Eigen::Vector3d _swingDampingRatio = Eigen::Vector3d::Constant(0.8);
  bool _swingInverseDynamics = true;
  double _swingFeedforwardScale = 0.0;
  double _jointDamping = 0.2;

  double _positionClamp = 0.1;  // [m]
  double _yawClamp = 0.2;       // [rad]
  double _trackingErrorLimit = 0.5;
  int _cmdFailureLimit = 3;
  int _mpcFailureLimit = 3;

  // -- Whole-body impulse control, [trot] wbc --------------------------------
  //
  // Go2 only: the floating-base model carries the Go2 link inertials, so for
  // any other [hardware] robot_model init() warns and leaves _wbc false.
  bool _wbc = false;
  QuadrupedDynamics* _wbModel = nullptr;
  QuadrupedWBC* _wbcSolver = nullptr;
  wbc_gains_t _wbcGains;
  // QP weights; friction and normal-force bounds are copied from [mpc] in
  // activate() so the WBIC pyramid is the MPC's.
  double _wbcWBase = 0.1;
  double _wbcWForce = 1.0;
  // Joint PD around the hierarchy's joint references, every leg.
  double _wbcJointKp = 30.0;
  double _wbcJointKd = 3.0;
  double _wbcTouchdownDepth = 0.003;
  int _wbcFailures = 0;  // consecutive ticks without a WBC command
  // Statistics since activation, for deactivate()'s closing line.
  int _wbcTicks = 0;
  int _wbcQpFailures = 0;
  int _wbcFallbackTicks = 0;
  int _wbcSaturatedTicks = 0;
  double _wbcTimeSum = 0.0;  // [s]
  double _wbcTimeMax = 0.0;  // [s]
  // Logged as MdlTrot_wbc, laid out as:
  //   [6i .. 6i+2]    leg i desired (MPC) ground reaction, world [N]
  //   [6i+3 .. 6i+5]  leg i solved (WBIC) ground reaction, world [N]; zero in swing
  //   [24]            QP status: qpOASES returnValue, 0 success, -1 not solved
  //                   (joint state or model terms unusable, or more than two
  //                   feet swinging)
  //   [25..27]        body orientation task error, rotation vector, world [rad]
  //   [28..30]        base position task error, world [m]
  //   [31+3i .. 33+3i] leg i swing error p_B_des - p_B, body axes [m]; zero in stance
  //   [43]            1 when the tick fell back to the per-leg commands; [0..23]
  //                   and [25..42] then hold the last solved tick's values, [24]
  //                   is this tick's status
  //   [44]            WBC time of the tick: model terms, tasks and QP [us]
  double _logWbc[45] = {0};

  void _readConfig();

  /** \brief Nominal resting foot position for a leg, in the body frame. */
  Eigen::Vector3d _originFoot(int leg) const;

  /** \brief Foot velocity in the body frame while this leg is planted. */
  Eigen::Vector3d _stanceVelocity(int leg) const;

  /** \brief Refreshes _comPos, _comVel, _rpy, _omegaWorld and the unwrapped yaw
      from the two estimators. False when either has nothing usable to say. */
  bool _readBodyState();

  /** \brief Advances the commanded world position and yaw by one behavior
      cycle, and clamps the position back toward the estimate. */
  void _integrateBodyReference();

  /** \brief Builds one horizon from the gait and runs the solver. */
  bool _solveMPC(double elapsed);

  /** \brief Body velocity used for the stance sweep: filtered estimate, or
      command when no estimate is available. It is independent of the command
      ramp so planted feet follow the measured body motion. */
  Eigen::Vector3d _sweepVelocity() const;

  /** \brief Commanded body twist and yaw rate, scaled by the entry/exit ramp.
      Every consumer of the command goes through these two. */
  Eigen::Vector3d _twistCommand() const { return _speedScale * _vcmd; }
  double _yawRateCommand() const { return _speedScale * _yawRate; }

  /** \brief Advances _speedScale, _liftScale and _liftScaleDot for the current
      state. \param elapsed Time since TROT entry. */
  void _updateGaitScales(double elapsed);

  bool _startSwing(int leg, double elapsed, double duration);

  /** \brief Rotation about the world z axis, the only reference attitude the
      body reference ever has. */
  static Eigen::Matrix3d _rotZ(double yaw);

  void _sendTarget();
  bool _checkTrackingError() const;

  /** \brief Commands all four legs through wbcSolve(). False, with nothing
      commanded, when the WBC has no solution this tick. */
  bool _wbcTick();

  void _hold();
  void _prepEntry();
  void _prepDuring();
  void _trotEntry();
  void _trotDuring();
  void _stoppingEntry();
  void _centeringEntry();
  void _centeringDuring();
};

#endif
