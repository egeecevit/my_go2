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

class MdlLegControl;
class MdlOrientationEstimator;
class MdlPosVelEstimator;
class QuadrupedKinematics;

/** Integer-clock schedule for MPC solve deadlines.

  Module time is represented in integer microseconds. Keeping the cadence in
  that representation avoids the repeated floating comparison which made a
  nominal 10 ms solve slip to 11 ms. Contact changes may trigger an early solve;
  that event starts a fresh exact period so it is not followed immediately by
  the obsolete deadline. */
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

  A segment is reset exactly once at liftoff and sampled until touchdown. This
  prevents the current velocity estimate from moving both endpoints every
  control cycle, which would make an analytic acceleration feedforward describe
  a different trajectory from the position command. */
class SwingTrajectory {
 public:
  bool reset(const Eigen::Vector3d& position0, const Eigen::Vector3d& velocity0,
             const Eigen::Vector3d& position1, const Eigen::Vector3d& velocity1,
             double duration, double height);

  /** Sample with a possibly time-varying clearance scale.

    scale_dot and scale_ddot are included so STOPPING can fade clearance without
    breaking the derivative relationship between the returned p, v and a. */
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

  Pure geometry with no hardware or framework dependency, so it can be
  exercised directly from a unit test. MdlTrot owns one and adds the robot.

  The scheduler answers a single question: given a time and a leg, where should
  that foot be relative to its nominal resting place, and how fast is it moving
  there. It knows nothing about where the nominal footprint is, which is what
  lets the caller decide the stance height and stance width.

  The caller supplies the leg's stance velocity u, the velocity of the foot in
  the body frame while it is planted. For a body advancing at v with no turn,
  u is simply -v for every leg. The gait is parameterized this way rather than
  by a stride length because it is the velocity that has to be right: a planted
  foot which does not translate at exactly the negated body velocity scrapes
  along the ground. Stride length then falls out as u * duty * period.

  Phase runs 0 to 1 over one cycle. A leg is in stance while its phase is below
  the duty factor and in swing above it. Stance sweeps the foot backwards at
  constant velocity. Swing returns it along a cubic Hermite whose end tangents
  both equal u, so the foot leaves the ground at the velocity stance ended with
  and lands at the velocity stance is about to resume, and lifts it along a
  raised cosine which is zero in both value and slope at either end. The result
  is continuous in position and velocity everywhere around the cycle, including
  across both transitions.

  The phase offsets select the gait. Diagonal pairs a half cycle apart is a
  trot; the same code gives a pace, a bound or a static walk for other offsets.
 */
struct TrotGait {
  /** \brief Leg count, indexed as QuadrupedKinematics::LegIndex. */
  static constexpr int NUM_LEGS = 4;

  struct params_t {
    /** \brief Duration of one full gait cycle [s]. */
    double period = 0.5;
    /** \brief Fraction of the cycle a leg spends in stance. 0.5 is a pure
        trot; above that the diagonal pairs overlap, which is slower but keeps
        more feet on the ground and is markedly easier to balance. */
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

  /** \brief Progress through the stance phase, in [0, 1), and exactly 0 while
      swinging.

    Distinct from legPhase(), which runs over the whole cycle: a leg at phase
    0.6 with a duty of 0.5 is airborne, and reports 0 here.

    This is the form the state estimator's contact trust ramp is written
    against, and it matches MIT Cheetah's Gait::getContactState(). Note that a
    leg entering stance reports 0, the same value a swinging leg reports. That
    is deliberate rather than an edge case to paper over: the first instant of
    stance is the least trustworthy moment of the whole cycle, because the foot
    is still arriving. */
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

  MdlTrot is the coordinator. It owns gait phase, future contact timing, the
  body reference and the swing foot references; MdlConvexMPC turns those plus
  the estimated body state into four world frame ground reaction forces; and
  MdlLegControl turns those into motor torque. The division follows the block
  diagram of Di Carlo et al., *Dynamic Locomotion in the MIT Cheetah 3 through
  Convex Model-Predictive Control* (IROS 2018).

  Both control modes end at MdlLegControl's Cartesian torque boundary:

    - swing: a frozen C2 foot segment with configuration-dependent
      apparent-mass kp/kd and an explicitly gated inverse-dynamics term
    - stance: zero Cartesian gains, the MPC force rotated into the body frame

  This is what the module could not do before. Its earlier revision commanded
  stance through IK and joint PD, which caps the speed the robot can make good:
  a planted foot swept by a position controlled leg turns the tracking error
  into leg deflection rather than body motion, and the robot realized about a
  tenth of the commanded velocity whatever the gains were. Stance is now a force
  command and there is no position loop on a planted foot at all.

  Only the TROT state changed. WAIT holds the pose the module was activated in,
  PREP blends from there to the nominal footprint, and CENTERING decelerates
  back to the footprint with zero velocity so the handoff to whatever comes next
  has no discontinuity; all three keep four feet planted and stay on the
  IK/joint-PD path, which is the right controller for holding a pose.

  Entry and exit are ramped, and the ramp is on the *command* rather than on the
  foot trajectory. A speed scale rising over rampup_duration multiplies the
  commanded twist wherever it is consumed -- the body reference, the MPC's
  reference trajectory and the stance sweep's turn term -- so the feet step in
  place at full clearance from the first cycle and the solver is only asked for
  acceleration once the gait is actually running. Scaling the foot offsets
  instead, which is what this did before, breaks the one invariant stance rests
  on: the sweep is built on the estimated body velocity precisely so a planted
  foot translates at -v_actual, and a factor in front of it is scrub.

  stopTrotting() runs the same scale back down over rampdown_duration in the
  STOPPING state, fading the swing clearance with it. Both diagonal pairs end up
  on the nominal footprint at rest, which is what makes the handoff to CENTERING,
  and from there to a pose controller, free of a step.

  There is no silent fallback. If either estimator is not ready, or the first
  MPC solve fails, or solves keep failing, the module goes to ERROR rather than
  quietly reverting to position controlled stance -- a robot trotting on the old
  controller while the log says MPC is the thing nobody would notice.

  Configuration lives in the [trot] table; see trot.toml, and in [mpc] for the
  solver; see mpc.toml.
 */
class MdlTrot : public rtcore::Module {
 public:
  static constexpr int NUM_LEGS = TrotGait::NUM_LEGS;

  /** \brief Same shape as MdlStand and MdlSit, so the Supervisor can treat all
      three alike. SETTLED means centering finished and motion has come to rest. */
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

  /** \brief Per-leg stance progress, for the state estimator's contact trust.

    Returns false and leaves phase untouched unless the gait is actually
    running. A false return means "no schedule to report", not "no contact":
    WAIT, PREP, CENTERING and DONE all hold four feet on the ground, as does
    every other behavior on this platform, so the caller should fall back to
    treating the robot as fully planted.

    Recomputed from the gait datum on every call rather than served from the
    cached _stance array, because the estimator runs at SENSING_MODULES and
    this module at BEHAVIORAL_CONTROLLERS. The estimator therefore asks before
    this module has updated, and anything cached would always be one cycle old. */
  bool getStancePhase(double phase[NUM_LEGS]) const;

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

  // Reference trajectory for the current cycle, in the body frame
  Eigen::Vector3d _footpos[NUM_LEGS];
  Eigen::Vector3d _footvel[NUM_LEGS];
  Eigen::Vector3d _footacc[NUM_LEGS];
  bool _stance[NUM_LEGS] = {true, true, true, true};

  SwingTrajectory _swingTrajectory[NUM_LEGS];
  double _swingStart[NUM_LEGS] = {0};
  double _swingEnd[NUM_LEGS] = {0};

  // Pose captured at activation, and the PREP blend target
  Eigen::Vector3d _footpos_start[NUM_LEGS];
  Eigen::Vector3d _footpos_end[NUM_LEGS];

  // Snapshot at stop time. Holding the velocity too lets CENTERING decelerate
  // smoothly through a cubic Hermite instead of snapping to zero.
  Eigen::Vector3d _center_pos0[NUM_LEGS];
  Eigen::Vector3d _center_vel0[NUM_LEGS];
  Eigen::Vector3d _center_pos1[NUM_LEGS];

  double _mark = 0.0;       // entry time of the current state
  double _trot_mark = 0.0;  // entry time of TROT, the gait phase datum
  double _stop_mark = 0.0;  // entry time of STOPPING, the ramp-down datum

  // -- Command ramp ----------------------------------------------------------
  //
  // _speedScale multiplies the commanded twist; _liftScale multiplies the swing
  // clearance. They are deliberately not the same number. On the way up the lift
  // stays at one, so the gait, the contact schedule and the MPC's contact mask
  // are all real before any acceleration is demanded; on the way down it fades
  // with the speed, so every foot settles onto the nominal footprint before a
  // pose controller takes over.
  double _speedScale = 0.0;
  double _liftScale = 1.0;
  double _liftScaleDot = 0.0;  // d/dt, for the swing velocity's product rule
  double _liftScaleDDot = 0.0;
  // Speed scale at the moment the stop was asked for, so a stop during the
  // ramp-up decelerates from where it had got to instead of stepping up first.
  double _stopSpeedScale0 = 0.0;

  int _cmdFailures[NUM_LEGS] = {0, 0, 0, 0};

  // The two ramp scales, which are otherwise invisible offline: nothing else in
  // the log distinguishes a command the robot is failing to track from one that
  // was never given. Two doubles and no more -- see the size budget in
  // supervisor.toml, which the full body reference would overflow.
  rtcore::LogServer* _logserver = nullptr;
  double _logScales[2] = {0};  // speed scale, lift scale
  double _logFootReference[NUM_LEGS * 9] = {0};  // per-leg p, v, a
  double _logContact[NUM_LEGS] = {0};
  double _logTorqueScale[NUM_LEGS] = {1, 1, 1, 1};
  // Torque _limitTorque() asked for, before the saturation scale above is
  // applied to it -- MdlSimDriver_ctrl only logs what the motor actually got,
  // so "commanded vs applied" is otherwise unplottable. Same 3*leg+joint
  // layout as MotorHW.
  double _logTorqueRequest[NUM_LEGS * 3] = {0};
  // Body reference and the ramp/clamp state around it, laid out as:
  //   [0] _desPos.x()      [4] desVel_world.x()   [8]  _yawUnwrapped
  //   [1] _desPos.y()      [5] desVel_world.y()   [9]  _speedScale
  //   [2] _desPos.z()      [6] desVel_world.z()   [10] _liftScale
  //   [3] _desYaw          [7] _yawRateCommand()  [11] _yawClamp
  // _yawClamp rides along so an offline plot can draw the clamp band without
  // hard-coding a number that may change; everything else here was previously
  // dead reckonable only from the command config, which a ramp or a clamp
  // silently overrides.
  double _logBodyReference[12] = {0};

  // -- Measured body state, refreshed once per TROT cycle --------------------
  //
  // At the whole-robot centre of mass, not the body frame origin: that is the
  // point the paper's rigid body model and mpc.inertia_body are both about, and
  // MdlPosVelEstimator estimates the other one. Mixing them puts a fixed lever
  // arm error into every predicted moment.
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

  // Filtered body-frame velocity estimate, refreshed once per cycle in TROT.
  // The touchdown geometry is built on the estimate rather than the command,
  // Low-passed body-frame velocity used as a bounded correction to the commanded
  // stance sweep. Pure estimated-velocity sweep turns a small persistent KF
  // bias into continuous foot scrubbing, while a pure command cannot recover
  // from a real disturbance.
  Eigen::Vector3d _vfilt = Eigen::Vector3d::Zero();
  bool _vfiltValid = false;
  double _stanceVelocityFeedback = 0.25;

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

  // Cartesian swing gains [N/m] and [N/(m/s)], and the joint damping that rides
  // along with every TROT command. Not the old joint-space numbers under new
  // names: the units changed with the control law.
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
  int _torqueSaturationLimit = 20;
  int _torqueSaturationCycles[NUM_LEGS] = {0, 0, 0, 0};

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

  /** \brief Body velocity the stance sweep is built on: the filtered estimate,
      or the command when no estimate is available.

    Deliberately not scaled by the command ramp. A planted foot has to translate
    at the negated velocity the body actually has, whatever the command is doing;
    scaling this is what makes a stance foot scrub. */
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
