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

#include "rtcore/Module.hh"

#define TROTMODULE_NAME "MdlTrot"

class MdlLegControl;
class MdlPosVelEstimator;
class QuadrupedKinematics;

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

/** \brief Behavioral module that walks the robot with an open loop trot.

  Commands all four feet along the TrotGait schedule, offset from a nominal
  footprint built under the hips, through MdlLegControl's foot space interface.

  The module is deliberately open loop: it reads no state estimate and closes
  no loop around the body pose. It exists so that MdlStateEstimator has motion
  to track, and feeding the estimate back into the gait would make the two
  impossible to judge separately. Every other behavior on this platform keeps
  four feet planted, which leaves the estimator's foothold relocation, its
  velocity states and its bias states entirely unexercised.

  The state machine mirrors MdlDrawSquare. WAIT holds the pose the module was
  activated in, PREP blends from there to the nominal footprint, TROT runs the
  gait, and CENTERING decelerates back to the footprint with zero velocity so
  the handoff to whatever comes next has no discontinuity.

  Stride grows from zero over rampup_duration once TROT starts, so the robot
  steps in place before it accelerates. That removes the velocity step at the
  PREP handoff and keeps the first strides gentle.

  Configuration lives in the [trot] table; see trot.toml.
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
  enum class _state_t { WAIT, PREP, TROT, CENTERING, DONE };

  MdlLegControl* _legs[NUM_LEGS] = {};
  QuadrupedKinematics* _kinematics = nullptr;
  MdlPosVelEstimator* _posvel = nullptr;
  TrotGait _gait;

  _state_t _state = _state_t::WAIT;
  Status _status = IDLE;

  // Reference trajectory for the current cycle, in the body frame
  Eigen::Vector3d _footpos[NUM_LEGS];
  Eigen::Vector3d _footvel[NUM_LEGS];
  bool _stance[NUM_LEGS] = {true, true, true, true};

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

  int _ikFailures[NUM_LEGS] = {0, 0, 0, 0};

  // -- Configuration, all from the [trot] table

  Eigen::Vector3d _vcmd = Eigen::Vector3d(0.15, 0.0, 0.0);
  double _yawRate = 0.0;

  // Stance sweep velocity feedback. Off by default, which keeps the module
  // open loop and so keeps the estimator's drift figure a measurement of the
  // estimator rather than of the two together.
  //
  // A planted foot only avoids scrubbing if it is swept at the negated *actual*
  // body velocity. Open loop the gait uses the commanded one, and the
  // difference drags the foot for the whole stance. Enabling this replaces the
  // command with the estimate, blended by gain so that 0 is fully open loop
  // and 1 fully closed, and clamped so a bad estimate cannot run away with the
  // stride.
  bool _vfbEnable = false;
  double _vfbGain = 1.0;
  double _vfbTau = 0.1;    // [s] low pass on the estimate
  double _vfbLimit = 0.1;  // [m/s] cap on the correction, per axis
  // Filtered body-frame velocity estimate, updated once per cycle in TROT.
  Eigen::Vector3d _vfilt = Eigen::Vector3d::Zero();
  bool _vfbActive = false;  // estimate was usable on the last cycle

  double _origin[3] = {0.0, 0.10, -0.28};

  double _wait_duration = 0.3;
  double _prep_duration = 1.5;
  double _rampup_duration = 2.0;
  double _centering_duration = 1.5;

  Eigen::Vector3d _stance_kp = Eigen::Vector3d::Constant(150.0);
  Eigen::Vector3d _stance_kd = Eigen::Vector3d::Constant(4.0);
  Eigen::Vector3d _swing_kp = Eigen::Vector3d::Constant(15.0);
  Eigen::Vector3d _swing_kd = Eigen::Vector3d::Constant(0.6);

  double _trackingErrorLimit = 0.5;
  int _ikFailureLimit = 3;

  void _readConfig();

  /** \brief Nominal resting foot position for a leg, in the body frame. */
  Eigen::Vector3d _originFoot(int leg) const;

  /** \brief Foot velocity in the body frame while this leg is planted. */
  Eigen::Vector3d _stanceVelocity(int leg) const;

  /** \brief Refreshes _vfilt from the estimator. Once per cycle, not once per
      leg, so all four feet are swept against the same velocity. */
  void _updateVelocityFeedback();

  /** \brief Body velocity the stance sweep is built on: the command, or the
      estimate when velocity feedback is enabled and available. */
  Eigen::Vector3d _sweepVelocity() const;

  void _sendTarget();
  bool _checkTrackingError() const;

  void _hold();
  void _prepEntry();
  void _prepDuring();
  void _trotEntry();
  void _trotDuring();
  void _centeringEntry();
  void _centeringDuring();
};

#endif
