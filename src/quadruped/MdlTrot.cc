/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "hardware/MotorHW.hh"
#include "quadruped/MdlConvexMPC.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/MdlOrientationEstimator.hh"
#include "quadruped/MdlPosVelEstimator.hh"
#include "quadruped/MdlTrot.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/TrajectoryUtils.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ModuleManager.hh"

using namespace rtcore;

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...)  // printf(__VA_ARGS__)

// ---------------------------------------------------------------------------
// PeriodicSolveSchedule
// ---------------------------------------------------------------------------

bool PeriodicSolveSchedule::reset(CLOCK now, double period_seconds) {
  _valid = false;
  const double ticks = 1.0e6 * period_seconds;
  if (!std::isfinite(ticks) || ticks < 0.5 ||
      ticks > static_cast<double>(std::numeric_limits<CLOCK>::max()))
    return false;

  _period = static_cast<CLOCK>(std::llround(ticks));
  _next = now + _period;
  _valid = true;
  return true;
}

void PeriodicSolveSchedule::consumed(CLOCK now, bool asynchronous) {
  if (!_valid) return;

  if (asynchronous && now < _next) {
    _next = now + _period;
    return;
  }

  // Preserve the integer deadline lattice when a controller cycle arrives
  // late. Assigning now+period here would turn one late cycle into permanent
  // cadence drift.
  while (_next <= now) _next += _period;
}

// ---------------------------------------------------------------------------
// SwingTrajectory
// ---------------------------------------------------------------------------

bool SwingTrajectory::reset(const Eigen::Vector3d& position0,
                            const Eigen::Vector3d& velocity0,
                            const Eigen::Vector3d& position1,
                            const Eigen::Vector3d& velocity1, double duration,
                            double height) {
  _valid = false;
  if (!position0.allFinite() || !velocity0.allFinite() || !position1.allFinite() ||
      !velocity1.allFinite() || !(duration > 0.0) || !std::isfinite(duration) ||
      !(height >= 0.0) || !std::isfinite(height))
    return false;

  _duration = duration;
  _height = height;
  _position1 = position1;
  _velocity1 = velocity1;

  const double T = duration;
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;
  const Eigen::Vector3d delta = position1 - position0;

  // Fifth-order Hermite polynomial with zero acceleration at both ends.
  _coefficient[0] = position0;
  _coefficient[1] = velocity0;
  _coefficient[2].setZero();
  _coefficient[3] = 10.0 * delta / T3 - (6.0 * velocity0 + 4.0 * velocity1) / T2;
  _coefficient[4] = -15.0 * delta / T4 +
                    (8.0 * velocity0 + 7.0 * velocity1) / T3;
  _coefficient[5] = 6.0 * delta / T5 -
                    3.0 * (velocity0 + velocity1) / T4;
  _valid = true;
  return true;
}

void SwingTrajectory::sample(double time, double scale, double scale_dot,
                             double scale_ddot, Eigen::Vector3d& position,
                             Eigen::Vector3d& velocity,
                             Eigen::Vector3d& acceleration) const {
  if (!_valid) {
    position.setZero();
    velocity.setZero();
    acceleration.setZero();
    return;
  }

  const double t = std::max(0.0, std::min(_duration, time));
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  position = _coefficient[0] + _coefficient[1] * t + _coefficient[2] * t2 +
             _coefficient[3] * t3 + _coefficient[4] * t4 + _coefficient[5] * t5;
  velocity = _coefficient[1] + 2.0 * _coefficient[2] * t +
             3.0 * _coefficient[3] * t2 + 4.0 * _coefficient[4] * t3 +
             5.0 * _coefficient[5] * t4;
  acceleration = 2.0 * _coefficient[2] + 6.0 * _coefficient[3] * t +
                 12.0 * _coefficient[4] * t2 + 20.0 * _coefficient[5] * t3;

  const double s = t / _duration;
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  const double s6 = s5 * s;
  const double bump = 64.0 * (s3 - 3.0 * s4 + 3.0 * s5 - s6);
  const double bump_dot =
      64.0 * (3.0 * s2 - 12.0 * s3 + 15.0 * s4 - 6.0 * s5) / _duration;
  const double bump_ddot =
      64.0 * (6.0 * s - 36.0 * s2 + 60.0 * s3 - 30.0 * s4) /
      (_duration * _duration);

  position.z() += _height * scale * bump;
  velocity.z() += _height * (scale * bump_dot + scale_dot * bump);
  acceleration.z() +=
      _height * (scale * bump_ddot + 2.0 * scale_dot * bump_dot + scale_ddot * bump);
}

// ---------------------------------------------------------------------------
// TrotGait
// ---------------------------------------------------------------------------

void TrotGait::setParams(const params_t& params) {
  _params = params;

  // Clamp anything that would divide by zero or leave a phase with no extent.
  // Written to also reject NaN, which would otherwise propagate into every
  // foot command silently.
  if (!(_params.period > 0.0)) _params.period = 0.5;
  if (!(_params.duty > 0.1)) _params.duty = 0.1;
  if (!(_params.duty < 0.9)) _params.duty = 0.9;
  if (!(_params.swing_height >= 0.0)) _params.swing_height = 0.0;

  for (int i = 0; i < NUM_LEGS; i++)
    if (!std::isfinite(_params.phase_offset[i])) _params.phase_offset[i] = 0.0;
}

double TrotGait::legPhase(int leg, double t) const {
  double phi = t / _params.period + _params.phase_offset[leg];
  phi -= std::floor(phi);
  // floor() of a value just below an integer can round up to it in the
  // subtraction, so pin the result strictly inside [0, 1).
  if (phi >= 1.0) phi = 0.0;
  return phi;
}

bool TrotGait::inStance(int leg, double t) const {
  return legPhase(leg, t) < _params.duty;
}

double TrotGait::stanceProgress(int leg, double t) const {
  const double phi = legPhase(leg, t);
  if (phi >= _params.duty) return 0.0;
  return phi / _params.duty;
}

void TrotGait::sample(int leg, double t, const Eigen::Vector3d& u, Eigen::Vector3d& dp,
                      Eigen::Vector3d& dv) const {
  const double beta = _params.duty;
  const double stanceTime = beta * _params.period;
  const double swingTime = (1.0 - beta) * _params.period;
  const double phi = legPhase(leg, t);

  if (phi < beta) {
    // Stance. Constant velocity throughout: the foot is on the ground, so it
    // must translate at exactly the negated body velocity or it drags. This is
    // also what lets the state estimator treat the foothold as a fixed point
    // in the world, which is the assumption its kinematic measurement rests on.
    const double s = phi / beta;
    dp = u * stanceTime * (s - 0.5);
    dv = u;
    return;
  }

  // Swing. Cubic Hermite from the back of the stroke to the front, with both
  // end tangents equal to the stance velocity. The foot therefore lifts off
  // still moving backwards at u and touches down moving backwards at u, which
  // is what stance is about to do anyway. Position and velocity are continuous
  // across both transitions and the cycle closes on itself.
  const double us = (phi - beta) / (1.0 - beta);

  const Eigen::Vector3d p0 = u * stanceTime * 0.5;   // where stance left off
  const Eigen::Vector3d p1 = -u * stanceTime * 0.5;  // where stance resumes
  const Eigen::Vector3d m = u * swingTime;           // shared end tangent

  const double u2 = us * us;
  const double u3 = u2 * us;

  const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
  const double h10 = u3 - 2.0 * u2 + us;
  const double h01 = -2.0 * u3 + 3.0 * u2;
  const double h11 = u3 - u2;

  const double d00 = 6.0 * u2 - 6.0 * us;
  const double d10 = 3.0 * u2 - 4.0 * us + 1.0;
  const double d01 = -6.0 * u2 + 6.0 * us;
  const double d11 = 3.0 * u2 - 2.0 * us;

  dp = h00 * p0 + h10 * m + h01 * p1 + h11 * m;
  dv = (d00 * p0 + d10 * m + d01 * p1 + d11 * m) / swingTime;

  // Vertical clearance. A raised cosine is zero in both value and slope at
  // either end, so it grafts onto the flat stance height without a kink and
  // sets the foot down softly rather than driving it into the ground.
  dp.z() += _params.swing_height * 0.5 * (1.0 - std::cos(2.0 * M_PI * us));
  dv.z() += _params.swing_height * M_PI * std::sin(2.0 * M_PI * us) / swingTime;
}

// ---------------------------------------------------------------------------
// MdlTrot
// ---------------------------------------------------------------------------

MdlTrot::MdlTrot() : Module(TROTMODULE_NAME, 0, SINGLE_USER) {
  DBGPRINT("MdlTrot::MdlTrot\n");
}

MdlTrot::~MdlTrot() { DBGPRINT("MdlTrot::~MdlTrot\n"); }

void MdlTrot::init() {
  DBGPRINT("MdlTrot::init\n");

  for (int i = 0; i < NUM_LEGS; i++)
    _legs[i] = (MdlLegControl*)_mgr->findModule(LEGMODULE_NAME, i);

  // Read only, and no longer optional. Both estimators are MULTI_USER shared
  // sensors created before this module in AddCoreModules and running at
  // SENSING_MODULES, so their answer is from this cycle and there is nothing to
  // grab. Stance is a force command now, and a force command with no idea where
  // the body is or which way it is pointing is not something to fall back to.
  _orientation = (MdlOrientationEstimator*)_mgr->findModule(ORIENTATIONMODULE_NAME, 0);
  _posvel = (MdlPosVelEstimator*)_mgr->findModule(POSVELMODULE_NAME, 0);
  if (!_orientation || !_posvel)
    _mgr->fatalError(TROTMODULE_NAME,
                     "%s and %s must be created before %s in AddCoreModules()",
                     ORIENTATIONMODULE_NAME, POSVELMODULE_NAME, TROTMODULE_NAME);

  // The solver, on the other hand, is SINGLE_USER and is grabbed alongside the
  // legs: it holds warm start state that belongs to whoever is walking.
  _mpc = (MdlConvexMPC*)_mgr->findModule(MPCMODULE_NAME, 0);
  if (!_mpc)
    _mgr->fatalError(TROTMODULE_NAME, "%s must be created before %s in AddCoreModules()",
                     MPCMODULE_NAME, TROTMODULE_NAME);

  ConfigTable hwConfig;
  std::string hwlib;
  if (_mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");

  if (hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

  // The two ramp scales. Nothing else in the log tells a command the robot
  // failed to track apart from one that was never given, which is exactly the
  // distinction the entry and exit transitions turn on.
  _logserver = (LogServer*)_mgr->findModule(LOGSERVER_NAME, 0);
  if (_logserver) {
    _logserver->registerVar(LOG_DOUBLE, 2, TROTMODULE_NAME, "scales",
                            (unsigned char*)_logScales);
    _logserver->registerVar(LOG_DOUBLE, NUM_LEGS * 9, TROTMODULE_NAME, "footref",
                            (unsigned char*)_logFootReference);
    _logserver->registerVar(LOG_DOUBLE, NUM_LEGS, TROTMODULE_NAME, "contact",
                            (unsigned char*)_logContact);
    _logserver->registerVar(LOG_DOUBLE, NUM_LEGS, TROTMODULE_NAME, "torquescale",
                            (unsigned char*)_logTorqueScale);
    _logserver->registerVar(LOG_DOUBLE, NUM_LEGS * 3, TROTMODULE_NAME, "torquereq",
                            (unsigned char*)_logTorqueRequest);
    _logserver->registerVar(LOG_DOUBLE, 12, TROTMODULE_NAME, "bodyref",
                            (unsigned char*)_logBodyReference);
  }

  _readConfig();
}

void MdlTrot::_readConfig() {
  ConfigTable config;
  if (!_mgr->getConfigTable("trot", config)) {
    _mgr->warning(TROTMODULE_NAME, "No [trot] config table; using built-in defaults");
    _gait.setParams(TrotGait::params_t());
    return;
  }

  _vcmd.x() = config.getDouble("forward_velocity", _vcmd.x());
  _vcmd.y() = config.getDouble("lateral_velocity", _vcmd.y());
  _vcmd.z() = 0.0;
  _yawRate = config.getDouble("yaw_rate", _yawRate);

  _velocityFilterTau = config.getDouble("velocity_filter_tau", _velocityFilterTau);
  // A zero time constant would make the filter a passthrough of a 1 kHz signal,
  // which is exactly what it is there to avoid.
  if (!(_velocityFilterTau > 0.0)) _velocityFilterTau = 0.01;
  _stanceVelocityFeedback =
      config.getDouble("stance_velocity_feedback", _stanceVelocityFeedback);
  if (!(_stanceVelocityFeedback >= 0.0)) _stanceVelocityFeedback = 0.0;
  if (_stanceVelocityFeedback > 1.0) _stanceVelocityFeedback = 1.0;

  TrotGait::params_t gp;
  gp.period = config.getDouble("period", gp.period);
  gp.duty = config.getDouble("duty", gp.duty);
  gp.swing_height = config.getDouble("swing_height", gp.swing_height);

  ConfigArray offsets;
  if (config.getArray("phase_offset", offsets) && offsets.size() >= NUM_LEGS) {
    for (int i = 0; i < NUM_LEGS; i++)
      gp.phase_offset[i] = offsets.getDoubleAt(i, gp.phase_offset[i]);
  }
  _gait.setParams(gp);

  ConfigArray origin;
  if (config.getArray("origin", origin)) {
    for (int i = 0; i < 3; i++) _origin[i] = origin.getDoubleAt(i, _origin[i]);
  }

  _wait_duration = config.getDouble("wait_duration", _wait_duration);
  _prep_duration = config.getDouble("prep_duration", _prep_duration);
  _rampup_duration = config.getDouble("rampup_duration", _rampup_duration);
  _rampdown_duration = config.getDouble("rampdown_duration", _rampdown_duration);
  _centering_duration = config.getDouble("centering_duration", _centering_duration);

  // Zero or negative durations would divide by zero in the blend math
  if (!(_wait_duration >= 0.0)) _wait_duration = 0.0;
  if (!(_prep_duration > 0.0)) _prep_duration = 0.01;
  if (!(_rampup_duration > 0.0)) _rampup_duration = 0.01;
  if (!(_rampdown_duration > 0.0)) _rampdown_duration = 0.01;
  if (!(_centering_duration > 0.0)) _centering_duration = 0.01;

  _prep_kp = Eigen::Vector3d::Constant(config.getDouble("prep_kp", _prep_kp.x()));
  _prep_kd = Eigen::Vector3d::Constant(config.getDouble("prep_kd", _prep_kd.x()));
  _swing_kp =
      Eigen::Vector3d::Constant(config.getDouble("swing_kp_cartesian", _swing_kp.x()));
  _swing_kd =
      Eigen::Vector3d::Constant(config.getDouble("swing_kd_cartesian", _swing_kd.x()));

  ConfigArray naturalFrequency;
  if (config.getArray("swing_natural_frequency", naturalFrequency) &&
      naturalFrequency.size() == 3) {
    for (int i = 0; i < 3; ++i)
      _swingNaturalFrequency[i] =
          naturalFrequency.getDoubleAt(i, _swingNaturalFrequency[i]);
  }
  ConfigArray dampingRatio;
  if (config.getArray("swing_damping_ratio", dampingRatio) && dampingRatio.size() == 3) {
    for (int i = 0; i < 3; ++i)
      _swingDampingRatio[i] = dampingRatio.getDoubleAt(i, _swingDampingRatio[i]);
  }
  for (int i = 0; i < 3; ++i) {
    if (!(_swingNaturalFrequency[i] > 0.0)) _swingNaturalFrequency[i] = 28.0;
    if (!(_swingDampingRatio[i] > 0.0)) _swingDampingRatio[i] = 0.8;
  }
  _swingInverseDynamics = config.getBool("swing_inverse_dynamics", true);
  _swingFeedforwardScale =
      config.getDouble("swing_feedforward_scale", _swingFeedforwardScale);
  if (!(_swingFeedforwardScale >= 0.0) || !std::isfinite(_swingFeedforwardScale))
    _swingFeedforwardScale = 0.0;

  _jointDamping = config.getDouble("joint_damping", _jointDamping);
  // MdlSimDriver substitutes 5.0 for a non-positive kd by mutating the stored
  // command, so a zero here would silently install a damping forty times the
  // intended one and keep it there.
  if (!(_jointDamping > 0.0)) _jointDamping = 0.2;

  _positionClamp = config.getDouble("position_reference_clamp", _positionClamp);
  if (!(_positionClamp > 0.0)) _positionClamp = 0.1;

  _yawClamp = config.getDouble("yaw_reference_clamp", _yawClamp);
  if (!(_yawClamp > 0.0)) _yawClamp = 0.2;

  _trackingErrorLimit = config.getDouble("tracking_error_limit", _trackingErrorLimit);
  _cmdFailureLimit = (int)config.getInt("command_failure_limit", _cmdFailureLimit);
  if (_cmdFailureLimit < 1) _cmdFailureLimit = 1;
  _mpcFailureLimit = (int)config.getInt("mpc_failure_limit", _mpcFailureLimit);
  if (_mpcFailureLimit < 0) _mpcFailureLimit = 0;
  _torqueSaturationLimit =
      (int)config.getInt("torque_saturation_limit", _torqueSaturationLimit);
  if (_torqueSaturationLimit < 1) _torqueSaturationLimit = 1;
}

void MdlTrot::uninit() {
  DBGPRINT("MdlTrot::uninit\n");

  if (_logserver) {
    _logserver->deleteVar(TROTMODULE_NAME, "scales");
    _logserver->deleteVar(TROTMODULE_NAME, "footref");
    _logserver->deleteVar(TROTMODULE_NAME, "contact");
    _logserver->deleteVar(TROTMODULE_NAME, "torquescale");
    _logserver->deleteVar(TROTMODULE_NAME, "torquereq");
    _logserver->deleteVar(TROTMODULE_NAME, "bodyref");
    _logserver = nullptr;
  }

  delete _kinematics;
  _kinematics = nullptr;
}

void MdlTrot::activate() {
  DBGPRINT("MdlTrot::activate\n");

  _status = ACTIVE;
  _state = _state_t::WAIT;
  _mark = _mgr->readTime();

  for (int i = 0; i < NUM_LEGS; i++) {
    _cmdFailures[i] = 0;
    _torqueSaturationCycles[i] = 0;
    _logTorqueScale[i] = 1.0;
    _stance[i] = true;
    _mgr->grabModule(_legs[i], this);
  }
  // The solver goes with the legs. Its warm start describes the gait it was
  // last solving, so handing it to another owner mid-stride would be worse than
  // useless.
  _mgr->grabModule(_mpc, this);

  // Grab everything first, then capture FK. If any capture fails, release
  // everything before reporting the error.
  for (int i = 0; i < NUM_LEGS; i++) {
    if (!_legs[i]->getFootPosition(_footpos_start[i])) {
      _mgr->warning(TROTMODULE_NAME, "Failed to capture foot %d position", i);
      for (int j = 0; j < NUM_LEGS; j++) _mgr->releaseModule(_legs[j], this);
      _mgr->releaseModule(_mpc, this);
      _status = ERROR;
      return;
    }
    _footpos[i] = _footpos_start[i];
    _footvel[i] = Eigen::Vector3d::Zero();
  }

  _mgr->message("MdlTrot: v=[%.3f %.3f] m/s yaw=%.3f rad/s period=%.3f s duty=%.2f",
                _vcmd.x(), _vcmd.y(), _yawRate, _gait.getParams().period,
                _gait.getParams().duty);
}

void MdlTrot::deactivate() {
  DBGPRINT("MdlTrot::deactivate\n");

  // Status is deliberately left alone: the Supervisor reads it to decide what
  // to do next, and clearing it here would erase an ERROR it has not seen yet.
  for (int i = 0; i < NUM_LEGS; i++) _mgr->releaseModule(_legs[i], this);
  _mgr->releaseModule(_mpc, this);
}

bool MdlTrot::getStancePhase(double phase[NUM_LEGS]) const {
  // TROT and STOPPING run the schedule; every other state holds all four feet
  // down, which the caller represents itself, and saying so here by returning
  // false keeps this function honest about what it actually knows. STOPPING has
  // to be in the list: the feet are still swinging on a fading clearance for the
  // whole ramp-down, and reporting four planted feet there would hand the
  // estimator a measurement from an airborne foot on every stop.
  if (_state != _state_t::TROT && _state != _state_t::STOPPING) return false;

  const double elapsed = _mgr->readTime() - _trot_mark;
  for (int i = 0; i < NUM_LEGS; i++) phase[i] = _gait.stanceProgress(i, elapsed);
  return true;
}

// ---------------------------------------------------------------------------
//  Geometry helpers
// ---------------------------------------------------------------------------

Eigen::Matrix3d MdlTrot::_rotZ(double yaw) {
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

Eigen::Vector3d MdlTrot::_originFoot(int leg) const {
  // Nominal resting foot position for this leg, relative to the body. Same
  // construction as MdlDrawSquare: hip location, shifted by the configured
  // offset with the lateral component mirrored for the right hand legs.
  Eigen::Vector3d p = _kinematics->getKinematicParams().hip_positions.row(leg).transpose();
  p[0] += _origin[0];
  p[1] += _origin[1] * (leg % 2 == 0 ? 1.0 : -1.0);
  p[2] += _origin[2];
  return p;
}

Eigen::Vector3d MdlTrot::_sweepVelocity() const {
  const Eigen::Vector3d command = _twistCommand();
  if (!_vfiltValid) return command;

  // Equation (33) wants the actual velocity, but the filter is the signal
  // closing this loop. In the H25 failure it under-reported forward velocity
  // by about 3 cm/s for every stance, so a pure estimate swept each planted
  // foot forward relative to the ground and accumulated slip. Keep the
  // feedforward stride as the anchor and admit only a bounded correction.
  return command + _stanceVelocityFeedback * (_vfilt - command);
}

Eigen::Vector3d MdlTrot::_stanceVelocity(int leg) const {
  // Velocity of a planted foot in the body frame is the negated body twist
  // evaluated at that foot. The cross product term is what swings the footprint
  // around for a turn; it vanishes when yaw_rate is zero.
  const Eigen::Vector3d omega(0.0, 0.0, _yawRateCommand());
  return -(_sweepVelocity() + omega.cross(_originFoot(leg)));
}

// ---------------------------------------------------------------------------
//  Body state and reference
// ---------------------------------------------------------------------------

bool MdlTrot::_readBodyState() {
  _stateValid = false;
  if (!_orientation || !_posvel || !_mpc) return false;
  if (!_orientation->isReady() || !_posvel->isReady()) return false;

  Eigen::Vector3d pBody, vBody;
  if (!_posvel->getBodyPosition(pBody) || !_posvel->getBodyVelocity(vBody)) return false;
  if (!pBody.allFinite() || !vBody.allFinite()) return false;

  _Rbw = _orientation->getRotation();
  _rpy = _orientation->getRPY();
  _omegaWorld = _orientation->getAngularVelocityWorld();
  if (!_Rbw.allFinite() || !_rpy.allFinite() || !_omegaWorld.allFinite()) return false;

  // Body frame origin to whole-robot COM. Both the rigid body model and
  // mpc.inertia_body are about the COM, and the estimator reports the origin.
  const Eigen::Vector3d r = _Rbw * _mpc->getParams().com_offset_body;
  _comPos = pBody + r;
  _comVel = vBody + _omegaWorld.cross(r);

  // Unwrap onto a continuous branch. The QP differences the current and
  // reference yaw directly, so a wrap between the two reads as a full turn of
  // heading error and would produce a violent correcting moment.
  const double yaw = _rpy.z();
  if (!_haveYaw) {
    _yawUnwrapped = yaw;
    _haveYaw = true;
  } else {
    double d = yaw - _yawWrapped;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    _yawUnwrapped += d;
  }
  _yawWrapped = yaw;

  // Low passed body-frame COM velocity, for the touchdown geometry only.
  const Eigen::Vector3d vb = _Rbw.transpose() * _comVel;
  if (!_vfiltValid) {
    _vfilt = vb;
    _vfiltValid = true;
  } else {
    const double dt = CLOCK_TO_SEC(_mgr->getStepPeriod());
    const double a = dt / (_velocityFilterTau + dt);
    _vfilt += a * (vb - _vfilt);
  }
  // Horizontal only. The trunk really does rise and fall during a trot, but
  // that is the gait working, not a tracking error, and sweeping the stance
  // feet vertically to chase it would fight the stance height instead.
  _vfilt.z() = 0.0;

  _stateValid = true;
  return true;
}

void MdlTrot::_integrateBodyReference() {
  const double dt = CLOCK_TO_SEC(_mgr->getStepPeriod());

  // _vcmd is a body-frame twist, so it has to be rotated by the *desired*
  // heading before it can be integrated in the world. Using the measured
  // heading instead would let a heading error steer the reference.
  const Eigen::Vector3d vc = _twistCommand();
  _desYaw += _yawRateCommand() * dt;

  // Heading needs the same bound as position below, and needs it even when
  // nothing has gone wrong. A yaw rate the robot cannot quite achieve leaves
  // the reference running away at the shortfall for as long as the turn
  // lasts: at 0.5 rad/s commanded the robot held 0.46 rad/s while otherwise
  // healthy, and that 8% alone carried the reference 7.1 rad ahead in 30 s.
  // Bounding it converts an integrator into a saturated proportional heading
  // correction. 0.2 rad is one gait cycle of commanded heading at the nominal
  // yaw rate, the scale position_reference_clamp uses against stride length,
  // and a quarter of the 1.4 rad error the robot was toppling at.
  const double yawErr = _desYaw - _yawUnwrapped;
  if (yawErr > _yawClamp) _desYaw = _yawUnwrapped + _yawClamp;
  if (yawErr < -_yawClamp) _desYaw = _yawUnwrapped - _yawClamp;

  const Eigen::Vector3d v = _rotZ(_desYaw) * Eigen::Vector3d(vc.x(), vc.y(), 0.0);
  _desPos.x() += v.x() * dt;
  _desPos.y() += v.y() * dt;

  // Clamp back toward the estimate. Left alone the reference is a pure
  // integrator, so a slip, a shove, or a foot that never lands leaves a
  // position error that grows without bound, and the MPC then spends its whole
  // force budget chasing a point the robot is never going to reach.
  for (int i = 0; i < 2; i++) {
    const double err = _desPos[i] - _comPos[i];
    if (err > _positionClamp) _desPos[i] = _comPos[i] + _positionClamp;
    if (err < -_positionClamp) _desPos[i] = _comPos[i] - _positionClamp;
  }

  _logBodyReference[0] = _desPos.x();
  _logBodyReference[1] = _desPos.y();
  _logBodyReference[2] = _desPos.z();
  _logBodyReference[3] = _desYaw;
  _logBodyReference[4] = v.x();
  _logBodyReference[5] = v.y();
  _logBodyReference[6] = v.z();
  _logBodyReference[7] = _yawRateCommand();
  _logBodyReference[8] = _yawUnwrapped;
  _logBodyReference[9] = _speedScale;
  _logBodyReference[10] = _liftScale;
  _logBodyReference[11] = _yawClamp;
}

bool MdlTrot::_solveMPC(double elapsed) {
  const MdlConvexMPC::params_t& mp = _mpc->getParams();
  const double dtm = mp.dt;
  const Eigen::Vector3d& rcom = mp.com_offset_body;

  MdlConvexMPC::input_t in;
  in.current.rpy = Eigen::Vector3d(_rpy.x(), _rpy.y(), _yawUnwrapped);
  in.current.com_position_world = _comPos;
  in.current.angular_velocity_world = _omegaWorld;
  in.current.com_velocity_world = _comVel;

  double yaw = _desYaw;
  Eigen::Vector3d p = _desPos;

  // One scale for the whole horizon rather than one per step. The horizon is a
  // tenth of a second against a ramp measured in seconds, so anticipating the
  // ramp would move the last reference by a few percent of one step and buy
  // nothing; holding it fixed keeps the reference consistent with the body
  // reference integrated at the behavior rate.
  const Eigen::Vector3d vc = _twistCommand();
  const double yawRate = _yawRateCommand();

  for (int k = 0; k < mp.horizon; k++) {
    // contact[k] and moment_arm_world[k] describe the interval that *starts* at
    // now + k*dt, while reference[k] is the state at its end: the condensed
    // state vector stacks x_1 ... x_N and never contains x_0. The two indexings
    // are offset by one step by construction.
    const double tk = elapsed + k * dtm;
    const Eigen::Matrix3d Rz = _rotZ(yaw);

    for (int leg = 0; leg < NUM_LEGS; leg++) {
      in.contact[k][leg] = _gait.inStance(leg, tk);

      // TrotGait::sample() returns a delta from the nominal footprint and knows
      // nothing about where that footprint is; the composition with
      // _originFoot() is the same one the actual foot command uses. The stride
      // ramp is deliberately not applied to the future steps -- it scales a
      // couple of centimetres of moment arm against a hip offset of twenty.
      Eigen::Vector3d dp, dv;
      if (!_gait.inStance(leg, elapsed) && in.contact[k][leg] &&
          _swingTrajectory[leg].isValid() && tk >= _swingEnd[leg]) {
        dp = _swingTrajectory[leg].endPosition() - _originFoot(leg) +
             _swingTrajectory[leg].endVelocity() * (tk - _swingEnd[leg]);
        dv = _swingTrajectory[leg].endVelocity();
      } else {
        _gait.sample(leg, tk, _stanceVelocity(leg), dp, dv);
      }
      in.moment_arm_world[k].col(leg) = Rz * (_originFoot(leg) + dp - rcom);
    }

    yaw += yawRate * dtm;
    const Eigen::Vector3d v = _rotZ(yaw) * Eigen::Vector3d(vc.x(), vc.y(), 0.0);
    p.x() += v.x() * dtm;
    p.y() += v.y() * dtm;

    in.reference[k].rpy = Eigen::Vector3d(0.0, 0.0, yaw);
    // Height is the COM height captured at TROT entry, which is whatever the
    // nominal footprint left the robot at after PREP. Deriving it instead would
    // mean duplicating the foot radius and ground height assumptions that
    // already live in the estimator.
    in.reference[k].com_position_world = Eigen::Vector3d(p.x(), p.y(), _desPos.z());
    in.reference[k].angular_velocity_world = Eigen::Vector3d(0.0, 0.0, yawRate);
    in.reference[k].com_velocity_world = v;
  }

  // Paper Section IV-C: the first dynamics matrix has to describe where the
  // feet actually are. A tracking error or a disturbance moves a planted foot
  // away from the schedule, and a moment arm taken from the schedule then
  // attributes the resulting moment to the wrong place for the one interval
  // whose force is actually applied.
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    if (!in.contact[0][leg]) continue;
    Eigen::Vector3d pfoot;
    if (!_legs[leg]->getFootPosition(pfoot) || !pfoot.allFinite()) continue;
    in.moment_arm_world[0].col(leg) = _Rbw * (pfoot - rcom);
  }

  MdlConvexMPC::output_t out;
  if (_mpc->solve(in, out)) {
    _mpcOut = out;
    _mpcHave = true;
    _mpcFailures = 0;
    return true;
  }

  // Hold the last valid force for a small number of solves and no more. There
  // is no position-controlled stance to fall back to any more, and pretending
  // otherwise would put the robot on a controller nobody chose.
  _mpcFailures++;
  return _mpcHave && _mpcFailures <= _mpcFailureLimit;
}

// ---------------------------------------------------------------------------
//  Pose-holding states
// ---------------------------------------------------------------------------

void MdlTrot::_sendTarget() {
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  for (int i = 0; i < NUM_LEGS; i++) {
    // Only WAIT, PREP and CENTERING reach here, and all three keep four feet
    // planted, so there is one pair of joint gains rather than a stance/swing
    // split. TROT has its own dispatch and does not use this path at all.
    if (_legs[i]->setFootCommand(_footpos[i], _footvel[i], _prep_kp, _prep_kd, zero)) {
      _cmdFailures[i] = 0;
      continue;
    }

    // A rejected command leaves the previous one latched in MotorHW, so a
    // single miss is survivable. A run of them means the trajectory has left
    // the workspace and the gait is no longer being executed at all.
    if (++_cmdFailures[i] >= _cmdFailureLimit) {
      _mgr->warning(TROTMODULE_NAME,
                    "Leg %d command rejected %d cycles running, target=[%.4f %.4f %.4f]",
                    i, _cmdFailures[i], _footpos[i][0], _footpos[i][1], _footpos[i][2]);
      _status = ERROR;
    }
  }
}

bool MdlTrot::_checkTrackingError() const {
  MotorHW* hw = MotorHW::instance();
  if (!hw) return true;

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int j = 0; j < 3; j++) {
      const int idx = leg * 3 + j;
      MotorHW::state_t state;
      MotorHW::cmd_t cmd;
      hw->getState(idx, state);
      hw->getCommand(idx, cmd);
      if (std::fabs(state.pos - cmd.pos) > _trackingErrorLimit) return false;
    }
  }
  return true;
}

void MdlTrot::_hold() {
  for (int i = 0; i < NUM_LEGS; i++) {
    _footpos[i] = _footpos_start[i];
    _footvel[i] = Eigen::Vector3d::Zero();
  }
  _sendTarget();
}

void MdlTrot::_prepEntry() {
  _mark = _mgr->readTime();

  // TrotGait::sample() returns a zero horizontal offset at zero stance velocity
  // and zero clearance at a phase boundary, and TROT starts with the sweep built
  // on a body at rest, so every leg's offset from the nominal footprint is zero
  // at the first TROT sample. Blending to the plain footprint therefore matches
  // TROT in both position and velocity, and the handoff has no discontinuity.
  // test_zero_command_steps_in_place asserts the gait half of that.
  for (int i = 0; i < NUM_LEGS; i++) {
    _footpos_end[i] = _originFoot(i);
    _stance[i] = true;
  }
}

void MdlTrot::_prepDuring() {
  const double t = _mgr->readTime();

  double sigma, sigma_dot;
  TrajectoryUtils::sampleQuintic((t - _mark) / _prep_duration, _prep_duration, sigma,
                                 sigma_dot);

  for (int i = 0; i < NUM_LEGS; i++) {
    const Eigen::Vector3d diff = _footpos_end[i] - _footpos_start[i];
    _footpos[i] = _footpos_start[i] + sigma * diff;
    _footvel[i] = sigma_dot * diff;
  }
  _sendTarget();
}

// ---------------------------------------------------------------------------
//  TROT
// ---------------------------------------------------------------------------

void MdlTrot::_trotEntry() {
  _mark = _mgr->readTime();
  _trot_mark = _mark;
  if (!_mpcSchedule.reset(_mgr->readClock(), _mpc->getParams().solve_period)) {
    _mgr->warning(TROTMODULE_NAME, "Invalid MPC solve period");
    _status = ERROR;
    return;
  }

  _mpcOut = MdlConvexMPC::output_t();
  _mpcHave = false;
  _mpcFailures = 0;
  _vfiltValid = false;
  _haveYaw = false;

  // The command starts at zero, so the first solve is asked to stand still. That
  // is the point: the alternative is a solver told to be at the full commanded
  // speed on the cycle the gait starts, which it can only answer with ground
  // reaction force at feet that have not begun to sweep.
  _speedScale = 0.0;
  _liftScale = 1.0;
  _liftScaleDot = 0.0;
  _liftScaleDDot = 0.0;

  // Torque-controlled stance with no state estimate is a robot falling over in
  // a controlled fashion, so this is where the behavior refuses rather than
  // where it improvises.
  if (!_readBodyState()) {
    _mgr->warning(TROTMODULE_NAME,
                  "State estimate not available; refusing to start MPC stance");
    _status = ERROR;
    return;
  }

  // The reference starts wherever PREP left the robot, which is the height the
  // nominal footprint produces, and on the heading it is already facing.
  _desPos = _comPos;
  _desYaw = _yawUnwrapped;

  for (int i = 0; i < NUM_LEGS; i++) {
    _stance[i] = _gait.inStance(i, 0.0);
    _mpcMask[i] = _stance[i];
    _footacc[i].setZero();
    if (!_stance[i]) {
      const double phi = _gait.legPhase(i, 0.0);
      const double beta = _gait.getParams().duty;
      const double progress = (phi - beta) / (1.0 - beta);
      const double remaining =
          (1.0 - progress) * (1.0 - beta) * _gait.getParams().period;
      if (!_startSwing(i, 0.0, remaining)) {
        _mgr->warning(TROTMODULE_NAME,
                      "Cannot initialize swing trajectory for leg %d", i);
        _status = ERROR;
        return;
      }
    }
  }

  if (!_solveMPC(0.0)) {
    _mgr->warning(TROTMODULE_NAME, "First MPC solve failed (status %d); not trotting",
                  _mpcOut.solver_status);
    _status = ERROR;
  }
}

void MdlTrot::_updateGaitScales(double elapsed) {
  if (_state == _state_t::STOPPING) {
    // sigma runs 0 -> 1 over the ramp-down, so both scales are 1 - sigma. The
    // speed starts from wherever the ramp-up had got to rather than from one,
    // so a stop asked for mid-ramp decelerates instead of stepping up first.
    double sigma, sigma_dot, sigma_ddot;
    TrajectoryUtils::sampleQuintic((_mgr->readTime() - _stop_mark) / _rampdown_duration,
                                   _rampdown_duration, sigma, sigma_dot, sigma_ddot);
    _speedScale = _stopSpeedScale0 * (1.0 - sigma);
    _liftScale = 1.0 - sigma;
    _liftScaleDot = -sigma_dot;
    _liftScaleDDot = -sigma_ddot;
    return;
  }

  // Clearance stays at full through the ramp-up. The gait, the contact schedule
  // and the MPC's contact mask are then all real from the first cycle, and the
  // robot steps in place until the command has something to ask for.
  double scale_dot;
  TrajectoryUtils::sampleQuintic(elapsed / _rampup_duration, _rampup_duration, _speedScale,
                                 scale_dot);
  _liftScale = 1.0;
  _liftScaleDot = 0.0;
  _liftScaleDDot = 0.0;
}

bool MdlTrot::_startSwing(int leg, double elapsed, double duration) {
  Eigen::Vector3d position, velocity;
  if (!_legs[leg]->getFootState(position, velocity)) return false;
  const Eigen::Vector3d stanceVelocity = _stanceVelocity(leg);
  const double stanceTime = _gait.getParams().duty * _gait.getParams().period;
  const Eigen::Vector3d touchdown =
      _originFoot(leg) - 0.5 * stanceTime * stanceVelocity;
  if (!_swingTrajectory[leg].reset(position, velocity, touchdown, stanceVelocity,
                                   duration, _gait.getParams().swing_height))
    return false;
  _swingStart[leg] = elapsed;
  _swingEnd[leg] = elapsed + duration;
  return true;
}

void MdlTrot::_stoppingEntry() {
  _stop_mark = _mgr->readTime();
  _stopSpeedScale0 = _speedScale;
}

void MdlTrot::_trotDuring() {
  const double t = _mgr->readTime();
  const CLOCK now = _mgr->readClock();
  const double elapsed = t - _trot_mark;

  // Once per cycle, before any leg is sampled, so all four are swept against
  // the same velocity and the MPC sees one consistent body state.
  if (!_readBodyState()) {
    _mgr->warning(TROTMODULE_NAME, "State estimate lost at t=%.3f", t);
    _status = ERROR;
    return;
  }

  _updateGaitScales(elapsed);

  bool maskChanged = false;
  for (int i = 0; i < NUM_LEGS; i++) {
    const bool scheduledStance = _gait.inStance(i, elapsed);
    if (!scheduledStance && _stance[i]) {
      const double swingTime =
          (1.0 - _gait.getParams().duty) * _gait.getParams().period;
      if (!_startSwing(i, elapsed, swingTime)) {
        _mgr->warning(TROTMODULE_NAME, "Cannot start swing trajectory for leg %d", i);
        _status = ERROR;
        return;
      }
    }

    if (scheduledStance) {
      Eigen::Vector3d dp, dv;
      _gait.sample(i, elapsed, _stanceVelocity(i), dp, dv);
      _footpos[i] = _originFoot(i) + dp;
      _footvel[i] = dv;
      _footacc[i].setZero();
    } else {
      _swingTrajectory[i].sample(elapsed - _swingStart[i], _liftScale,
                                 _liftScaleDot, _liftScaleDDot, _footpos[i],
                                 _footvel[i], _footacc[i]);
    }

    _stance[i] = scheduledStance;
    _logContact[i] = scheduledStance ? 1.0 : 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      _logFootReference[9 * i + axis] = _footpos[i][axis];
      _logFootReference[9 * i + 3 + axis] = _footvel[i][axis];
      _logFootReference[9 * i + 6 + axis] = _footacc[i][axis];
    }
    if (_stance[i] != _mpcMask[i]) maskChanged = true;
  }

  _integrateBodyReference();

  // At the configured cadence, and immediately on a contact change. The second
  // trigger is what stops a leg that has just landed from waiting up to a full
  // solve interval for a force, which is what happens whenever the gait period
  // and the solve period are not commensurate. It matters more, not less, as the
  // horizon step grows: mpc.solve_period is deliberately not mpc.dt.
  const bool deadlineDue = _mpcSchedule.due(now);
  if (maskChanged || deadlineDue) {
    _mpcSchedule.consumed(now, maskChanged && !deadlineDue);
    for (int i = 0; i < NUM_LEGS; i++) _mpcMask[i] = _stance[i];

    if (!_solveMPC(elapsed)) {
      _mgr->warning(TROTMODULE_NAME, "MPC failed %d solves running (status %d) at t=%.3f",
                    _mpcFailures, _mpcOut.solver_status, t);
      _status = ERROR;
      return;
    }
  }

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const Eigen::Vector3d gravityBody =
      _Rbw.transpose() * Eigen::Vector3d(0.0, 0.0, -_mpc->getParams().gravity);
  for (int i = 0; i < NUM_LEGS; i++) {
    bool ok;
    MdlLegControl::command_result_t command;

    if (!_stance[i]) {
      if (_swingInverseDynamics && _legs[i]->hasSwingDynamics()) {
        ok = _legs[i]->setSwingCommand(_footpos[i], _footvel[i], _footacc[i],
                                       gravityBody, _swingNaturalFrequency,
                                       _swingDampingRatio, _swingFeedforwardScale,
                                       _jointDamping, &command);
      } else {
        ok = _legs[i]->setCartesianForceCommand(_footpos[i], _footvel[i], _swing_kp,
                                                _swing_kd, zero, _jointDamping,
                                                &command);
      }
    } else {
      // Convert the MPC's world-frame ground reaction into the force the leg
      // applies at its foot.  The static MuJoCo sign regression covers this
      // convention; _mpcOut may be the latest retained successful solve.
      const Eigen::Vector3d f_foot_B = -(_Rbw.transpose() * _mpcOut.force_world[i]);
      ok = _legs[i]->setCartesianForceCommand(_footpos[i], _footvel[i], zero, zero,
                                              f_foot_B, _jointDamping, &command);
    }

    if (ok) {
      _cmdFailures[i] = 0;
      _logTorqueScale[i] = command.torque_scale;
      for (int axis = 0; axis < 3; axis++) {
        _logTorqueRequest[3 * i + axis] = command.requested_torque[axis];
      }
      if (command.torque_scale < 0.999999) {
        if (++_torqueSaturationCycles[i] >= _torqueSaturationLimit) {
          _mgr->warning(TROTMODULE_NAME,
                        "Leg %d torque limited for %d cycles (scale %.3f)", i,
                        _torqueSaturationCycles[i], command.torque_scale);
          _status = ERROR;
        }
      } else {
        _torqueSaturationCycles[i] = 0;
      }
      continue;
    }

    // In TROT there is no IK to fail: a rejection means the motors are not
    // ready or the computed torque is not finite. Either way the leg is not
    // executing what the controller asked for.
    if (++_cmdFailures[i] >= _cmdFailureLimit) {
      _mgr->warning(TROTMODULE_NAME, "Leg %d force command rejected %d cycles running", i,
                    _cmdFailures[i]);
      _status = ERROR;
    }
  }
}

// ---------------------------------------------------------------------------
//  CENTERING
// ---------------------------------------------------------------------------

void MdlTrot::_centeringEntry() {
  _mark = _mgr->readTime();

  for (int i = 0; i < NUM_LEGS; i++) {
    // Start the blend from where the foot actually is, not from where the
    // trajectory reference says it should be. Under force control a stance leg
    // does not track a position at all, so the reference can be centimetres
    // away and the first centering command would be a step.
    Eigen::Vector3d measured;
    if (_legs[i]->getFootPosition(measured) && measured.allFinite())
      _center_pos0[i] = measured;
    else
      _center_pos0[i] = _footpos[i];

    // Velocity still comes from the reference: there is no measured foot
    // velocity available here, and decelerating out of the commanded one is
    // what keeps the stop free of a velocity kink.
    _center_vel0[i] = _footvel[i];
    _center_pos1[i] = _originFoot(i);
    _footpos[i] = _center_pos0[i];
    // Coming to rest on all four feet, so every leg gets support gains.
    _stance[i] = true;
  }
}

void MdlTrot::_centeringDuring() {
  double tau = (_mgr->readTime() - _mark) / _centering_duration;
  if (tau < 0.0) tau = 0.0;
  if (tau > 1.0) tau = 1.0;

  // Cubic Hermite between (start pos, start vel) and (end pos, zero vel).
  // The h11 term drops out since the end velocity is zero.
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double h00 = 2.0 * tau3 - 3.0 * tau2 + 1.0;
  const double h10 = tau3 - 2.0 * tau2 + tau;
  const double h01 = -2.0 * tau3 + 3.0 * tau2;
  const double dh00 = 6.0 * tau2 - 6.0 * tau;
  const double dh10 = 3.0 * tau2 - 4.0 * tau + 1.0;
  const double dh01 = -6.0 * tau2 + 6.0 * tau;

  const double T = _centering_duration;
  for (int i = 0; i < NUM_LEGS; i++) {
    _footpos[i] = h00 * _center_pos0[i] + h10 * _center_vel0[i] * T + h01 * _center_pos1[i];
    _footvel[i] =
        (dh00 * _center_pos0[i] + dh10 * _center_vel0[i] * T + dh01 * _center_pos1[i]) / T;
  }
  _sendTarget();
}

void MdlTrot::stopTrotting() {
  // No-op if we are already stopping or stopped
  if (_state == _state_t::STOPPING || _state == _state_t::CENTERING ||
      _state == _state_t::DONE)
    return;

  // A running gait is decelerated first, so the body is at rest and every foot
  // is back on the nominal footprint before a pose controller takes over.
  // WAIT and PREP have no gait to ramp and are already holding a pose, so they
  // go straight to the centering blend as before.
  if (_state == _state_t::TROT) {
    _state = _state_t::STOPPING;
    _stoppingEntry();
    return;
  }

  _state = _state_t::CENTERING;
  _centeringEntry();
}

void MdlTrot::update() {
  if (_status == ERROR) return;

  const double t = _mgr->readTime();

  switch (_state) {
    case _state_t::WAIT:
      if (t - _mark > _wait_duration) {
        _state = _state_t::PREP;
        _prepEntry();
        break;
      }
      _hold();
      break;

    case _state_t::PREP:
      if (t - _mark > _prep_duration) {
        _state = _state_t::TROT;
        _trotEntry();
        break;
      }
      _prepDuring();
      break;

    case _state_t::TROT:
      _trotDuring();
      break;

    case _state_t::STOPPING:
      // Same controller as TROT, with the command ramping to zero underneath it.
      // At the end the clearance is zero and the sweep has decayed with the body
      // velocity, so every foot is on the nominal footprint and the centering
      // blend has almost nothing left to do.
      if (t - _stop_mark > _rampdown_duration) {
        _state = _state_t::CENTERING;
        _centeringEntry();
        break;
      }
      _trotDuring();
      break;

    case _state_t::CENTERING:
      if (t - _mark > _centering_duration) {
        _state = _state_t::DONE;
        _status = SETTLED;
        break;
      }
      _centeringDuring();
      break;

    case _state_t::DONE:
      // Hold the centered pose. The last command from CENTERING stays latched
      // in MotorHW, so there is nothing to send.
      break;
  }

  // Joint tracking error only means something where a joint is being asked to
  // reach a position. In TROT and STOPPING the joint kp is zero by construction
  // and the measured position is fed back as the command, so the error is
  // identically zero and the check would be vacuous; the motor-ready,
  // finite-torque and solver checks above cover those states instead.
  if (_status == ACTIVE && _state != _state_t::WAIT && _state != _state_t::TROT &&
      _state != _state_t::STOPPING && !_checkTrackingError()) {
    _mgr->warning(TROTMODULE_NAME, "Tracking error exceeded at t=%.3f", t);
    _status = ERROR;
  }

  _logScales[0] = _speedScale;
  _logScales[1] = _liftScale;
}
