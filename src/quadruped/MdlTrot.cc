/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include <cmath>
#include <cstdio>

#include "hardware/MotorHW.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/MdlPosVelEstimator.hh"
#include "quadruped/MdlTrot.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/TrajectoryUtils.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/ModuleManager.hh"

using namespace rtcore;

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...)  // printf(__VA_ARGS__)

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

  // Optional, and read only: the estimator is created before this module in
  // AddCoreModules, runs at SENSING_MODULES so its answer is from this cycle,
  // and is MULTI_USER, so there is nothing to grab and nobody to contend with.
  // A null pointer simply leaves the gait open loop.
  _posvel = (MdlPosVelEstimator*)_mgr->findModule(POSVELMODULE_NAME, 0);

  ConfigTable hwConfig;
  std::string hwlib;
  if (_mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");

  if (hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

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

  ConfigTable vfb;
  if (config.getTable("velocity_feedback", vfb)) {
    _vfbEnable = vfb.getBool("enable", _vfbEnable);
    _vfbGain = vfb.getDouble("gain", _vfbGain);
    _vfbTau = vfb.getDouble("tau", _vfbTau);
    _vfbLimit = vfb.getDouble("limit", _vfbLimit);

    if (!(_vfbGain >= 0.0)) _vfbGain = 0.0;
    if (_vfbGain > 1.0) _vfbGain = 1.0;
    // A zero time constant would make the filter a passthrough of a 1 kHz
    // signal, which is exactly what it is there to avoid.
    if (!(_vfbTau > 0.0)) _vfbTau = 0.01;
    if (!(_vfbLimit >= 0.0)) _vfbLimit = 0.0;
  }

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
  _centering_duration = config.getDouble("centering_duration", _centering_duration);

  // Zero or negative durations would divide by zero in the blend math
  if (!(_wait_duration >= 0.0)) _wait_duration = 0.0;
  if (!(_prep_duration > 0.0)) _prep_duration = 0.01;
  if (!(_rampup_duration > 0.0)) _rampup_duration = 0.01;
  if (!(_centering_duration > 0.0)) _centering_duration = 0.01;

  _stance_kp = Eigen::Vector3d::Constant(config.getDouble("stance_kp", _stance_kp.x()));
  _stance_kd = Eigen::Vector3d::Constant(config.getDouble("stance_kd", _stance_kd.x()));
  _swing_kp = Eigen::Vector3d::Constant(config.getDouble("swing_kp", _swing_kp.x()));
  _swing_kd = Eigen::Vector3d::Constant(config.getDouble("swing_kd", _swing_kd.x()));

  _trackingErrorLimit = config.getDouble("tracking_error_limit", _trackingErrorLimit);
  _ikFailureLimit = (int)config.getInt("ik_failure_limit", _ikFailureLimit);
  if (_ikFailureLimit < 1) _ikFailureLimit = 1;
}

void MdlTrot::uninit() {
  DBGPRINT("MdlTrot::uninit\n");
  delete _kinematics;
  _kinematics = nullptr;
}

void MdlTrot::activate() {
  DBGPRINT("MdlTrot::activate\n");

  _status = ACTIVE;
  _state = _state_t::WAIT;
  _mark = _mgr->readTime();

  for (int i = 0; i < NUM_LEGS; i++) {
    _ikFailures[i] = 0;
    _stance[i] = true;
    _mgr->grabModule(_legs[i], this);
  }

  // Grab all legs first, then capture FK. If any capture fails, release
  // everything before reporting the error.
  for (int i = 0; i < NUM_LEGS; i++) {
    if (!_legs[i]->getFootPosition(_footpos_start[i])) {
      _mgr->warning(TROTMODULE_NAME, "Failed to capture foot %d position", i);
      for (int j = 0; j < NUM_LEGS; j++) _mgr->releaseModule(_legs[j], this);
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
}

bool MdlTrot::getStancePhase(double phase[NUM_LEGS]) const {
  // Only the TROT state runs the schedule. Every other state holds all four
  // feet down, which the caller represents itself; saying so here by returning
  // false keeps this function honest about what it actually knows.
  if (_state != _state_t::TROT) return false;

  const double elapsed = _mgr->readTime() - _trot_mark;
  for (int i = 0; i < NUM_LEGS; i++) phase[i] = _gait.stanceProgress(i, elapsed);
  return true;
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

void MdlTrot::_updateVelocityFeedback() {
  _vfbActive = false;
  if (!_vfbEnable || !_posvel) return;

  Eigen::Vector3d v;
  if (!_posvel->getBodyVelocityInBody(v) || !v.allFinite()) return;

  // Low pass before use. The estimate is refreshed every millisecond and
  // carries the trunk's own bounce and sway at the gait frequency; feeding that
  // straight into the sweep would modulate the stride within a single stance.
  // A time constant well under the gait period tracks a genuine change in speed
  // while leaving the per-stride oscillation behind.
  const double dt = CLOCK_TO_SEC(_mgr->getStepPeriod());
  const double a = dt / (_vfbTau + dt);
  _vfilt += a * (v - _vfilt);
  _vfbActive = true;
}

Eigen::Vector3d MdlTrot::_sweepVelocity() const {
  if (!_vfbActive) return _vcmd;

  // Blend from the command toward the estimate, then clamp. The clamp is the
  // part that matters: this is a positive feedback path, since a velocity
  // estimate that is too low shortens the stride, which slows the robot, which
  // lowers the estimate again. Bounding the correction keeps a bad estimate
  // from walking the stride away entirely.
  Eigen::Vector3d correction = _vfbGain * (_vfilt - _vcmd);
  for (int i = 0; i < 3; i++) {
    if (correction[i] > _vfbLimit) correction[i] = _vfbLimit;
    if (correction[i] < -_vfbLimit) correction[i] = -_vfbLimit;
  }
  // Horizontal only. The trunk really does rise and fall during a trot, but
  // that is the gait working, not a tracking error, and sweeping the stance
  // feet vertically to chase it would fight the stance height instead.
  correction.z() = 0.0;

  return _vcmd + correction;
}

Eigen::Vector3d MdlTrot::_stanceVelocity(int leg) const {
  // Velocity of a planted foot in the body frame is the negated body twist
  // evaluated at that foot. The cross product term is what swings the footprint
  // around for a turn; it vanishes when yaw_rate is zero.
  const Eigen::Vector3d omega(0.0, 0.0, _yawRate);
  return -(_sweepVelocity() + omega.cross(_originFoot(leg)));
}

void MdlTrot::_sendTarget() {
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  for (int i = 0; i < NUM_LEGS; i++) {
    // Stiff while supporting the body, soft while swinging. The soft swing also
    // keeps swing joint torques well clear of the estimator's contact detection
    // threshold, which reads ground reaction force through the leg Jacobian.
    const Eigen::Vector3d& kp = _stance[i] ? _stance_kp : _swing_kp;
    const Eigen::Vector3d& kd = _stance[i] ? _stance_kd : _swing_kd;

    if (_legs[i]->setFootCommand(_footpos[i], _footvel[i], kp, kd, zero)) {
      _ikFailures[i] = 0;
      continue;
    }

    // A rejected command leaves the previous one latched in MotorHW, so a
    // single miss is survivable. A run of them means the trajectory has left
    // the workspace and the gait is no longer being executed at all.
    if (++_ikFailures[i] >= _ikFailureLimit) {
      _mgr->warning(TROTMODULE_NAME,
                    "Leg %d command rejected %d cycles running, target=[%.4f %.4f %.4f]",
                    i, _ikFailures[i], _footpos[i][0], _footpos[i][1], _footpos[i][2]);
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

  // The stride ramp starts at zero, so every leg's offset from the nominal
  // footprint is zero at the first TROT sample whatever the duty factor is.
  // Blending to the plain footprint therefore matches TROT in both position
  // and velocity, and the handoff has no discontinuity.
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

void MdlTrot::_trotEntry() {
  _mark = _mgr->readTime();
  _trot_mark = _mark;

  // Seed the filter at the command so the first strides start from the open
  // loop behaviour and the feedback eases in, rather than the stride jumping
  // on the first cycle from whatever the robot happened to be doing in PREP.
  _vfilt = _vcmd;
  _vfbActive = false;
}

void MdlTrot::_trotDuring() {
  const double elapsed = _mgr->readTime() - _trot_mark;

  // Once per cycle, before any leg is sampled, so all four are swept against
  // the same velocity.
  _updateVelocityFeedback();

  // Stride grows from zero, so the robot steps in place before it accelerates.
  double ramp, ramp_dot;
  TrajectoryUtils::sampleQuintic(elapsed / _rampup_duration, _rampup_duration, ramp,
                                 ramp_dot);

  for (int i = 0; i < NUM_LEGS; i++) {
    Eigen::Vector3d dp, dv;
    _gait.sample(i, elapsed, _stanceVelocity(i), dp, dv);
    _stance[i] = _gait.inStance(i, elapsed);

    // Product rule: the ramp is a function of time as well, and dropping its
    // term would leave the commanded velocity inconsistent with the commanded
    // position for the whole ramp.
    _footpos[i] = _originFoot(i) + ramp * dp;
    _footvel[i] = ramp * dv + ramp_dot * dp;
  }
  _sendTarget();
}

void MdlTrot::_centeringEntry() {
  _mark = _mgr->readTime();

  // Capture where the reference trajectory is right now, velocity included.
  // That is what we decelerate out of, so the stop has no velocity kink.
  for (int i = 0; i < NUM_LEGS; i++) {
    _center_pos0[i] = _footpos[i];
    _center_vel0[i] = _footvel[i];
    _center_pos1[i] = _originFoot(i);
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
  if (_state == _state_t::CENTERING || _state == _state_t::DONE) return;
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

  if (_status == ACTIVE && _state != _state_t::WAIT && !_checkTrackingError()) {
    _mgr->warning(TROTMODULE_NAME, "Tracking error exceeded at t=%.3f", t);
    _status = ERROR;
  }
}
