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

// Replace the macro body with printf(...) to enable debug messages.
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

  // Preserve the integer deadline lattice when a cycle arrives late.
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

  // Build a fifth-order Hermite polynomial with zero endpoint acceleration.
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

  // Clamp parameters that would make the phase or swing interval invalid.
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
  // Keep the wrapped phase strictly below 1.
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
    // Stance uses constant body-frame velocity u so the planted foot does not
    // slip.
    const double s = phi / beta;
    dp = u * stanceTime * (s - 0.5);
    dv = u;
    return;
  }

  // Swing uses matching endpoint tangents and closes continuously into stance.
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

  // Raised-cosine clearance has zero value and slope at both endpoints.
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

  // Estimators are shared read-only sensors; the force controller requires both.
  _orientation = (MdlOrientationEstimator*)_mgr->findModule(ORIENTATIONMODULE_NAME, 0);
  _posvel = (MdlPosVelEstimator*)_mgr->findModule(POSVELMODULE_NAME, 0);
  if (!_orientation || !_posvel)
    _mgr->fatalError(TROTMODULE_NAME,
                     "%s and %s must be created before %s in AddCoreModules()",
                     ORIENTATIONMODULE_NAME, POSVELMODULE_NAME, TROTMODULE_NAME);

  // MPC is single-user state and is grabbed with the leg controllers.
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

  // Register ramp, reference, contact, torque, and body-reference logs.
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
  // Nonpositive tau disables the intended velocity filtering; use a positive
  // fallback.
  if (!(_velocityFilterTau > 0.0)) _velocityFilterTau = 0.01;
  _stanceVelocityFeedback =
      config.getDouble("stance_velocity_feedback", _stanceVelocityFeedback);
  if (!(_stanceVelocityFeedback >= 0.0)) _stanceVelocityFeedback = 0.0;
  if (_stanceVelocityFeedback > 1.0) _stanceVelocityFeedback = 1.0;

  _raibertGain = config.getDouble("raibert_gain", _raibertGain);
  if (!(_raibertGain >= 0.0)) _raibertGain = 0.0;
  _centrifugalTerm = config.getBool("centrifugal_term", _centrifugalTerm);

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

  // Nonpositive durations would make the blend invalid.
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
  // Keep joint damping positive; MotorHW substitutes a large fallback otherwise.
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
  // Grab the solver with the legs so its warm start remains owned by this behavior.
  _mgr->grabModule(_mpc, this);

  // Capture all foot states after grabbing modules; release all modules on failure.
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

  // Preserve status for Supervisor inspection while releasing owned modules.
  for (int i = 0; i < NUM_LEGS; i++) _mgr->releaseModule(_legs[i], this);
  _mgr->releaseModule(_mpc, this);
}

bool MdlTrot::getStancePhase(double phase[NUM_LEGS]) const {
  // Only TROT and STOPPING have a gait schedule; other states are fully planted.
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
  // Build the nominal body-frame foot position from the hip and mirrored lateral
  // offset.
  Eigen::Vector3d p = _kinematics->getKinematicParams().hip_positions.row(leg).transpose();
  p[0] += _origin[0];
  p[1] += _origin[1] * (leg % 2 == 0 ? 1.0 : -1.0);
  p[2] += _origin[2];
  return p;
}

Eigen::Vector3d MdlTrot::_sweepVelocity() const {
  const Eigen::Vector3d command = _twistCommand();
  if (!_vfiltValid) return command;

  // Blend the filtered body velocity toward the command with a bounded correction.
  return command + _stanceVelocityFeedback * (_vfilt - command);
}

Eigen::Vector3d MdlTrot::_stanceVelocity(int leg) const {
  // Negate body twist at this foot; yaw rate adds the turn-induced cross product.
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

  // Convert the body-origin estimate to COM using the configured body-frame
  // offset.
  const Eigen::Vector3d r = _Rbw * _mpc->getParams().com_offset_body;
  _comPos = pBody + r;
  _comVel = vBody + _omegaWorld.cross(r);

  // Keep yaw continuous so current/reference subtraction does not cross a 2*pi
  // wrap.
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

  // Low-pass body-frame COM velocity for touchdown geometry.
  const Eigen::Vector3d vb = _Rbw.transpose() * _comVel;
  if (!_vfiltValid) {
    _vfilt = vb;
    _vfiltValid = true;
  } else {
    const double dt = CLOCK_TO_SEC(_mgr->getStepPeriod());
    const double a = dt / (_velocityFilterTau + dt);
    _vfilt += a * (vb - _vfilt);
  }
  // Use horizontal velocity for stance sweep and touchdown feedback.
  _vfilt.z() = 0.0;

  _stateValid = true;
  return true;
}

void MdlTrot::_integrateBodyReference() {
  const double dt = CLOCK_TO_SEC(_mgr->getStepPeriod());

  // Rotate the body-frame command by desired yaw before integrating world
  // position.
  const Eigen::Vector3d vc = _twistCommand();
  _desYaw += _yawRateCommand() * dt;

  // Bound yaw error so the reference cannot drift away from the measured heading.
  const double yawErr = _desYaw - _yawUnwrapped;
  if (yawErr > _yawClamp) _desYaw = _yawUnwrapped + _yawClamp;
  if (yawErr < -_yawClamp) _desYaw = _yawUnwrapped - _yawClamp;

  const Eigen::Vector3d v = _rotZ(_desYaw) * Eigen::Vector3d(vc.x(), vc.y(), 0.0);
  _desPos.x() += v.x() * dt;
  _desPos.y() += v.y() * dt;

  // Bound horizontal position error relative to the estimated COM.
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

  // Use one command scale for all horizon steps so references share one
  // operating point.
  const Eigen::Vector3d vc = _twistCommand();
  const double yawRate = _yawRateCommand();

  for (int k = 0; k < mp.horizon; k++) {
    // Contact and moment-arm samples describe interval k; reference[k] is its
    // end state x(k+1).
    const double tk = elapsed + k * dtm;
    const Eigen::Matrix3d Rz = _rotZ(yaw);

    for (int leg = 0; leg < NUM_LEGS; leg++) {
      in.contact[k][leg] = _gait.inStance(leg, tk);

      // Compose the gait offset with the nominal footprint used by leg commands.
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
    // Hold the COM height captured at TROT entry.
    in.reference[k].com_position_world = Eigen::Vector3d(p.x(), p.y(), _desPos.z());
    in.reference[k].angular_velocity_world = Eigen::Vector3d(0.0, 0.0, yawRate);
    in.reference[k].com_velocity_world = v;
  }

  // Use measured stance-foot positions for interval zero so applied forces use
  // current moment arms.
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

  // Retain the last valid force only up to the configured consecutive-failure
  // limit.
  _mpcFailures++;
  return _mpcHave && _mpcFailures <= _mpcFailureLimit;
}

// ---------------------------------------------------------------------------
//  Pose-holding states
// ---------------------------------------------------------------------------

void MdlTrot::_sendTarget() {
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  for (int i = 0; i < NUM_LEGS; i++) {
    // WAIT, PREP, and CENTERING hold all feet through the pose-control path.
    if (_legs[i]->setFootCommand(_footpos[i], _footvel[i], _prep_kp, _prep_kd, zero)) {
      _cmdFailures[i] = 0;
      continue;
    }

    // MotorHW retains the prior command on rejection; escalate after the
    // configured consecutive-failure limit.
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

  // PREP ends at the nominal footprint, matching the zero-velocity first TROT
  // sample.
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

  // Start with zero command scale; ramp to the configured twist in TROT.
  _speedScale = 0.0;
  _liftScale = 1.0;
  _liftScaleDot = 0.0;
  _liftScaleDDot = 0.0;

  // Refuse torque-controlled stance without a valid body state.
  if (!_readBodyState()) {
    _mgr->warning(TROTMODULE_NAME,
                  "State estimate not available; refusing to start MPC stance");
    _status = ERROR;
    return;
  }

  // Initialize the reference from the current COM and unwrapped heading.
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
    // Both scales follow the quintic ramp down; speed starts from its captured
    // value at stop request.
    double sigma, sigma_dot, sigma_ddot;
    TrajectoryUtils::sampleQuintic((_mgr->readTime() - _stop_mark) / _rampdown_duration,
                                   _rampdown_duration, sigma, sigma_dot, sigma_ddot);
    _speedScale = _stopSpeedScale0 * (1.0 - sigma);
    _liftScale = 1.0 - sigma;
    _liftScaleDot = -sigma_dot;
    _liftScaleDDot = -sigma_ddot;
    return;
  }

  // Keep clearance full while speed scale rises during ramp-up.
  double scale_dot;
  TrajectoryUtils::sampleQuintic(elapsed / _rampup_duration, _rampup_duration, _speedScale,
                                 scale_dot);
  _liftScale = 1.0;
  _liftScaleDot = 0.0;
  _liftScaleDDot = 0.0;
}

Eigen::Vector3d MdlTrot::touchdownFeedback(const Eigen::Vector3d& vMeasured,
                                           const Eigen::Vector3d& vCommanded,
                                           double raibertGain, double omega_z,
                                           double height, double gravity,
                                           bool centrifugalEnable) {
  // Velocity error shifts touchdown; a slower measured body places the foot
  // farther back.
  Eigen::Vector3d fb = raibertGain * (vMeasured - vCommanded);
  fb.z() = 0.0;

  if (centrifugalEnable) {
    // Add optional yaw-rate centrifugal skew.
    const Eigen::Vector3d omega(0.0, 0.0, omega_z);
    fb += 0.5 * std::sqrt(height / gravity) * vMeasured.cross(omega);
  }
  return fb;
}

bool MdlTrot::_startSwing(int leg, double elapsed, double duration) {
  Eigen::Vector3d position, velocity;
  if (!_legs[leg]->getFootState(position, velocity)) return false;
  const Eigen::Vector3d stanceVelocity = _stanceVelocity(leg);
  const double stanceTime = _gait.getParams().duty * _gait.getParams().period;
  // Touchdown is body-frame and uses the nominal footprint plus stance and
  // feedback offsets.
  const Eigen::Vector3d touchdown =
      _originFoot(leg) - 0.5 * stanceTime * stanceVelocity +
      touchdownFeedback(_vfilt, _twistCommand(), _raibertGain, _yawRateCommand(),
                        std::fabs(_origin[2]), _mpc->getParams().gravity, _centrifugalTerm);
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

  // Read one consistent body state before sampling all legs.
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

  // Solve at the deadline or immediately after a contact-mask change.
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
      // Convert the MPC world-frame ground reaction to the body-frame foot
      // force; _mpcOut may be the latest retained successful solve.
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

    // A rejected TROT command means the motors are not ready or the torque is
    // invalid; escalate after the configured consecutive-failure limit.
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
    // Start from the measured foot position so centering does not introduce a
    // position step after force control.
    Eigen::Vector3d measured;
    if (_legs[i]->getFootPosition(measured) && measured.allFinite())
      _center_pos0[i] = measured;
    else
      _center_pos0[i] = _footpos[i];

    // Use reference velocity because foot velocity is not measured.
    _center_vel0[i] = _footvel[i];
    _center_pos1[i] = _originFoot(i);
    _footpos[i] = _center_pos0[i];
    // Center all legs with support gains.
    _stance[i] = true;
  }
}

void MdlTrot::_centeringDuring() {
  double tau = (_mgr->readTime() - _mark) / _centering_duration;
  if (tau < 0.0) tau = 0.0;
  if (tau > 1.0) tau = 1.0;

  // Cubic Hermite from current position/velocity to nominal position with zero
  // final velocity.
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
  // Ignore repeated stop requests after STOPPING, CENTERING, or DONE.
  if (_state == _state_t::STOPPING || _state == _state_t::CENTERING ||
      _state == _state_t::DONE)
    return;

  // TROT ramps down before centering; other pose states center directly.
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
      // Continue TROT control while speed and clearance ramp down.
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
      // Hold the centered pose; CENTERING's last command remains latched.
      break;
  }

  // Check joint position tracking only in pose-holding states; TROT and STOPPING
  // validate torque, motor, and solver paths separately.
  if (_status == ACTIVE && _state != _state_t::WAIT && _state != _state_t::TROT &&
      _state != _state_t::STOPPING && !_checkTrackingError()) {
    _mgr->warning(TROTMODULE_NAME, "Tracking error exceeded at t=%.3f", t);
    _status = ERROR;
  }

  _logScales[0] = _speedScale;
  _logScales[1] = _liftScale;
}
