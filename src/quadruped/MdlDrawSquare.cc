/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */


#include <math.h>
#include <stdio.h>

#include "rtcore/ConfigTable.hh"
#include "rtcore/ModuleManager.hh"
#include "rtcore/Profiler.hh"
#include "quadruped/MdlDrawSquare.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedKinematics.hh"

using namespace rtcore;

#include "quadruped/QuadrupedConfigs.hh"

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...) printf(__VA_ARGS__)

MdlDrawSquare::MdlDrawSquare() : Module(WALKMODULE_NAME, 0, SINGLE_USER) {
  DBGPRINT("MdlDrawSquare::MdlDrawSquare\n");
};

MdlDrawSquare::~MdlDrawSquare() { DBGPRINT("MdlDrawSquare::~MdlDrawSquare\n"); };

void MdlDrawSquare::init() {
  DBGPRINT("MdlDrawSquare::init\n");

  for (int l = 0; l < 4; l++)
    _legs[l] = (MdlLegControl*)_mgr->findModule(LEGMODULE_NAME, l);

  _kinematics = new QuadrupedKinematics(createGo2Config());

  ConfigTable config;
  bool hasConfig = _mgr->getConfigTable("drawsquare", config);

  if (hasConfig) {
    _sq_period = config.getDouble("period", _sq_period);
    _sq_xedge = config.getDouble("xedge", _sq_xedge);
    _sq_yedge = config.getDouble("yedge", _sq_yedge);
    _wait_duration = config.getDouble("wait_duration", _wait_duration);
    _preplegs_duration = config.getDouble("preplegs_duration", _preplegs_duration);
    _centering_duration = config.getDouble("centering_duration", _centering_duration);

    // Zero / negative durations would divide-by-zero in the blend math
    if (_preplegs_duration <= 0.0) _preplegs_duration = 0.01;
    if (_centering_duration <= 0.0) _centering_duration = 0.01;

    ConfigArray origin;
    bool hasOrigin = config.getArray("origin", origin);
    if (hasOrigin) {
      _origin[0] = origin.getDoubleAt(0, -0.05);
      _origin[1] = origin.getDoubleAt(1, 0.12);
      _origin[2] = origin.getDoubleAt(2, -0.26);
    }
  }

  _fpos.push_back(Eigen::Vector3d(_sq_xedge / 2, _sq_yedge / 2, 0.0));
  _fpos.push_back(Eigen::Vector3d(_sq_xedge / 2, -_sq_yedge / 2, 0.0));
  _fpos.push_back(Eigen::Vector3d(-_sq_xedge / 2, -_sq_yedge / 2, 0.0));
  _fpos.push_back(Eigen::Vector3d(-_sq_xedge / 2, _sq_yedge / 2, 0.0));

  for (int i = 0; i < 3; i++) _profiler[i] = new Profiler();
}

void MdlDrawSquare::uninit() { DBGPRINT("MdlDrawSquare::uninit\n"); }

void MdlDrawSquare::activate() {
  DBGPRINT("MdlDrawSquare[%d]::activate\n", getIndex());

  for (int i = 0; i < 4; i++) _mgr->grabModule(_legs[i], this);

  _mark = _mgr->readTime();

  _state = _state_t::WAIT;
  _wait_entry();
}

void MdlDrawSquare::deactivate() {
  DBGPRINT("MdlDrawSquare::deactivate\n");

  for (int i = 0; i < 4; i++) _mgr->releaseModule(_legs[i], this);
}

// Event methods
bool MdlDrawSquare::_wait_done(double t) {
  return (t - _mark) > _wait_duration;
}
bool MdlDrawSquare::_preplegs_done(double t) {
  return (t - _mark) > _preplegs_duration;
}

// State methods
void MdlDrawSquare::_wait_entry() {
  // Snapshot whatever pose we activated in. PREPLEGS will blend out of this
  // toward the first point of the square, so the handoff has no discontinuity.
  for (int i = 0; i < 4; i++) {
    _legs[i]->getFootPosition(_footpos_start[i]);
    _footpos[i] = _footpos_start[i];
    _footvel[i] = Eigen::Vector3d::Zero();
  }
}
void MdlDrawSquare::_wait_during() {
  _sendTarget();
}
void MdlDrawSquare::_wait_exit() {}

void MdlDrawSquare::_preplegs_entry() {
  _mark = _mgr->readTime();
  // End pose is exactly where DRAWSQUARE will begin — origin foot + first corner —
  // so the velocity and position match across the state boundary.
  for (int i = 0; i < 4; i++)
    _footpos_end[i] = _originFoot(i) + _fpos[0];
}
void MdlDrawSquare::_preplegs_during() {
  double t = _mgr->readTime();
  double tau = (t - _mark) / _preplegs_duration;
  if (tau < 0.0) tau = 0.0;
  if (tau > 1.0) tau = 1.0;
  // Smoothstep: zero velocity at both ends, peak at the middle.
  double s     = tau * tau * (3.0 - 2.0 * tau);
  double s_dot = 6.0 * tau * (1.0 - tau) / _preplegs_duration;
  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d diff = _footpos_end[i] - _footpos_start[i];
    _footpos[i] = _footpos_start[i] + s * diff;
    _footvel[i] = s_dot * diff;
  }
  _sendTarget();
}
void MdlDrawSquare::_preplegs_exit() {}

void MdlDrawSquare::_drawsquare_entry() {
  _mark = _mgr->readTime();
  for (int i = 0; i < 3; i++) {
    _profiler[i]->clear();
    for (unsigned int j = 0; j < _fpos.size(); j++)
      _profiler[i]->add(j * _sq_period / _fpos.size(), _fpos[j][i]);
    _profiler[i]->setPeriod(_sq_period);
  }
}
void MdlDrawSquare::_drawsquare_during() {
  _resetTarget();
  _computeProfile();
  _sendTarget();
}
void MdlDrawSquare::_drawsquare_exit() {}

bool MdlDrawSquare::_centering_done(double t) {
  return (t - _mark) > _centering_duration;
}

void MdlDrawSquare::_centering_entry() {
  _mark = _mgr->readTime();
  // Capture whatever the reference trajectory is right now. This is what we
  // decelerate out of so the handoff has no velocity kink.
  for (int i = 0; i < 4; i++) {
    _footpos_start_c[i] = _footpos[i];
    _footvel_start_c[i] = _footvel[i];
    _footpos_end_c[i] = _originFoot(i);
  }
}

void MdlDrawSquare::_centering_during() {
  double t = _mgr->readTime();
  double tau = (t - _mark) / _centering_duration;
  if (tau < 0.0) tau = 0.0;
  if (tau > 1.0) tau = 1.0;

  // Cubic Hermite between (start pos, start vel) and (end pos, zero vel).
  // The h11 term drops out since the end velocity is zero.
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  double h00 = 2.0 * tau3 - 3.0 * tau2 + 1.0;
  double h10 = tau3 - 2.0 * tau2 + tau;
  double h01 = -2.0 * tau3 + 3.0 * tau2;
  double dh00 = 6.0 * tau2 - 6.0 * tau;
  double dh10 = 3.0 * tau2 - 4.0 * tau + 1.0;
  double dh01 = -6.0 * tau2 + 6.0 * tau;

  double T = _centering_duration;
  for (int i = 0; i < 4; i++) {
    _footpos[i] = h00 * _footpos_start_c[i]
                + h10 * _footvel_start_c[i] * T
                + h01 * _footpos_end_c[i];
    _footvel[i] = (dh00 * _footpos_start_c[i]
                 + dh10 * _footvel_start_c[i] * T
                 + dh01 * _footpos_end_c[i]) / T;
  }
  _sendTarget();
}

void MdlDrawSquare::_centering_exit() {}

void MdlDrawSquare::stopDrawing() {
  // No-op if we're already stopping or stopped
  if (_state == _state_t::CENTERING || _state == _state_t::DONE) return;
  _state = _state_t::CENTERING;
  _centering_entry();
}

void MdlDrawSquare::_computeProfile() {
  double t = _mgr->readTime();

  for (int i = 0; i < 3; i++) {
    Profiler::fval_t val;
    _profiler[i]->value(t - _mark, val);
    for (int j = 0; j < 4; j++) {
      _footpos[j][i] += val.v;
      _footvel[j][i] += val.d;
    }
  }
}

void MdlDrawSquare::_sendTarget() {
  for (int i = 0; i < 4; i++) {
    _legs[i]->setTargetPosition(_footpos[i], _footvel[i]);
  }
}

Eigen::Vector3d MdlDrawSquare::_originFoot(int leg) const {
  // Nominal resting foot position for this leg, relative to the body.
  // Same geometry as _resetTarget, just per-leg so PREPLEGS can target it directly.
  Eigen::Vector3d p = _kinematics->getKinematicParams().hip_positions.row(leg).transpose();
  p[0] += _origin[0];
  p[1] += _origin[1] * (leg % 2 == 0 ? 1 : -1);
  p[2] += _origin[2];
  return p;
}

void MdlDrawSquare::_resetTarget() {
  for (int i = 0; i < 4; i++) _footpos[i] = _originFoot(i);
}

void MdlDrawSquare::update() {
  double t = _mgr->readTime();

  for (int i = 0; i < 4; i++) _footvel[i] = Eigen::Vector3d::Zero();

  switch (_state) {
    case _state_t::WAIT:
      if (_wait_done(t)) {
        _state = _state_t::PREPLEGS;
        _wait_exit();
        _preplegs_entry();
        break;
      }
      _wait_during();
      break;
    case _state_t::PREPLEGS:
      if (_preplegs_done(t)) {
        _state = _state_t::DRAWSQUARE;
        _preplegs_exit();
        _drawsquare_entry();
        break;
      }
      _preplegs_during();
      break;
    case _state_t::DRAWSQUARE:
      _drawsquare_during();
      break;
    case _state_t::CENTERING:
      if (_centering_done(t)) {
        _state = _state_t::DONE;
        _centering_exit();
        break;
      }
      _centering_during();
      break;
    case _state_t::DONE:
      // Hold the centered pose — last _sendTarget from CENTERING persists in MotorHW
      break;
  }

}
