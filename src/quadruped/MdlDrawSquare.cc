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

extern QuadrupedKinematics::params_t createGo2Config();

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
  return (t - _mark > 3);
}
bool MdlDrawSquare::_preplegs_done(double t) {
  return (t - _mark > _sq_period / 4);
}

// State methods
void MdlDrawSquare::_wait_entry() {
  _resetTarget();
}
void MdlDrawSquare::_wait_during() {
  _sendTarget();
}
void MdlDrawSquare::_wait_exit() {}

void MdlDrawSquare::_preplegs_entry() {
  _mark = _mgr->readTime();
  for (int i = 0; i < 3; i++) {
    _profiler[i]->clear();
    _profiler[i]->add(0, 0);
    _profiler[i]->add(_sq_period / 4, _fpos[0][i]);
  }
}
void MdlDrawSquare::_preplegs_during() {
  _resetTarget();
  _computeProfile();
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

void MdlDrawSquare::_resetTarget() {
  // Reset the foot positions to the initial state
  for (int i = 0; i < 4; i++) {
    _footpos[i] = _kinematics->getKinematicParams().hip_positions.row(i).transpose();
    // Adjust abduction joint so that the feet are slightly outside and below the hip
    _footpos[i][0] += _origin[0];
    _footpos[i][1] += _origin[1] * (i % 2 == 0 ? 1 : -1);
    _footpos[i][2] += _origin[2];
  }
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
    case _state_t::DONE:
      // Do nothing, we are done
      break;
  }

}
