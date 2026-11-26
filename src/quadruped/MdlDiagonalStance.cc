/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#include <cmath>
#include "rtcore/ModuleManager.hh"
#include "rtcore/ConfigTable.hh"
#include "quadruped/MdlDiagonalStance.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/TrajectoryUtils.hh"
#include "hardware/MotorHW.hh"
#include "hardware/IMUHW.hh"

namespace {
// Go2 foot sphere radius: a foot centre at -(base_z - radius) in the level frame
// puts the base base_z above the floor, which is the height MdlAMB's nominal
// posture (plan: foot centres at z = -0.278 for base_z = 0.30) assumes.
const double FOOT_RADIUS = 0.022;
const char *STAGE_NAMES[] = {"SHIFT1", "STEP_RR", "SHIFT2", "STEP_FL", "CENTRE"};
// CoM margin [m] inside the support triangle on the way back; the fixed shifts
// of the way in keep 5.4-5.5 cm.
const double RETURN_MARGIN = 0.05;
double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
  return a.x() * b.y() - a.y() * b.x();
}
}  // namespace

MdlDiagonalStance::MdlDiagonalStance() : Module(DIAGSTANCEMODULE_NAME, 0, SINGLE_USER) {}
MdlDiagonalStance::~MdlDiagonalStance() {}

void MdlDiagonalStance::init() {
  for (int i = 0; i < 4; i++)
    _legs[i] = (MdlLegControl *)_mgr->findModule(LEGMODULE_NAME, i);

  rtcore::ConfigTable config;
  if (_mgr->getConfigTable("diagstance", config)) {
    rtcore::ConfigArray arr;
    if (config.getArray("shift1", arr) && arr.size() >= 2)
      _shift1 = Eigen::Vector2d(arr.getDoubleAt(0, _shift1(0)), arr.getDoubleAt(1, _shift1(1)));
    if (config.getArray("shift2", arr) && arr.size() >= 2)
      _shift2 = Eigen::Vector2d(arr.getDoubleAt(0, _shift2(0)), arr.getDoubleAt(1, _shift2(1)));
    _shiftDuration = config.getDouble("shift_duration", _shiftDuration);
    _stepDuration = config.getDouble("step_duration", _stepDuration);
    _stepClearance = config.getDouble("step_clearance", _stepClearance);
    _trackingErrorLimit = config.getDouble("tracking_error_limit", _trackingErrorLimit);
    _kp = config.getDouble("kp", _kp);
    _kd = config.getDouble("kd", _kd);
  } else {
    _mgr->warning(DIAGSTANCEMODULE_NAME, "No [diagstance] config table; using built-in defaults");
  }
  // The final posture is MdlAMB's nominal one, so its geometry comes from [amb]
  // rather than being duplicated under [diagstance].
  if (_mgr->getConfigTable("amb", config)) {
    _stanceX = config.getDouble("stance_x", _stanceX);
    _baseZ = config.getDouble("base_z", _baseZ);
  }
}

void MdlDiagonalStance::uninit() {}

void MdlDiagonalStance::activate() {
  _status = IDLE;

  // Grab all legs first, then capture FK. If any FK capture fails, release all
  // legs before returning ERROR.
  for (int i = 0; i < 4; i++)
    _mgr->grabModule(_legs[i], this);

  for (int i = 0; i < 4; i++) {
    if (!_legs[i]->getFootPosition(_p0[i])) {
      _mgr->warning(DIAGSTANCEMODULE_NAME, "Failed to capture foot %d position", i);
      for (int j = 0; j < 4; j++)
        _mgr->releaseModule(_legs[j], this);
      _status = ERROR;
      return;
    }
  }

  // One-time tilt snapshot (MdlStand pattern), not a feedback loop: feeding the
  // live tilt back into the targets would leave the body free to rotate.
  _tilt = Eigen::AngleAxisd::Identity();
  IMUHW::imudata_t imu;
  if (IMUHW::instance() && IMUHW::instance()->getLastReading(0, imu)) {
    Eigen::Quaterniond q_BW(imu.q.v[0], imu.q.v[1], imu.q.v[2], imu.q.v[3]);
    double norm = q_BW.norm();
    if (std::isfinite(norm) && norm > 0.5) {
      q_BW.normalize();
      Eigen::Vector3d up = (q_BW.conjugate() * Eigen::Vector3d::UnitZ()).normalized();
      _tilt = Eigen::AngleAxisd(Eigen::Quaterniond::FromTwoVectors(up, Eigen::Vector3d::UnitZ()));
    }
  }
  for (int i = 0; i < 4; i++) {
    _p0[i] = _tilt * _p0[i];
    _pEnd[i] = _p0[i];  // _beginStage starts every stage from the previous end
    // Recorded once: MdlStand lets the body settle about 2.5 cm back after a
    // return, so re-recording would walk the footprint forward every cycle.
    if (!_reverse && !_haveStand) _stand[i] = _p0[i];
    _placed[i] = false;
  }
  _shift.setZero();
  _mgr->message("MdlDiagonalStance: activation tilt=%.1f deg%s", _tilt.angle() * 180.0 / M_PI,
                _reverse ? ", back to the stand" : "");
  if (_reverse && !_haveStand) {
    _mgr->warning(DIAGSTANCEMODULE_NAME, "No stand footprint recorded to return to");
    for (int j = 0; j < 4; j++)
      _mgr->releaseModule(_legs[j], this);
    _status = ERROR;
    return;
  }
  _haveStand = true;

  _beginStage(SHIFT1, _mgr->readTime());
  _status = ACTIVE;
}

void MdlDiagonalStance::deactivate() {
  for (int i = 0; i < 4; i++)
    _mgr->releaseModule(_legs[i], this);
}

bool MdlDiagonalStance::_checkTrackingError() const {
  MotorHW *hw = MotorHW::instance();
  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      int idx = leg * 3 + j;
      MotorHW::state_t state;
      MotorHW::cmd_t cmd;
      hw->getState(idx, state);
      hw->getCommand(idx, cmd);
      if (std::abs(state.pos - cmd.pos) > _trackingErrorLimit)
        return false;
    }
  }
  return true;
}

// Stage table of the plan, in the level frame: each stage interpolates every
// foot from where the previous stage left it (the commanded end, not the
// measured FK, so the command stays continuous across the boundary) to its new
// end. The stepping foot additionally carries the clearance hump in update().
void MdlDiagonalStance::_beginStage(int stage, double t) {
  const Eigen::Vector3d d1(_shift1(0), _shift1(1), 0.0);
  const Eigen::Vector3d d2(_shift2(0), _shift2(1), 0.0);

  _stage = stage;
  _stageStart = t;
  _stepLeg = -1;
  _last = false;
  for (int i = 0; i < 4; i++)
    _pStart[i] = _pEnd[i];
  // The first stage also levels the body and brings it to the hand-off height
  // before any foot lifts, since a tilted body moves the CoM by
  // base_z * sin(tilt) and eats the margin.
  if (stage == 0)
    for (int i = 0; i < 4; i++) _p0[i](2) = -(_baseZ - FOOT_RADIUS);

  if (_reverse) {
    _planReturn(stage);
    _duration = (_stepLeg >= 0) ? _stepDuration : _shiftDuration;
    _mgr->message("MdlDiagonalStance: t=%.3f %s, leg %d next, body shift [%.3f %.3f]", t,
                  _last ? "CENTRE" : _stepLeg >= 0 ? "STEP" : "SHIFT", _nextLeg, _shift(0),
                  _shift(1));
    return;
  }
  _duration = (stage == STEP_FL || stage == STEP_RR) ? _stepDuration : _shiftDuration;

  switch (stage) {
  case SHIFT1:  // body toward FL, inside the FL/FR/RL triangle
    for (int i = 0; i < 4; i++) _pEnd[i] = _p0[i] - d1;
    break;
  case STEP_RR:  // nominal spot expressed in the shifted frame
    _stepLeg = 3;
    _p0[3] = Eigen::Vector3d(-_stanceX, 0.0, _p0[3](2));
    _pEnd[3] = _p0[3] - d1;
    break;
  case SHIFT2:  // body toward RR, inside the FR/RL/RR triangle
    for (int i = 0; i < 4; i++) _pEnd[i] = _p0[i] - d2;
    break;
  case STEP_FL:
    _stepLeg = 0;
    _p0[0] = Eigen::Vector3d(_stanceX, 0.0, _p0[0](2));
    _pEnd[0] = _p0[0] - d2;
    break;
  case CENTRE:  // shift back to zero: FL/RR land on (+-stance_x, 0, -0.278)
    for (int i = 0; i < 4; i++) _pEnd[i] = _p0[i];
    _last = true;
    break;
  }

  _mgr->message("MdlDiagonalStance: t=%.3f %s FL->[%.3f %.3f %.3f] RR->[%.3f %.3f %.3f]",
                t, STAGE_NAMES[stage], _pEnd[0](0), _pEnd[0](1), _pEnd[0](2),
                _pEnd[3](0), _pEnd[3](1), _pEnd[3](2));
}

// The way back alternates a body shift (even stage) and a step (odd stage)
// until every foot is on its stand spot, then centres. A shift takes the body
// to the nearest point RETURN_MARGIN inside the triangle the other three feet
// span, the body origin standing in for the CoM as in the fixed shifts of the
// way in. The foot that needs the shortest shift steps next: a long one runs a
// swing foot that landed far out past its leg's reach, and it settles the
// order too (with FR or RL landed near the centreline, lifting FL first would
// leave a sliver of a triangle).
void MdlDiagonalStance::_planReturn(int stage) {
  if (stage % 2 == 1) {
    _stepLeg = _nextLeg;
    _p0[_stepLeg].head<2>() = _stand[_stepLeg].head<2>();
    _pEnd[_stepLeg] = _p0[_stepLeg] - _shift;
    _placed[_stepLeg] = true;
    return;
  }
  const Eigen::Vector2d here = _shift.head<2>();
  Eigen::Vector2d target = Eigen::Vector2d::Zero();  // all four placed: centre the body
  double best = INFINITY;
  _nextLeg = -1;
  for (int leg = 0; leg < 4; leg++) {
    if (_placed[leg]) continue;
    Eigen::Vector2d v[3];
    for (int i = 0, n = 0; i < 4; i++)
      if (i != leg) v[n++] = _p0[i].head<2>();
    const double a = (v[1] - v[2]).norm(), b = (v[0] - v[2]).norm(), c = (v[0] - v[1]).norm();
    const double area2 = cross2(v[1] - v[0], v[2] - v[0]);
    const double inradius = std::abs(area2) / (a + b + c);
    const Eigen::Vector2d incentre = (a * v[0] + b * v[1] + c * v[2]) / (a + b + c);
    // Shrinking the triangle about its incentre moves every edge in by the
    // margin; one too thin for that is left with its incentre alone, and
    // ranks behind every triangle that is not.
    Eigen::Vector2d q = incentre;
    double cost = 1.0 + RETURN_MARGIN - inradius;
    if (inradius > RETURN_MARGIN) {
      const double k = 1.0 - RETURN_MARGIN / inradius;
      bool inside = true;
      cost = INFINITY;
      for (int i = 0; i < 3; i++) {
        const Eigen::Vector2d w = incentre + k * (v[i] - incentre);
        const Eigen::Vector2d e = k * (v[(i + 1) % 3] - v[i]);
        inside = inside && cross2(e, here - w) * area2 >= 0.0;
        const double along = (here - w).dot(e) / e.squaredNorm();
        const Eigen::Vector2d on = w + std::min(1.0, std::max(0.0, along)) * e;
        if ((on - here).norm() < cost) {
          cost = (on - here).norm();
          q = on;
        }
      }
      if (inside) {
        cost = 0.0;
        q = here;
      }
    }
    if (cost < best) {
      best = cost;
      _nextLeg = leg;
      target = q;
    }
  }
  _last = (_nextLeg < 0);
  _shift.head<2>() = target;
  for (int i = 0; i < 4; i++) _pEnd[i] = _p0[i] - _shift;
}

void MdlDiagonalStance::update() {
  if (_status != ACTIVE && _status != SETTLED) return;

  double t = _mgr->readTime();
  double tau = (t - _stageStart) / _duration;
  double sigma, sigma_dot;
  TrajectoryUtils::sampleQuintic(tau, _duration, sigma, sigma_dot);

  // Level-frame targets go to the legs through the remaining tilt, which the
  // first stage blends from the snapshot to zero (all later stages already see 0).
  double lev = (_stage == 0) ? sigma : 1.0;
  double lev_dot = (_stage == 0) ? sigma_dot : 0.0;
  Eigen::Matrix3d R_LB = Eigen::AngleAxisd((1.0 - lev) * _tilt.angle(), _tilt.axis())
                             .toRotationMatrix().transpose();

  // After SETTLED tau keeps growing, sigma stays 1 and the CENTRE end targets
  // are re-sent every tick, so the hand-off tick to MdlAMB sees a held posture.
  const Eigen::Vector3d kp = Eigen::Vector3d::Constant(_kp);
  const Eigen::Vector3d kd = Eigen::Vector3d::Constant(_kd);
  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d p = _pStart[i] + (_pEnd[i] - _pStart[i]) * sigma;
    Eigen::Vector3d pdot = (_pEnd[i] - _pStart[i]) * sigma_dot;
    if (i == _stepLeg) {
      p(2) += _stepClearance * std::sin(M_PI * sigma);
      pdot(2) += _stepClearance * M_PI * std::cos(M_PI * sigma) * sigma_dot;
    }
    Eigen::Vector3d target = R_LB * p;
    pdot = R_LB * (pdot + lev_dot * _tilt.angle() * _tilt.axis().cross(p));
    if (!_legs[i]->setFootCommand(target, pdot, kp, kd, Eigen::Vector3d::Zero())) {
      _status = ERROR;
      _mgr->warning(DIAGSTANCEMODULE_NAME, "Leg %d IK fail t=%.3f %s tau=%.4f target=[%.4f,%.4f,%.4f]",
                    i, t, _reverse ? "RETURN" : STAGE_NAMES[_stage], tau, target(0), target(1),
                    target(2));
      return;
    }
  }

  if (!_checkTrackingError()) {
    _status = ERROR;
    _mgr->warning(DIAGSTANCEMODULE_NAME, "Tracking error exceeded at t=%.3f", t);
    return;
  }

  if (_status != ACTIVE || tau < 1.0) return;

  if (!_last) {
    _beginStage(_stage + 1, t);
    return;
  }

  _status = SETTLED;
  Eigen::Vector3d p[4];
  for (int i = 0; i < 4; i++)
    if (!_legs[i]->getFootPosition(p[i])) p[i].setConstant(NAN);
  _mgr->message("MdlDiagonalStance: t=%.3f SETTLED, FK feet FL[%.4f %.4f %.4f] FR[%.4f %.4f %.4f] "
                "RL[%.4f %.4f %.4f] RR[%.4f %.4f %.4f]",
                t, p[0](0), p[0](1), p[0](2), p[1](0), p[1](1), p[1](2),
                p[2](0), p[2](1), p[2](2), p[3](0), p[3](1), p[3](2));
}
