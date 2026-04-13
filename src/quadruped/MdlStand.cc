#include <cmath>
#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlStand.hh"
#include "quadruped/MdlLegControl.hh"
#include "hardware/MotorHW.hh"

#define DBGPRINT(...) //printf(__VA_ARGS__)

MdlStand::MdlStand() : Module(STANDMODULE_NAME, 0, SINGLE_USER) {}
MdlStand::~MdlStand() {}

void MdlStand::init() {
  for (int i = 0; i < 4; i++)
    _legs[i] = (MdlLegControl *)_mgr->findModule(LEGMODULE_NAME, i);

  rtcore::ConfigTable config;
  if (_mgr->getConfigTable("stand", config)) {
    _configDuration = config.getDouble("duration", 2.0);
    _trackingErrorLimit = config.getDouble("tracking_error_limit", 0.5);
  }
}

void MdlStand::uninit() {}

void MdlStand::activate() {
  _status = IDLE;
  _deltaHStart = 0.0;
  _deltaHEnd = 0.0;
  _footCaptured = false;

  // Grab all legs first, then capture FK.
  // If any FK capture fails, release all legs before returning ERROR.
  for (int i = 0; i < 4; i++)
    _mgr->grabModule(_legs[i], this);

  for (int i = 0; i < 4; i++) {
    if (!_legs[i]->getFootPosition(_footB0[i])) {
      _mgr->warning("MdlStand", "Failed to capture foot %d position", i);
      for (int j = 0; j < 4; j++)
        _mgr->releaseModule(_legs[j], this);
      _status = ERROR;
      return;
    }
  }
  _footCaptured = true;

  // Debug: print captured state so we can diagnose IK failures
  for (int i = 0; i < 4; i++) {
    MotorHW::state_t s[3];
    for (int j = 0; j < 3; j++)
      MotorHW::instance()->getState(i * 3 + j, s[j]);
    _mgr->message("MdlStand: leg %d joints=[%.4f, %.4f, %.4f] foot=[%.4f, %.4f, %.4f]",
                   i, s[0].pos, s[1].pos, s[2].pos,
                   _footB0[i](0), _footB0[i](1), _footB0[i](2));
  }
}

void MdlStand::deactivate() {
  for (int i = 0; i < 4; i++)
    _mgr->releaseModule(_legs[i], this);
}

void MdlStand::setTargetHeight(double delta_h) {
  setTargetHeight(delta_h, _configDuration);
}

void MdlStand::setTargetHeight(double delta_h, double duration) {
  if (!_footCaptured) {
    _status = ERROR;
    return;
  }
  // Interpolate from wherever we are now to the new target.
  // If mid-transition, _deltaHStart snaps to current achieved delta.
  double t = _mgr->readTime();
  double tau = (_status == ACTIVE && _duration > 0.0)
      ? (t - _startTime) / _duration : 1.0;
  _deltaHStart = _deltaHStart + (_deltaHEnd - _deltaHStart) * _quintic(tau);
  _deltaHEnd = delta_h;
  _duration = (duration > 0.0) ? duration : _configDuration;
  _startTime = t;
  _status = ACTIVE;
}

double MdlStand::_quintic(double tau) {
  if (tau <= 0.0) return 0.0;
  if (tau >= 1.0) return 1.0;
  double t3 = tau * tau * tau;
  return 10.0 * t3 - 15.0 * t3 * tau + 6.0 * t3 * tau * tau;
}

double MdlStand::_quinticDot(double tau) {
  if (tau <= 0.0 || tau >= 1.0) return 0.0;
  double t2 = tau * tau;
  return 30.0 * t2 - 60.0 * t2 * tau + 30.0 * t2 * tau * tau;
}

bool MdlStand::_checkTrackingError() const {
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

void MdlStand::update() {
  if (_status != ACTIVE) return;

  double t = _mgr->readTime();
  double elapsed = t - _startTime;
  double tau = elapsed / _duration;
  double sigma = _quintic(tau);
  double sigma_dot = _quinticDot(tau) / _duration;

  // Interpolate from _deltaHStart to _deltaHEnd using quintic.
  // Body moves UP by delta_h -> feet go DOWN in body frame.
  double delta_h = _deltaHStart + (_deltaHEnd - _deltaHStart) * sigma;
  double delta_hd = (_deltaHEnd - _deltaHStart) * sigma_dot;
  Eigen::Vector3d delta(0.0, 0.0, delta_h);
  Eigen::Vector3d delta_dot(0.0, 0.0, delta_hd);

  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d target = _footB0[i] - delta;
    Eigen::Vector3d pdot = -delta_dot;

    if (!_legs[i]->setTargetPosition(target, pdot)) {
      _status = ERROR;
      _mgr->warning("MdlStand", "Leg %d IK fail t=%.3f tau=%.4f target=[%.4f,%.4f,%.4f]",
                     i, t, tau, target(0), target(1), target(2));
      return;
    }
  }

  if (!_checkTrackingError()) {
    _status = ERROR;
    _mgr->warning("MdlStand", "Tracking error exceeded at t=%.3f", t);
    return;
  }

  if (tau >= 1.0)
    _status = SETTLED;
}
