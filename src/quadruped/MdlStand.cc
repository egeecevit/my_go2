#include <cmath>
#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlStand.hh"
#include "quadruped/MdlLegControl.hh"
#include "hardware/MotorHW.hh"
#include "hardware/IMUHW.hh"

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

  // Figure out which way is "up" so we stand along gravity, not body-z.
  // One-time snapshot — not a feedback loop.
  _worldUpInBody = Eigen::Vector3d::UnitZ();
  IMUHW::imudata_t imu;
  if (IMUHW::instance() && IMUHW::instance()->getLastReading(0, imu)) {
    Eigen::Quaterniond q_BW(imu.q.v[0], imu.q.v[1], imu.q.v[2], imu.q.v[3]);
    double norm = q_BW.norm();
    if (std::isfinite(norm) && norm > 0.5) {
      q_BW.normalize();
      _worldUpInBody = (q_BW.conjugate() * Eigen::Vector3d::UnitZ()).normalized();
    }
  }
  double tilt = std::acos(std::min(1.0, _worldUpInBody.dot(Eigen::Vector3d::UnitZ())));
  _mgr->message("MdlStand: activation tilt=%.1f deg", tilt * 180.0 / M_PI);
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

  // Move along gravity (not body-z) so the robot levels out
  double delta_h = _deltaHStart + (_deltaHEnd - _deltaHStart) * sigma;
  double delta_hd = (_deltaHEnd - _deltaHStart) * sigma_dot;
  Eigen::Vector3d delta = _worldUpInBody * delta_h;
  Eigen::Vector3d delta_dot = _worldUpInBody * delta_hd;

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
