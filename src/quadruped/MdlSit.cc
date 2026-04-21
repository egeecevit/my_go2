#include <cmath>
#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlSit.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/TrajectoryUtils.hh"
#include "hardware/MotorHW.hh"

MdlSit::MdlSit() : Module(SITMODULE_NAME, 0, SINGLE_USER) {}
MdlSit::~MdlSit() {}

void MdlSit::init() {
  for (int i = 0; i < 4; i++)
    _legs[i] = (MdlLegControl *)_mgr->findModule(LEGMODULE_NAME, i);

  // Pick kinematics config by hardware target
  rtcore::ConfigTable hwConfig;
  std::string hwlib;
  if (_mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");
  if (hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

  // Load sit config: timing + per-leg joint targets
  rtcore::ConfigTable config;
  if (_mgr->getConfigTable("sit", config)) {
    _duration = config.getDouble("duration", 2.0);
    _trackingErrorLimit = config.getDouble("tracking_error_limit", 0.5);

    const char* keys[] = {"fl_joints", "fr_joints", "rl_joints", "rr_joints"};
    _sitTargetValid = true;
    for (int i = 0; i < 4; i++) {
      rtcore::ConfigArray arr;
      if (!config.getArray(keys[i], arr) || arr.size() < 3) {
        _mgr->warning("MdlSit", "Missing or incomplete %s in sit config", keys[i]);
        _sitTargetValid = false;
        break;
      }
      Eigen::Vector3d angles(arr.getDoubleAt(0, 0), arr.getDoubleAt(1, 0), arr.getDoubleAt(2, 0));
      Eigen::Vector3d footpos;
      if (!_kinematics->forwardKinematicsUnchecked(i, angles, footpos)) {
        _mgr->warning("MdlSit", "FK failed on %s angles [%.3f, %.3f, %.3f]",
                       keys[i], angles(0), angles(1), angles(2));
        _sitTargetValid = false;
        break;
      }
      _sitTarget[i] = footpos;
    }
  } else {
    _mgr->warning("MdlSit", "No [sit] config found");
    _sitTargetValid = false;
  }
}

void MdlSit::uninit() {
  if (_kinematics) delete _kinematics;
  _kinematics = nullptr;
}

void MdlSit::activate() {
  _status = IDLE;

  if (!_sitTargetValid) {
    _mgr->warning("MdlSit", "No valid sit target — can't activate");
    _status = ERROR;
    return;
  }

  for (int i = 0; i < 4; i++)
    _mgr->grabModule(_legs[i], this);

  for (int i = 0; i < 4; i++) {
    if (!_legs[i]->getFootPosition(_footStart[i])) {
      _mgr->warning("MdlSit", "Failed to capture foot %d position", i);
      for (int j = 0; j < 4; j++)
        _mgr->releaseModule(_legs[j], this);
      _status = ERROR;
      return;
    }
  }

  _startTime = _mgr->readTime();
  _status = ACTIVE;
}

void MdlSit::deactivate() {
  for (int i = 0; i < 4; i++)
    _mgr->releaseModule(_legs[i], this);
}

bool MdlSit::_checkTrackingError() const {
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

void MdlSit::update() {
  if (_status != ACTIVE) return;

  double t = _mgr->readTime();
  double elapsed = t - _startTime;
  double tau = elapsed / _duration;
  double sigma, sigma_dot;
  TrajectoryUtils::sampleQuintic(tau, _duration, sigma, sigma_dot);

  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d diff = _sitTarget[i] - _footStart[i];
    Eigen::Vector3d target = _footStart[i] + sigma * diff;
    Eigen::Vector3d pdot = sigma_dot * diff;

    if (!_legs[i]->setTargetPosition(target, pdot)) {
      _status = ERROR;
      _mgr->warning("MdlSit", "Leg %d command failed at t=%.3f", i, t);
      return;
    }
  }

  if (!_checkTrackingError()) {
    _status = ERROR;
    _mgr->warning("MdlSit", "Tracking error exceeded at t=%.3f", t);
    return;
  }

  if (tau >= 1.0)
    _status = SETTLED;
}
