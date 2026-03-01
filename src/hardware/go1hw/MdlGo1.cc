#include "MdlGo1.hh"
#include "rtcore/ModuleManager.hh"
#include <boost/bind.hpp>

using namespace rtcore;
using namespace UNITREE_LEGGED_SDK;

// Mapping for motor ids to GO1 motors
static constexpr int IdxToJoint[12] = {FR_0, FR_1, FR_2, FL_0, FL_1, FL_2,
                                       RR_0, RR_1, RR_2, RL_0, RL_1, RL_2};

MdlGo1::MdlGo1()
    : Module(GO1MODULE_NAME, 0, SINGLE_USER), _safe(LeggedType::Go1),
      _udp(LOWLEVEL, 8090, "192.168.123.10", 8007) {
}

MdlGo1::~MdlGo1() {
}

bool MdlGo1::setEnable(unsigned int index, bool enable) {
  if (index >= _m.size())
    return false;

  bool changed = false;
  bool prev;
  {
    std::lock_guard<std::mutex> lock(_data_mutex);
    prev = _m[index].enable;
    _m[index].enable = enable;
    if (prev != enable) {
      _updateStates = true;
      changed = true;
    }
  }
  if (changed) sendSync();
  return prev;
}

bool MdlGo1::isEnabled(unsigned int index) {
  if (index >= _m.size())
    return false;
  std::lock_guard<std::mutex> lock(_data_mutex);
  return _m[index].enable;
}

void MdlGo1::init() {
  _mgr->message("MdlGo1: Initializing...");
  _udp.InitCmdData(_cmd);

  ConfigTable dc;
  bool hasConfig = _mgr->getConfigTable("go1", dc);
  if (hasConfig) {
    ConfigArray ids;
    bool hasIds = dc.getArray("motor_ids", ids);
    if (hasIds) {
      for (int i = 0; i < ids.size(); i++) {
        if (ids.hasTypeAt(i, ConfigType::Int)) {
          _motor_t m;
          m.enable = false;
          m.id = ids.getIntAt(i, 0);
          if (m.id >= 0)
            _m.push_back(m);
        } else
          _mgr->fatalError(
              "MdlGo1", "All members of motor_ids config should be integers");
      }
    }
  }

  if (_m.size() == 0)
    _mgr->warning("MdlGo1", "No motor IDs are configured!");
  else {
    _mgr->message("MdlGo1: %d motors configured.", _m.size());
  }

  // Read fixed offset information from the configuration
  ConfigTable calibConfig;
  bool hasCalibConfig = _mgr->getConfigTable("calibration", calibConfig);
  if (hasCalibConfig) {
    ConfigArray off, pol;
    calibConfig.getArray("offsets", off);
    if (off.size() != _m.size())
      _mgr->fatalError("MdlGo1", "calibration.offset must have %d entries!",
                       _m.size());
    for (int i = 0; i < _m.size(); i++) {
      if (!off.hasTypeAt(i, ConfigType::Double))
        _mgr->fatalError("MdlGo1",
                         "calibration.offset must consist of numbers!");
      _m[i].offset = off.getDoubleAt(i);
    }

    calibConfig.getArray("polarity", pol);
    if (pol.size() != _m.size())
      _mgr->fatalError("MdlGo1", "calibration.polarity must have %d entries!",
                       _m.size());

    for (int i = 0; i < _m.size(); i++) {
      _m[i].polarity = pol.getIntAt(i);
      if (_m[i].polarity != 1 && _m[i].polarity != -1)
        _mgr->fatalError("MdlGo1",
                         "calibration.polarity entries must be 1 or -1!");
    }
  }

  this->start("dmcomm", dc.getInt("thread_priority", 98));
}

MotorHW::status_t MdlGo1::getJointStatus(unsigned int index) {
  if (index >= _m.size())
    return MotorHW::STATUS_ERROR;
  std::lock_guard<std::mutex> lock(_data_mutex);
  return _m[index].status;
}

void MdlGo1::getJointState(unsigned int index, MotorHW::state_t &state) {
  if (index >= _m.size())
    return;
  std::lock_guard<std::mutex> lock(_data_mutex);
  state = _m[index].state;
}

void MdlGo1::setJointCommand(unsigned int index, MotorHW::cmd_t &cmd) {
  if (index >= _m.size())
    return;
  std::lock_guard<std::mutex> lock(_data_mutex);
  _m[index].cmd = cmd;
}

void MdlGo1::getJointCommand(unsigned int index, MotorHW::cmd_t &cmd) {
  if (index >= _m.size())
    return;
  std::lock_guard<std::mutex> lock(_data_mutex);
  cmd = _m[index].cmd;
}

bool MdlGo1::getIMUData(IMUHW::imudata_t &data) {
  std::lock_guard<std::mutex> lock(_data_mutex);
  if (_imuData.t < 0) return false;
  data = _imuData;
  return true;
}

void MdlGo1::uninit() {
  _mgr->message("MdlGo1: Shutting down...");
  this->terminate();
}
void MdlGo1::activate() {}
void MdlGo1::deactivate() {}

void MdlGo1::update() {
  {
    std::lock_guard<std::mutex> lock(_data_mutex);
    _updateStates = true;
    _updateCommands = true;
  }
  sendSync();
}

void MdlGo1::threadEnter(void) {
  _mgr->message("MdlGo1: Communication thread started.");
  _loopRecv = std::make_unique<LoopFunc>(
      "udp_recv", 0.002, 3, boost::bind(&UDP::Recv, &_udp));
  _loopSend = std::make_unique<LoopFunc>(
      "udp_send", 0.002, 3, boost::bind(&UDP::Send, &_udp));

  _loopRecv->start();
  _loopSend->start();
}

void MdlGo1::threadLoop(void) {
  waitSync();

  // Section 1: Read states from robot
  {
    std::lock_guard<std::mutex> lock(_data_mutex);
    _udp.GetRecv(_state);
    if (_updateStates) {
      _updateStates = false;
      for (unsigned int i = 0; i < _m.size(); i++) {
        int id = _m[i].id;
        _m[i].status = MotorHW::STATUS_READY;
        _m[i].state.t = _mgr->readTime();
        _m[i].state.pos =
            _m[i].polarity * double(_state.motorState[IdxToJoint[id]].q) +
            _m[i].offset;
        _m[i].state.vel =
            _m[i].polarity * double(_state.motorState[IdxToJoint[id]].dq);
        _m[i].state.tau =
            _m[i].polarity * double(_state.motorState[IdxToJoint[id]].tauEst);
        _m[i].state.temp = double(_state.motorState[IdxToJoint[id]].temperature);
      }

      // Copy IMU data
      _imuData.t = _mgr->readTime();
      _imuData.q.v[0] = double(_state.imu.quaternion[0]);
      _imuData.q.v[1] = double(_state.imu.quaternion[1]);
      _imuData.q.v[2] = double(_state.imu.quaternion[2]);
      _imuData.q.v[3] = double(_state.imu.quaternion[3]);
      _imuData.gyro.v[0] = double(_state.imu.gyroscope[0]);
      _imuData.gyro.v[1] = double(_state.imu.gyroscope[1]);
      _imuData.gyro.v[2] = double(_state.imu.gyroscope[2]);
      _imuData.acc.v[0] = double(_state.imu.accelerometer[0]);
      _imuData.acc.v[1] = double(_state.imu.accelerometer[1]);
      _imuData.acc.v[2] = double(_state.imu.accelerometer[2]);
      _imuData.rpy[0] = double(_state.imu.rpy[0]);
      _imuData.rpy[1] = double(_state.imu.rpy[1]);
      _imuData.rpy[2] = double(_state.imu.rpy[2]);
    }
  }

  // Section 2: Send commands to robot
  {
    std::lock_guard<std::mutex> lock(_data_mutex);
    if (_updateCommands) {
      _updateCommands = false;
      for (unsigned int i = 0; i < _m.size(); i++) {
        int id = _m[i].id;
        if (_m[i].enable) {
          _cmd.motorCmd[IdxToJoint[id]].mode = 0x0A;
          _cmd.motorCmd[IdxToJoint[id]].q =
              (_m[i].cmd.pos * _m[i].polarity) - _m[i].offset;
          _cmd.motorCmd[IdxToJoint[id]].dq = _m[i].cmd.vel * _m[i].polarity;
          _cmd.motorCmd[IdxToJoint[id]].tau = _m[i].cmd.tau * _m[i].polarity;
          _cmd.motorCmd[IdxToJoint[id]].Kp = _m[i].cmd.kp;
          _cmd.motorCmd[IdxToJoint[id]].Kd = _m[i].cmd.kd;
        } else {
          _cmd.motorCmd[IdxToJoint[id]].mode = 0x00;
          _cmd.motorCmd[IdxToJoint[id]].q = 0;
          _cmd.motorCmd[IdxToJoint[id]].dq = 0;
          _cmd.motorCmd[IdxToJoint[id]].tau = 0;
          _cmd.motorCmd[IdxToJoint[id]].Kp = 0;
          _cmd.motorCmd[IdxToJoint[id]].Kd = 0;
        }
      }
      int res = _safe.PowerProtect(_cmd, _state, 1);
      if (res < 0)
        _mgr->fatalError("MdlGo1", "Power Protect Triggered!");
      _udp.SetSend(_cmd);
    }
  }
}

void MdlGo1::threadExit(void) {
  _mgr->message("MdlGo1: Communication thread stopped.");
  _loopRecv->shutdown();
  _loopSend->shutdown();
  _loopRecv.reset();
  _loopSend.reset();
}
