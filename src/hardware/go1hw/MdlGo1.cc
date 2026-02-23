#include "MdlGo1.hh"
#include "rtcore/ModuleManager.hh"
#include <cmath>
#include <boost/bind.hpp>

using namespace rtcore;
using namespace UNITREE_LEGGED_SDK;

// Mapping for motor ids to GO1 motors
static constexpr int IdxToJoint[12] = {                                                                                                    
  FR_0, FR_1, FR_2,                                                           
  FL_0, FL_1, FL_2,                   
  RR_0, RR_1, RR_2,
  RL_0, RL_1, RL_2
};

#define DBGPRINT(...) printf(__VA_ARGS__)

MdlGo1::MdlGo1() : Module(GO1MODULE_NAME, 0, SINGLE_USER), _safe(LeggedType::Go1), _udp(LOWLEVEL, 8090, "192.168.123.10", 8007) {
  DBGPRINT("MdlGo1::MdlGo1\n");
  pthread_mutex_init( &_data_lock, NULL );
}

MdlGo1::~MdlGo1() {
  DBGPRINT("MdlGo1::~MdlGo1\n");
  pthread_mutex_destroy( &_data_lock );
}

bool MdlGo1::setEnable(unsigned int index, bool enable) {
  if (index >= _m.size()) return false;
  bool prev = _m[index].enable;
  _m[index].enable = enable;

  // If enable state changed, wake up the communication thread
  if (prev != enable) {
    pthread_mutex_lock( &_data_lock);
    _updateStates = true;
    pthread_mutex_unlock( &_data_lock);
    
    sendSync();
  }
  return prev;
}

bool MdlGo1::isEnabled( unsigned int index ) {
  if (index >= _m.size()) return false;
  return _m[index].enable;
}

void MdlGo1::init() {
  DBGPRINT("MdlGo1::init\n");
  _udp.InitCmdData(_cmd);
  
  ConfigTable dc;
  bool hasConfig = _mgr->getConfigTable("go1", dc);
  if (hasConfig) {
    ConfigArray ids;
    bool hasIds = dc.getArray( "motor_ids", ids );
    if (hasIds) {
      for (int i = 0; i < ids.size(); i++) {
        if (ids.hasTypeAt( i, ConfigType::Int)) {
          _motor_t m;
          m.enable = false;
          m.id = ids.getIntAt( i , 0);
          if (m.id >= 0) 
            _m.push_back( m );
        } else
          _mgr->fatalError("MdlGo1",
                           "All members of motor_ids config should be integers");
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
      _mgr->fatalError("MdlGo1","calibration.offset must have %d entries!",
                       _m.size());
    for (int i = 0; i < _m.size(); i++) {
      if (!off.hasTypeAt( i, ConfigType::Double))
        _mgr->fatalError("MdlGo1","calibration.offset must consist of numbers!");
      _m[i].offset = off.getDoubleAt(i);
    }
      
    calibConfig.getArray("polarity", pol);
    if (pol.size() != _m.size())
      _mgr->fatalError("MdlGo1","calibration.polarity must have %d entries!",
                       _m.size());
    
    for (int i = 0; i < _m.size(); i++) {
      _m[i].polarity = pol.getIntAt(i);
      if (_m[i].polarity != 1 && _m[i].polarity != -1)
        _mgr->fatalError("MdlGo1",
                         "calibration.polarity entries must be 1 or -1!");
    }
  }

  this->start( "dmcomm", dc.getInt("thread_priority", 98 ));
}

MotorHW::status_t MdlGo1::getJointStatus(unsigned int index) {
  if (index >= _m.size()) return MotorHW::STATUS_ERROR;
  return _m[index].status;
}

void MdlGo1::getJointState(unsigned int index, MotorHW::state_t &state) {
  if (index >= _m.size()) return;
  state = _m[index].state;
}

void MdlGo1::setJointCommand(unsigned int index, MotorHW::cmd_t &cmd) {
  if (index >= _m.size()) return;
  _m[index].cmd = cmd;
}

void MdlGo1::getJointCommand(unsigned int index, MotorHW::cmd_t &cmd) {
  if (index >= _m.size()) return;
  cmd = _m[index].cmd;
}

void MdlGo1::uninit() {
  DBGPRINT("MdlGo1::uninit\n");
  this->terminate();
}
void MdlGo1::activate() {}
void MdlGo1::deactivate() {}

void MdlGo1::update() {

  // periodically retrieve states and send commands.
  pthread_mutex_lock( &_data_lock);
  _updateStates = true;
  _updateCommands = true;
  pthread_mutex_unlock( &_data_lock);

  sendSync();
}

void MdlGo1::threadEnter(void) {
  DBGPRINT("MdlGo1::threadEnter\n");
  _loopRecv = new LoopFunc("udp_recv", 0.002, 3, boost::bind(&UDP::Recv, &_udp));
  _loopSend = new LoopFunc("udp_send", 0.002, 3, boost::bind(&UDP::Send, &_udp));

  _loopRecv->start();
  _loopSend->start();
}

void MdlGo1::threadLoop(void) {
  waitSync();

  // Check whether to update
  pthread_mutex_lock(&_data_lock);
  _udp.GetRecv(_state);
  if (_updateStates) {
    _updateStates = false;

    for (unsigned int i = 0; i < _m.size(); i++) {
      int id = _m[i].id;
      _m[i].status = MotorHW::STATUS_READY;
      _m[i].state.t = _mgr->readTime();
      _m[i].state.pos = _m[i].polarity * double(_state.motorState[IdxToJoint[id]].q) + _m[i].offset;
      _m[i].state.vel = _m[i].polarity * double(_state.motorState[IdxToJoint[id]].dq);
      _m[i].state.tau = _m[i].polarity * double(_state.motorState[IdxToJoint[id]].tauEst);
      _m[i].state.temp = double(_state.motorState[IdxToJoint[id]].temperature);
    }
    pthread_mutex_unlock(&_data_lock);
  } else {
    pthread_mutex_unlock(&_data_lock);
  }

  // Check whether to sent
  pthread_mutex_lock(&_data_lock);
  if (_updateCommands) {
    _updateCommands = false;

    for (unsigned int i = 0; i < _m.size(); i++) {
      int id = _m[i].id;
      _cmd.motorCmd[IdxToJoint[id]].q = (_m[i].cmd.pos* _m[i].polarity) - _m[i].offset;
      _cmd.motorCmd[IdxToJoint[id]].dq = _m[i].cmd.vel * _m[i].polarity;
      _cmd.motorCmd[IdxToJoint[id]].tau = _m[i].cmd.tau * _m[i].polarity;
      _cmd.motorCmd[IdxToJoint[id]].Kp = _m[i].cmd.kp;
      _cmd.motorCmd[IdxToJoint[id]].Kd = _m[i].cmd.kd;
    }
    int res = _safe.PowerProtect(_cmd, _state, 1);
    if (res < 0) _mgr->fatalError("MdlGo1", "Power Protect Triggered!");
    _udp.SetSend(_cmd);
    pthread_mutex_unlock(&_data_lock);
  } else {
    pthread_mutex_unlock(&_data_lock);
  }
}

void MdlGo1::threadExit( void ) {
  DBGPRINT("MdlGo1::threadExit\n");
  _loopRecv->shutdown();
  _loopSend->shutdown();
  delete _loopRecv;
  delete _loopSend;
}