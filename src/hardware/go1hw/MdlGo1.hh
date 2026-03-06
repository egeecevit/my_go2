#ifndef _MDLGO1_HH
#define _MDLGO1_HH
#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "rtcore/ThreadedLoop.hh"
#include <memory>
#include <mutex>
#include <rtcore/Module.hh>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#define GO1MODULE_NAME "MdlGo1"

class MdlGo1 : public rtcore::Module {
public:
  MdlGo1();
  ~MdlGo1();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  bool setEnable(unsigned int index, bool enable);
  bool isEnabled(unsigned int index);

  void getJointState(unsigned int index, MotorHW::state_t &state);

  void setJointCommand(unsigned int index, MotorHW::cmd_t &cmd);
  void getJointCommand(unsigned int index, MotorHW::cmd_t &cmd);

  MotorHW::status_t getJointStatus(unsigned int index);

  bool getIMUData(IMUHW::imudata_t &data);

private:
  // Unitree SDK related
  UNITREE_LEGGED_SDK::Safety _safe;
  std::unique_ptr<UNITREE_LEGGED_SDK::UDP> _udp;
  UNITREE_LEGGED_SDK::LowCmd _cmd = {0};
  UNITREE_LEGGED_SDK::LowState _state = {0};
  std::unique_ptr<UNITREE_LEGGED_SDK::LoopFunc> _loopSend;
  std::unique_ptr<UNITREE_LEGGED_SDK::LoopFunc> _loopRecv;

  typedef struct {
    unsigned int id;
    bool enable = false;
    MotorHW::state_t state;
    MotorHW::cmd_t cmd;
    // Axis polarity to multiple position, velocity and torque
    int polarity = 1;
    // Angular offset to be added to readings, and subtracted from commands
    double offset = 0;
    MotorHW::status_t status = MotorHW::STATUS_STARTUP;
  } _motor_t;

  // Motor info
  std::vector<_motor_t> _m;

  IMUHW::imudata_t _imuData;

  // Mutex to coordinate data access between the UDP thread and
  // joint data access methods
  std::mutex _data_mutex;

  // Triggers for the GO1 comms thread
  bool _updateStates = false;
  bool _updateCommands = false;
};

#endif
