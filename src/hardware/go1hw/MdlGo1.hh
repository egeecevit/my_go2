#ifndef GO1MODULE_HH
#define GO1MODULE_HH
#include <rtcore/Module.hh>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#define GO1MODULE_NAME "MdlGo1"

class MdlLegControl;

class MdlGo1 : public rtcore::Module {
public:
  MdlGo1();
  ~MdlGo1();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

private:
  UNITREE_LEGGED_SDK::Safety _safe;
  UNITREE_LEGGED_SDK::UDP _udp;
  UNITREE_LEGGED_SDK::LowCmd _cmd = {0};
  UNITREE_LEGGED_SDK::LowState _state = {0};
  double _motiontime = 0.0;

  MdlLegControl *_leg;

};

#endif