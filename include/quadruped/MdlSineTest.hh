#ifndef MDLSINETEST_HH
#define MDLSINETEST_HH

#include "rtcore/Module.hh"
#include "hardware/MotorHW.hh"

#define SINETESTMODULE_NAME "MdlSineTest"

class MdlSineTest : public rtcore::Module {
public:
  MdlSineTest();
  ~MdlSineTest();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

private:
  MotorHW *_motorhw = nullptr;

  // FR leg motor indices (hip=0, thigh=1, calf=2)
  static constexpr int _idx[3] = {0, 1, 2};

  // Initial joint positions captured at activation
  double _qInit[3] = {0};

  // Commands for each joint
  MotorHW::cmd_t _cmd[3];

  // Sinusoid parameters
  double _amplitude = 0.4;  // rad
  double _frequency = 0.2;  // Hz
  double _kp = 5.0;
  double _kd = 1.0;

  double _startTime = 0.0;
  int _warmup = 0;
  bool _initialized = false;
};

#endif
