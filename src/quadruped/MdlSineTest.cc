#include <cmath>
#include <stdio.h>

#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlSineTest.hh"

using namespace rtcore;

#define DBGPRINT(...) printf(__VA_ARGS__)

MdlSineTest::MdlSineTest() : Module(SINETESTMODULE_NAME, 0, SINGLE_USER) {
  DBGPRINT("MdlSineTest::MdlSineTest\n");
}

MdlSineTest::~MdlSineTest() {
  DBGPRINT("MdlSineTest::~MdlSineTest\n");
}

void MdlSineTest::init() {
  DBGPRINT("MdlSineTest::init\n");
  _motorhw = MotorHW::instance();
}

void MdlSineTest::uninit() {
  DBGPRINT("MdlSineTest::uninit\n");
}

void MdlSineTest::activate() {
  DBGPRINT("MdlSineTest::activate\n");

  for (int i = 0; i < 3; i++) {
    _motorhw->grab(_idx[i]);
    _motorhw->enable(_idx[i]);
  }

  _warmup = 0;
  _initialized = false;
}

void MdlSineTest::deactivate() {
  DBGPRINT("MdlSineTest::deactivate\n");

  for (int i = 0; i < 3; i++) {
    _motorhw->disable(_idx[i]);
    _motorhw->release(_idx[i]);
  }
}

void MdlSineTest::update() {
  if (!_initialized) {
    _warmup++;
    if (_warmup < 500) return;

    MotorHW::state_t state;
    for (int i = 0; i < 3; i++) {
      _motorhw->getState(_idx[i], state);
      _qInit[i] = state.pos;
      _cmd[i].pos = _qInit[i];
      _cmd[i].vel = 0.0;
      _cmd[i].tau = 0.0;
      _cmd[i].kp = _kp;
      _cmd[i].kd = _kd;
    }
    _cmd[0].tau = -0.65;

    DBGPRINT("MdlSineTest: qInit = [%f, %f, %f]\n", _qInit[0], _qInit[1], _qInit[2]);
    _startTime = _mgr->readTime();
    _initialized = true;
    return;
  }

  double t = _mgr->readTime() - _startTime;

  double q_ref  = _qInit[1] + _amplitude * sin(2.0 * M_PI * _frequency * t);
  double dq_ref = _amplitude * 2.0 * M_PI * _frequency * cos(2.0 * M_PI * _frequency * t);

  // Hip: hold at initial position with gravity compensation
  _cmd[0].pos = _qInit[0];
  _cmd[0].vel = 0.0;
  _cmd[0].tau = -0.65;

  // Thigh: sinusoidal reference
  _cmd[1].pos = q_ref;
  _cmd[1].vel = dq_ref;
  _cmd[1].tau = 0.0;

  // Calf: hold at initial position
  _cmd[2].pos = _qInit[2];
  _cmd[2].vel = 0.0;
  _cmd[2].tau = 0.0;

  for (int i = 0; i < 3; i++)
    _motorhw->setCommand(_idx[i], _cmd[i]);
}
