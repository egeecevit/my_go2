#include <cstdio>

#include "../CANMotorHW.hh"
#include "../MdlMotorMaster.hh"
#include "rtcore/ModuleManager.hh"

// Referencing initHardware (never called) pulls RobotHW.o into the link,
// which carries the HARDWARE_IMPL singleton statics rtcore needs.
static void (*const _pull_hw_impl)(rtcore::ModuleManager *) = initHardware;

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                  \
  do {                                                               \
    if (!(cond)) {                                                   \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                      \
    }                                                                \
  } while (0)

// Boundary behavior of CANMotorHW: index 11 is the last valid motor,
// index 12 (== max_index()) must be rejected by every method. The master
// is constructed without init(), so no CAN buses exist — valid-index
// coverage goes through the enable family, which only touches _enabled;
// invalid-index delegate calls are no-ops behind the master's own guard.
int main() {
  MdlMotorMaster master;
  CANMotorHW hw(&master);

  const unsigned VALID = 11, INVALID = 12;

  // Fresh object: nothing may report enabled (guards _enabled zero-init)
  for (unsigned i = 0; i < 12; i++)
    T_CHECK(!hw.isenabled(i));

  // Valid boundary: enable round trip
  hw.enable(VALID);
  T_CHECK(hw.isenabled(VALID));
  hw.disable(VALID);
  T_CHECK(!hw.isenabled(VALID));

  // Invalid index: enable must not stick
  hw.enable(INVALID);
  T_CHECK(!hw.isenabled(INVALID));

  // Invalid index: setCommand must not reach the bus layer, and
  // getCommand/getState must return the zero structs instead of leaving
  // the caller's buffer untouched.
  MotorHW::cmd_t junk;
  junk.t = 99.0; junk.pos = 99.0;
  hw.setCommand(INVALID, junk);

  MotorHW::cmd_t cmd;
  cmd.t = 99.0; cmd.pos = 99.0;
  hw.getCommand(INVALID, cmd);
  T_CHECK(cmd.t == -1);
  T_CHECK(cmd.pos == 0);
  T_CHECK(cmd.vel == 0);
  T_CHECK(cmd.tau == 0);
  T_CHECK(cmd.kp == 0);
  T_CHECK(cmd.kd == 0);

  MotorHW::state_t state;
  state.t = 99.0; state.pos = 99.0;
  hw.getState(INVALID, state);
  T_CHECK(state.t == -1);
  T_CHECK(state.pos == 0);
  T_CHECK(state.vel == 0);
  T_CHECK(state.tau == 0);
  T_CHECK(state.temp == 0);

  // Invalid index: status reports an error, not READY
  T_CHECK(hw.getStatus(INVALID) == MotorHW::STATUS_ERROR);
  T_CHECK(hw.getStatus(VALID) == MotorHW::STATUS_READY);

  printf("test_canmotorhw_bounds: all checks passed\n");
  return 0;
}
