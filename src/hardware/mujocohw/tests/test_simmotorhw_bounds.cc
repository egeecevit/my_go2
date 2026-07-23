#include <cstdio>

#include "../MdlSimDriver.hh"
#include "../SimMotorHW.hh"
#include "rtcore/ModuleManager.hh"

// Referencing initHardware (never called) pulls SimHW.o into the link,
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

// Boundary behavior of SimMotorHW: index 11 is the last valid motor,
// index 12 (== max_index()) must be rejected by every method. The driver
// is constructed without a simulation; the accessors exercised here only
// touch its member arrays.
int main() {
  MdlSimDriver driver;
  SimMotorHW hw(&driver);

  const unsigned VALID = 11, INVALID = 12;

  // Valid boundary: command round trip through the driver
  MotorHW::cmd_t in;
  in.t = 1.0; in.pos = 0.5; in.vel = -0.25; in.tau = 1.5; in.kp = 20.0; in.kd = 0.5;
  hw.setCommand(VALID, in);
  MotorHW::cmd_t out;
  hw.getCommand(VALID, out);
  T_CHECK(out.t == 1.0);
  T_CHECK(out.pos == 0.5);
  T_CHECK(out.vel == -0.25);
  T_CHECK(out.tau == 1.5);
  T_CHECK(out.kp == 20.0);
  T_CHECK(out.kd == 0.5);

  // Valid boundary: enable round trip
  T_CHECK(!hw.isenabled(VALID));
  hw.enable(VALID);
  T_CHECK(hw.isenabled(VALID));
  hw.disable(VALID);
  T_CHECK(!hw.isenabled(VALID));

  // Invalid index: enable must not stick anywhere
  hw.enable(INVALID);
  T_CHECK(!hw.isenabled(INVALID));

  // Invalid index: setCommand is a no-op, getCommand/getState return the
  // zero structs instead of leaking whatever the caller passed in.
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

  printf("test_simmotorhw_bounds: all checks passed\n");
  return 0;
}
