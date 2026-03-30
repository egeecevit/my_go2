#ifndef MDLHWTEST_HH
#define MDLHWTEST_HH

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "rtcore/Module.hh"
#include <string>
#include <termios.h>

#define HWTESTMODULE_NAME "MdlHWTest"

class MdlHWTest : public rtcore::Module {
public:
  MdlHWTest();
  ~MdlHWTest();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

private:
  // State machine
  enum State { READBACK, HOLD, SINGLE_JOINT, SINGLE_LEG, ALL_LEGS };
  static constexpr const char *stateNames[] = {
      "READBACK", "HOLD", "SINGLE_JOINT", "SINGLE_LEG", "ALL_LEGS"};

  State _state = READBACK;

  void enterState(State newState);
  void exitCurrentState();

  // Per-state update functions
  void updateReadback();
  void updateHold();
  void updateSingleJoint();
  void updateSingleLeg();
  void updateAllLegs();

  // Sine test helper: overlay sine on designated motors, hold all others
  void runSinePattern(const int *sineIndices, int sineCount);

  // Build and send all 12 motor commands (sets kp/kd, gravity comp on hips)
  void sendAllCommands();

  // Tracking error check for all 12 motors. Returns true if safe.
  bool checkTrackingError();

  // Hardware singletons
  MotorHW *_motorhw = nullptr;
  IMUHW *_imuhw = nullptr;

  // Keyboard
  struct termios _origTerm, _newTerm;
  bool _termConfigured = false;
  int readKey(); // Returns char or -1 if no key pressed

  // Configuration (from TOML [hwtest])
  int _testJoint = 0;             // motor index for SINGLE_JOINT
  std::string _testLeg = "FR";    // leg name for SINGLE_LEG
  int _legIndices[3] = {0, 1, 2}; // resolved motor indices for leg
  double _amplitude = 0.2;        // rad
  double _frequency = 0.2;        // Hz
  double _kp = 5.0;
  double _kd = 1.0;
  double _trackingLimit = 0.5; // rad

  // Home position (optional, from TOML)
  bool _hasHome = false;
  double _homePosition[12] = {};
  bool _ramping = false;
  double _rampLastTime = 0.0;
  static constexpr double RAMP_RATE = 0.5;          // rad/s
  static constexpr double HIP_GRAVITY_COMP = -0.65; // Nm

  // Joint limits (optional, from TOML)
  bool _hasLimits = false;
  double _jointMin[12] = {};
  double _jointMax[12] = {};

  // Hold state
  double _holdPosition[12] = {};

  // Sine state
  MotorHW::cmd_t _cmd[12] = {};
  bool _sineInitialized = false;
  double _startTime = 0.0;

  // Active motors tracking (for clean deactivate)
  static constexpr int MAX_MOTORS = 12;
  int _activeMotors[MAX_MOTORS] = {};
  int _activeCount = 0;

  void activateMotors(const int *indices, int count);
  void deactivateMotors();

  // Print throttle
  double _lastPrint = -10.0;
  static constexpr double PRINT_INTERVAL = 1.0; // seconds

  // Leg name -> motor index mapping
  void resolveLegIndices(const std::string &legName, int out[3]);

  // Snapshot current joint positions to gains.toml as home_position
  void snapshotHomePosition();
};

#endif
