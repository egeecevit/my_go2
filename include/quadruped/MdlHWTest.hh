#ifndef MDLHWTEST_HH
#define MDLHWTEST_HH

#include "rtcore/Module.hh"
#include "hardware/MotorHW.hh"
#include "hardware/IMUHW.hh"
#include <termios.h>
#include <string>

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
  enum State { READBACK, SINGLE_JOINT, SINGLE_LEG, ALL_LEGS };
  static constexpr const char *stateNames[] = {
      "READBACK", "SINGLE_JOINT", "SINGLE_LEG", "ALL_LEGS"};

  State _state = READBACK;

  void enterState(State newState);
  void exitCurrentState();

  // Per-state update functions
  void updateReadback();
  void updateSingleJoint();
  void updateSingleLeg();
  void updateAllLegs();

  // Sine test helper: run sine on a set of motors
  // indices: array of motor indices, count: number of motors
  // sineIdx: which motor in the set gets the sine (others hold)
  void runSinePattern(const int *indices, int count, int sineIdx);

  // Tracking error check for active motors. Returns true if safe.
  bool checkTrackingError(const int *indices, int count);

  // Hardware singletons
  MotorHW *_motorhw = nullptr;
  IMUHW *_imuhw = nullptr;

  // Keyboard
  struct termios _origTerm, _newTerm;
  bool _termConfigured = false;
  int readKey(); // Returns char or -1 if no key pressed

  // Configuration (from TOML [hwtest])
  int _testJoint = 0;        // motor index for SINGLE_JOINT
  std::string _testLeg = "FR"; // leg name for SINGLE_LEG
  int _legIndices[3] = {0, 1, 2}; // resolved motor indices for leg
  double _amplitude = 0.2;   // rad
  double _frequency = 0.2;   // Hz
  double _kp = 5.0;
  double _kd = 1.0;
  double _trackingLimit = 0.5; // rad

  // Home position (optional, from TOML)
  bool _hasHome = false;
  double _homePosition[12] = {};
  double _rampStart[12] = {};
  bool _ramping = false;
  double _rampStartTime = 0.0;
  static constexpr double RAMP_DURATION = 2.0; // seconds

  // Sine state
  double _qInit[12] = {};
  MotorHW::cmd_t _cmd[12] = {};
  int _warmup = 0;
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
};

#endif
