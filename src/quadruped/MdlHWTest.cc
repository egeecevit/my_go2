#include <cmath>
#include <cstdio>
#include <unistd.h>

#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlHWTest.hh"

using namespace rtcore;

constexpr const char *MdlHWTest::stateNames[];

MdlHWTest::MdlHWTest() : Module(HWTESTMODULE_NAME, 0, SINGLE_USER) {}

MdlHWTest::~MdlHWTest() {
  if (_termConfigured)
    tcsetattr(0, TCSANOW, &_origTerm);
}

void MdlHWTest::resolveLegIndices(const std::string &legName, int out[3]) {
  // Motor ordering: FR=0,1,2  FL=3,4,5  RR=6,7,8  RL=9,10,11
  int base = 0;
  if (legName == "FR")
    base = 0;
  else if (legName == "FL")
    base = 3;
  else if (legName == "RR")
    base = 6;
  else if (legName == "RL")
    base = 9;
  else {
    _mgr->warning("MdlHWTest", "Unknown leg '%s', defaulting to FR",
                   legName.c_str());
    base = 0;
  }
  out[0] = base;
  out[1] = base + 1;
  out[2] = base + 2;
}

void MdlHWTest::init() {
  _motorhw = MotorHW::instance();
  _imuhw = IMUHW::instance();

  if (!_motorhw)
    _mgr->fatalError("MdlHWTest", "MotorHW not available!");

  // Read config
  ConfigTable cfg;
  if (_mgr->getConfigTable("hwtest", cfg)) {
    _testJoint = cfg.getInt("joint", _testJoint);
    _testLeg = cfg.getString("leg", _testLeg);
    _amplitude = cfg.getDouble("amplitude", _amplitude);
    _frequency = cfg.getDouble("frequency", _frequency);
    _kp = cfg.getDouble("kp", _kp);
    _kd = cfg.getDouble("kd", _kd);
    _trackingLimit = cfg.getDouble("tracking_error_limit", _trackingLimit);

    ConfigArray homeArr;
    if (cfg.getArray("home_position", homeArr) && homeArr.size() == 12) {
      _hasHome = true;
      for (int i = 0; i < 12; i++)
        _homePosition[i] = homeArr.getDoubleAt(i);
      _mgr->message("MdlHWTest: Home position configured.");
    }
  }

  resolveLegIndices(_testLeg, _legIndices);

  // Configure terminal for non-blocking keyboard input
  tcgetattr(0, &_origTerm);
  _newTerm = _origTerm;
  _newTerm.c_lflag &= ~ICANON;
  _newTerm.c_lflag &= ~ECHO;
  _newTerm.c_cc[VMIN] = 1;
  _newTerm.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &_newTerm);
  _termConfigured = true;
}

void MdlHWTest::uninit() {
  if (_termConfigured) {
    tcsetattr(0, TCSANOW, &_origTerm);
    _termConfigured = false;
  }
}

void MdlHWTest::activate() {
  _state = READBACK;
  _lastPrint = -10.0;
  printf("\n=== Go1 Hardware Test ===\n");
  printf("  [N]ext state  [B]ack to readback  [Q]uit\n\n");
}

void MdlHWTest::deactivate() { deactivateMotors(); }

int MdlHWTest::readKey() {
  char ch;
  int nread;
  _newTerm.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &_newTerm);
  nread = read(0, &ch, 1);
  _newTerm.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &_newTerm);
  if (nread == 1)
    return ch;
  return -1;
}

void MdlHWTest::activateMotors(const int *indices, int count) {
  for (int i = 0; i < count && _activeCount < MAX_MOTORS; i++) {
    _motorhw->enable(indices[i]);
    _activeMotors[_activeCount++] = indices[i];
  }
}

void MdlHWTest::deactivateMotors() {
  for (int i = 0; i < _activeCount; i++) {
    _motorhw->disable(_activeMotors[i]);
  }
  _activeCount = 0;
}

void MdlHWTest::exitCurrentState() {
  _sineInitialized = false;
  _ramping = false;
}

void MdlHWTest::enterState(State newState) {
  exitCurrentState();
  _state = newState;
  _lastPrint = -10.0;
  printf("\n--- Entering %s ---\n", stateNames[_state]);

  switch (_state) {
  case READBACK:
    deactivateMotors();
    break;
  case HOLD: {
    // Read current positions, set up hold commands
    for (int i = 0; i < 12; i++) {
      MotorHW::state_t st;
      _motorhw->getState(i, st);
      _holdPosition[i] = st.pos;
      _cmd[i].pos = st.pos;
      _cmd[i].vel = 0.0;
    }
    // Set commands BEFORE enabling motors to avoid one-cycle gap
    sendAllCommands();
    // NOW enable motors — commands are already set
    int all[12];
    for (int i = 0; i < 12; i++)
      all[i] = i;
    activateMotors(all, 12);
    printf("  Holding all motors at current positions.\n");
    if (_hasHome) {
      _ramping = true;
      _rampLastTime = _mgr->readTime();
      printf("  Ramping to home position...\n");
    }
    break;
  }
  case SINGLE_JOINT:
  case SINGLE_LEG:
  case ALL_LEGS:
    // Motors already active from HOLD
    break;
  }
}

void MdlHWTest::sendAllCommands() {
  for (int i = 0; i < 12; i++) {
    _cmd[i].kp = _kp;
    _cmd[i].kd = _kd;
    _cmd[i].tau = (i % 3 == 0) ? HIP_GRAVITY_COMP : 0.0;
    _motorhw->setCommand(i, _cmd[i]);
  }
}

bool MdlHWTest::checkTrackingError() {
  for (int i = 0; i < 12; i++) {
    MotorHW::state_t st;
    _motorhw->getState(i, st);
    double err = fabs(st.pos - _cmd[i].pos);
    if (err > _trackingLimit) {
      printf("\n!!! TRACKING ERROR on motor %d: err=%.3f rad (limit=%.3f) !!!\n",
             i, err, _trackingLimit);
      printf("    Falling back to READBACK for safety.\n");
      return false;
    }
  }
  return true;
}

void MdlHWTest::runSinePattern(const int *sineIndices, int sineCount) {
  double t = _mgr->readTime();

  if (!_sineInitialized) {
    _startTime = t;
    _sineInitialized = true;
    printf("  Sine on motor(s):");
    for (int i = 0; i < sineCount; i++)
      printf(" %d", sineIndices[i]);
    printf(" from hold position.\n");
    return;
  }

  double dt = t - _startTime;
  double q_sin = _amplitude * sin(2.0 * M_PI * _frequency * dt);
  double dq_sin = _amplitude * 2.0 * M_PI * _frequency *
                  cos(2.0 * M_PI * _frequency * dt);

  // All 12 motors hold at _holdPosition
  for (int i = 0; i < 12; i++) {
    _cmd[i].pos = _holdPosition[i];
    _cmd[i].vel = 0.0;
  }

  // Overlay sine on designated motors
  for (int k = 0; k < sineCount; k++) {
    int j = sineIndices[k];
    _cmd[j].pos = _holdPosition[j] + q_sin;
    _cmd[j].vel = dq_sin;
  }

  sendAllCommands();

  if (!checkTrackingError()) {
    enterState(READBACK);
    return;
  }

  if (t - _lastPrint >= PRINT_INTERVAL) {
    _lastPrint = t;
    printf("  [%s] t=%.1f", stateNames[_state], dt);
    for (int k = 0; k < sineCount; k++) {
      int j = sineIndices[k];
      MotorHW::state_t st;
      _motorhw->getState(j, st);
      printf("  m%d: cmd=%.3f act=%.3f err=%.3f",
             j, _cmd[j].pos, st.pos, fabs(_cmd[j].pos - st.pos));
    }
    printf("\n");
  }
}

void MdlHWTest::updateReadback() {
  double t = _mgr->readTime();
  if (t - _lastPrint < PRINT_INTERVAL)
    return;
  _lastPrint = t;

  unsigned int numMotors = _motorhw->max_index();
  printf("  [READBACK] t=%.1f\n", t);
  for (unsigned int i = 0; i < numMotors; i++) {
    MotorHW::state_t st;
    MotorHW::status_t status;
    _motorhw->getState(i, st);
    status = _motorhw->getStatus(i);
    const char *statusStr =
        (status == MotorHW::STATUS_READY)
            ? "READY"
            : (status == MotorHW::STATUS_STARTUP ? "STARTUP" : "ERROR");
    printf("    Motor %2d: pos=%7.3f  vel=%7.3f  tau=%6.3f  temp=%4.0fC  [%s]\n",
           i, st.pos, st.vel, st.tau, st.temp, statusStr);
  }

  if (_imuhw) {
    IMUHW::imudata_t imu;
    if (_imuhw->getLastReading(0, imu)) {
      printf("    IMU: quat=[%.2f, %.2f, %.2f, %.2f]  "
             "gyro=[%.2f, %.2f, %.2f]  "
             "acc=[%.2f, %.2f, %.2f]\n",
             imu.q.v[0], imu.q.v[1], imu.q.v[2], imu.q.v[3], imu.gyro.v[0],
             imu.gyro.v[1], imu.gyro.v[2], imu.acc.v[0], imu.acc.v[1],
             imu.acc.v[2]);
    } else {
      printf("    IMU: no data\n");
    }
  }
  printf("  [N]ext state  [B]ack to readback  [Q]uit\n");
}

void MdlHWTest::updateHold() {
  double t = _mgr->readTime();

  if (_ramping) {
    double dt = t - _rampLastTime;
    _rampLastTime = t;
    double step = RAMP_RATE * dt;
    bool allHome = true;
    for (int i = 0; i < 12; i++) {
      double diff = _homePosition[i] - _cmd[i].pos;
      if (diff > step) { _cmd[i].pos += step; allHome = false; }
      else if (diff < -step) { _cmd[i].pos -= step; allHome = false; }
      else { _cmd[i].pos = _homePosition[i]; }
      _cmd[i].vel = 0.0;
    }
    sendAllCommands();

    if (!checkTrackingError()) {
      enterState(READBACK);
      return;
    }

    if (t - _lastPrint >= PRINT_INTERVAL) {
      _lastPrint = t;
      int worstIdx = 0;
      double worstErr = 0.0;
      for (int i = 0; i < 12; i++) {
        MotorHW::state_t st;
        _motorhw->getState(i, st);
        double err = fabs(st.pos - _cmd[i].pos);
        if (err > worstErr) { worstErr = err; worstIdx = i; }
      }
      MotorHW::state_t wst;
      _motorhw->getState(worstIdx, wst);
      printf("  [RAMP] worst: m%d err=%.3f (act=%.3f cmd=%.3f -> %.3f)\n",
             worstIdx, worstErr, wst.pos, _cmd[worstIdx].pos,
             _homePosition[worstIdx]);
    }

    if (allHome) {
      _ramping = false;
      for (int i = 0; i < 12; i++)
        _holdPosition[i] = _homePosition[i];
      printf("  Home reached. Holding.\n");
    }
    return;
  }

  // Steady hold
  for (int i = 0; i < 12; i++) {
    _cmd[i].pos = _holdPosition[i];
    _cmd[i].vel = 0.0;
  }
  sendAllCommands();

  if (!checkTrackingError()) {
    enterState(READBACK);
    return;
  }

  if (t - _lastPrint >= PRINT_INTERVAL) {
    _lastPrint = t;
    printf("  [HOLD]");
    for (int i = 0; i < 12; i += 3) {
      MotorHW::state_t st;
      _motorhw->getState(i, st);
      printf("  m%d:%.3f", i, st.pos);
    }
    printf("\n");
    printf("  [N]ext state  [B]ack to readback  [Q]uit\n");
  }
}

void MdlHWTest::updateSingleJoint() {
  runSinePattern(&_testJoint, 1);
}

void MdlHWTest::updateSingleLeg() {
  int sineMotor = _legIndices[1]; // thigh gets the sine
  runSinePattern(&sineMotor, 1);
}

void MdlHWTest::updateAllLegs() {
  int thighs[] = {1, 4, 7, 10};
  runSinePattern(thighs, 4);
}

void MdlHWTest::update() {
  // Handle keyboard
  int key = readKey();
  if (key != -1) {
    char c = (char)key;
    if (c == 'q' || c == 'Q') {
      printf("\n--- Quit requested. Shutting down. ---\n");
      deactivateMotors();
      if (_termConfigured) {
        tcsetattr(0, TCSANOW, &_origTerm);
        _termConfigured = false;
      }
      _mgr->exitMainLoop();
      return;
    } else if (c == 'b' || c == 'B') {
      printf("\n--- Back to READBACK ---\n");
      enterState(READBACK);
      return;
    } else if (c == 'n' || c == 'N') {
      State next = READBACK;
      switch (_state) {
      case READBACK:
        next = HOLD;
        break;
      case HOLD:
        next = SINGLE_JOINT;
        break;
      case SINGLE_JOINT:
        next = SINGLE_LEG;
        break;
      case SINGLE_LEG:
        next = ALL_LEGS;
        break;
      case ALL_LEGS:
        printf("  (Already at last state. Press B for readback or Q to quit.)\n");
        return;
      }
      enterState(next);
      return;
    }
  }

  // Run current state
  switch (_state) {
  case READBACK:
    updateReadback();
    break;
  case HOLD:
    updateHold();
    break;
  case SINGLE_JOINT:
    updateSingleJoint();
    break;
  case SINGLE_LEG:
    updateSingleLeg();
    break;
  case ALL_LEGS:
    updateAllLegs();
    break;
  }
}
