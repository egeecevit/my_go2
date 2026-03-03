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
  for (int i = 0; i < count; i++) {
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
  _warmup = 0;
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
    _holdInitialized = false;
    break;
  case HOLD: {
    // Set commands BEFORE enabling motors to avoid one-cycle gap
    // where motors would be active with default cmd (kp=0, kd=0)
    int hips[] = {0, 3, 6, 9};
    for (int i = 0; i < 12; i++) {
      MotorHW::state_t st;
      _motorhw->getState(i, st);
      _holdPosition[i] = st.pos;
      _cmd[i].pos = st.pos;
      _cmd[i].vel = 0.0;
      _cmd[i].kp = _kp;
      _cmd[i].kd = _kd;
      _cmd[i].tau = 0.0;
      _motorhw->setCommand(i, _cmd[i]);
    }
    for (int h : hips) {
      _cmd[h].tau = -0.65;
      _motorhw->setCommand(h, _cmd[h]);
    }
    // NOW enable motors — commands are already set
    int all[12];
    for (int i = 0; i < 12; i++)
      all[i] = i;
    activateMotors(all, 12);
    _holdInitialized = true;
    printf("  Holding all motors at current positions.\n");
    if (_hasHome) {
      _ramping = true;
      _rampStartTime = _mgr->readTime();
      for (int i = 0; i < 12; i++)
        _rampStart[i] = _holdPosition[i];
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

bool MdlHWTest::checkTrackingError(const int *indices, int count) {
  for (int i = 0; i < count; i++) {
    MotorHW::state_t st;
    _motorhw->getState(indices[i], st);
    MotorHW::cmd_t cmd;
    _motorhw->getCommand(indices[i], cmd);
    double err = fabs(st.pos - cmd.pos);
    if (err > _trackingLimit) {
      printf("\n!!! TRACKING ERROR on motor %d: err=%.3f rad (limit=%.3f) !!!\n",
             indices[i], err, _trackingLimit);
      printf("    Falling back to READBACK for safety.\n");
      return false;
    }
  }
  return true;
}

void MdlHWTest::runSinePattern(const int *indices, int count, int sineIdx) {
  double t = _mgr->readTime();
  int hips[] = {0, 3, 6, 9};

  if (!_sineInitialized) {
    for (int i = 0; i < count; i++)
      _qInit[i] = _holdPosition[indices[i]];
    _startTime = t;
    _sineInitialized = true;
    printf("  Sine on motor %d from hold position.\n", indices[sineIdx]);
    return;
  }

  double dt = t - _startTime;
  double q_ref = _qInit[sineIdx] +
                 _amplitude * sin(2.0 * M_PI * _frequency * dt);
  double dq_ref =
      _amplitude * 2.0 * M_PI * _frequency * cos(2.0 * M_PI * _frequency * dt);

  // Command all 12 motors: hold at _holdPosition
  for (int i = 0; i < 12; i++) {
    MotorHW::cmd_t cmd;
    cmd.pos = _holdPosition[i];
    cmd.vel = 0.0;
    cmd.kp = _kp;
    cmd.kd = _kd;
    cmd.tau = 0.0;
    _motorhw->setCommand(i, cmd);
  }

  // Overlay: test motors hold at _qInit, sine motor oscillates
  for (int i = 0; i < count; i++) {
    MotorHW::cmd_t cmd;
    cmd.pos = _qInit[i];
    cmd.vel = 0.0;
    cmd.kp = _kp;
    cmd.kd = _kd;
    cmd.tau = 0.0;
    _motorhw->setCommand(indices[i], cmd);
  }
  {
    MotorHW::cmd_t cmd;
    cmd.pos = q_ref;
    cmd.vel = dq_ref;
    cmd.kp = _kp;
    cmd.kd = _kd;
    cmd.tau = 0.0;
    _motorhw->setCommand(indices[sineIdx], cmd);
  }

  // Gravity comp on hips
  for (int h : hips) {
    MotorHW::cmd_t cmd;
    _motorhw->getCommand(h, cmd);
    cmd.tau = -0.65;
    _motorhw->setCommand(h, cmd);
  }

  // Print
  if (t - _lastPrint >= PRINT_INTERVAL) {
    _lastPrint = t;
    printf("  [%s] t=%.1f", stateNames[_state], dt);
    for (int i = 0; i < count; i++) {
      MotorHW::state_t st;
      _motorhw->getState(indices[i], st);
      MotorHW::cmd_t c;
      _motorhw->getCommand(indices[i], c);
      printf("  m%d: cmd=%.3f act=%.3f err=%.3f", indices[i], c.pos,
             st.pos, fabs(c.pos - st.pos));
    }
    printf("\n");
  }

  int all[12];
  for (int i = 0; i < 12; i++) all[i] = i;
  if (!checkTrackingError(all, 12))
    enterState(READBACK);
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
  int hips[] = {0, 3, 6, 9};

  if (!_holdInitialized) {
    for (int i = 0; i < 12; i++) {
      MotorHW::state_t st;
      _motorhw->getState(i, st);
      _holdPosition[i] = st.pos;
      _cmd[i].pos = st.pos;
      _cmd[i].vel = 0.0;
      _cmd[i].kp = _kp;
      _cmd[i].kd = _kd;
      _cmd[i].tau = 0.0;
      _motorhw->setCommand(i, _cmd[i]);
    }
    for (int h : hips) {
      _cmd[h].tau = -0.65;
      _motorhw->setCommand(h, _cmd[h]);
    }
    _holdInitialized = true;
    printf("  Holding all motors at current positions.\n");

    if (_hasHome) {
      _ramping = true;
      _rampStartTime = t;
      for (int i = 0; i < 12; i++)
        _rampStart[i] = _holdPosition[i];
      printf("  Ramping to home position...\n");
    }
    return;
  }

  if (_ramping) {
    double elapsed = t - _rampStartTime;
    double alpha = elapsed / RAMP_DURATION;
    if (alpha > 1.0) alpha = 1.0;

    for (int i = 0; i < 12; i++) {
      _cmd[i].pos = _rampStart[i] + alpha * (_homePosition[i] - _rampStart[i]);
      _cmd[i].vel = 0.0;
      _cmd[i].kp = _kp;
      _cmd[i].kd = _kd;
      _cmd[i].tau = 0.0;
      _motorhw->setCommand(i, _cmd[i]);
    }
    for (int h : hips) {
      _cmd[h].tau = -0.65;
      _motorhw->setCommand(h, _cmd[h]);
    }

    if (t - _lastPrint >= PRINT_INTERVAL) {
      _lastPrint = t;
      int thighs[] = {1, 4, 7, 10};
      printf("  [RAMP] %.0f%%", alpha * 100.0);
      for (int j : thighs) {
        MotorHW::state_t st;
        _motorhw->getState(j, st);
        printf("  m%d: %.3f->%.3f (at %.3f)", j,
               _rampStart[j], _homePosition[j], st.pos);
      }
      printf("\n");
    }

    int all[12];
    for (int i = 0; i < 12; i++) all[i] = i;
    if (!checkTrackingError(all, 12)) {
      enterState(READBACK);
      return;
    }

    if (alpha >= 1.0) {
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
    _cmd[i].kp = _kp;
    _cmd[i].kd = _kd;
    _cmd[i].tau = 0.0;
    _motorhw->setCommand(i, _cmd[i]);
  }
  for (int h : hips) {
    _cmd[h].tau = -0.65;
    _motorhw->setCommand(h, _cmd[h]);
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

  int all[12];
  for (int i = 0; i < 12; i++) all[i] = i;
  if (!checkTrackingError(all, 12))
    enterState(READBACK);
}

void MdlHWTest::updateSingleJoint() {
  runSinePattern(&_testJoint, 1, 0);
}

void MdlHWTest::updateSingleLeg() {
  // sineIdx=1 means thigh gets the sine (hip=0 holds, calf=2 holds)
  runSinePattern(_legIndices, 3, 1);
}

void MdlHWTest::updateAllLegs() {
  double t = _mgr->readTime();
  int hips[] = {0, 3, 6, 9};
  int thighs[] = {1, 4, 7, 10};

  if (!_sineInitialized) {
    for (int i = 0; i < 12; i++)
      _qInit[i] = _holdPosition[i];
    _startTime = t;
    _sineInitialized = true;
    printf("  All legs sine from hold positions.\n");
    return;
  }

  double dt = t - _startTime;
  double q_sin = _amplitude * sin(2.0 * M_PI * _frequency * dt);
  double dq_sin =
      _amplitude * 2.0 * M_PI * _frequency * cos(2.0 * M_PI * _frequency * dt);

  // Command all 12 motors: hold at _qInit
  for (int i = 0; i < 12; i++) {
    MotorHW::cmd_t cmd;
    cmd.pos = _qInit[i];
    cmd.vel = 0.0;
    cmd.kp = _kp;
    cmd.kd = _kd;
    cmd.tau = 0.0;
    _motorhw->setCommand(i, cmd);
  }

  // Overlay sine on thighs
  for (int j : thighs) {
    MotorHW::cmd_t cmd;
    cmd.pos = _qInit[j] + q_sin;
    cmd.vel = dq_sin;
    cmd.kp = _kp;
    cmd.kd = _kd;
    cmd.tau = 0.0;
    _motorhw->setCommand(j, cmd);
  }

  // Gravity comp on hips
  for (int h : hips) {
    MotorHW::cmd_t cmd;
    _motorhw->getCommand(h, cmd);
    cmd.tau = -0.65;
    _motorhw->setCommand(h, cmd);
  }

  // Print
  if (t - _lastPrint >= PRINT_INTERVAL) {
    _lastPrint = t;
    printf("  [ALL_LEGS] t=%.1f", dt);
    for (int j : thighs) {
      MotorHW::state_t st;
      _motorhw->getState(j, st);
      MotorHW::cmd_t c;
      _motorhw->getCommand(j, c);
      printf("  m%d:err=%.3f", j, fabs(c.pos - st.pos));
    }
    printf("\n");
  }

  int all[12];
  for (int i = 0; i < 12; i++) all[i] = i;
  if (!checkTrackingError(all, 12))
    enterState(READBACK);
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
