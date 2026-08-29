/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>
#include "rtcore/ModuleManager.hh"
#include "rtcore/LogServer.hh"

#include "rtclient/LogClient.hh"
#include "rtclient/WriteASCII.hh"
#include "rtclient/WriteRaw.hh"
#include "rtclient/WriteML.hh"

#include "quadruped/MdlDrawSquare.hh"
#include "quadruped/MdlTrot.hh"
#include "quadruped/MdlStand.hh"
#include "quadruped/MdlSit.hh"
#include "quadruped/MdlPosVelEstimator.hh"

#include "Supervisor.hh"

// IMPORTANT NOTE: Be careful with enet functions since both rtcore and rtclient
// has them separately
using namespace rtcore;

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...) //printf(__VA_ARGS__);

Supervisor::Supervisor( ) : Module("supervisor", 0, SINGLE_USER) {
  DBGPRINT("Supervisor::Supervisor\n");
};

Supervisor::~Supervisor() {
  DBGPRINT("Supervisor::~Supervisor\n");

  terminate();

  if (_logclient) {
    delete _logclient;
    _logclient = nullptr;
  }
};

#define LOGTHREAD_USLEEP 10000
#define LOGTHREAD_MAX_RETRY 3

void Supervisor::threadEnter() {
  DBGPRINT("Supervisor::threadEnter\n");

  bool querydone = false;

  // Try to successfully register all variables
  while (!querydone) {
    double t = _mgr->readTime();

    if (_logclient->query()) {
      // Create a new log task if necessary
      if (!_logtask) _logtask = _logclient->newLog();

      // Attempt to add all the requested variables
      bool alladded = true;
      for (const auto& varname : _logvars) {
        DBGPRINT("Supervisor: Registering variable %s for logging.\n", varname.c_str());
        if (!_logtask->addVar(varname.c_str())) {
          _mgr->warning("Supervisor", "Failed to add %s for logging.", varname.c_str());
          alladded = false;
          _logtask = nullptr;
          _logretries++;
          if (_logretries > LOGTHREAD_MAX_RETRY) {
            _mgr->warning("Supervisor", "Too many retries to register logging variables (%d). Aborting.", _logretries);
            _logenable = false; 
            setFinish(true); // Stop the thread
            querydone = true;
            break;
          }
          _logclient->query();
          break;
        }
      }
      if (alladded) {
        if (_logformat == "ascii") {
          _logwriter = new rtclient::WriteASCII(_logfile.c_str(), _logtask->varList(),
                                                "Supervisor local data log");
        } else if (_logformat == "raw") {
          _logwriter = new rtclient::WriteRaw(_logfile.c_str(), _logtask->varList(),
                                              "Supervisor local data log");
        } else if (_logformat == "matlab") {
          _logwriter = new rtclient::WriteML(_logfile.c_str(), _logtask->varList(),
                                             "Supervisor local data log");
        }
        t = _mgr->readTime();
        _mgr->message("Supervisor: Found all variables at t=%.3f s", t);
        querydone = true;
        break;
      }
    } else {
      DBGPRINT("Supervisor: Unable to query LogServer\n");
      _logretries++;
      if (_logretries > LOGTHREAD_MAX_RETRY) {
        _mgr->warning("Supervisor", "Too many retries to register logging variables (%d). Aborting.", _logretries);
        _logenable = false; 
        setFinish(true); // Stop the thread
        querydone = true;
        break;
      }
}
    if (!querydone) usleep(10000);
  }
}

void Supervisor::threadLoop() {
  double t = _mgr->readTime();

  if (!_logstarted) {
    if (_logenable) {
      if (t < _logstart) waitSync(); 
      t = _mgr->readTime();
      _mgr->message("Supervisor: Starting logging at t=%.3f s", t);
      _logtask->startLog(_logperiod, 0);
      _logstarted = true;
    }
  } else {
    log_line_t *d;
    while(_logtask && _logwriter && (d = _logtask->getData(0))) _logwriter->appendLine(d);
    if (_logtask->isDone()) {
      _logstarted = false;
      _logtask = nullptr;
    }
  } 

  usleep(LOGTHREAD_USLEEP); // Convert milliseconds to microseconds
}

void Supervisor::threadExit() {
  DBGPRINT("Supervisor::threadExit\n");

  if (_logtask) {
    _logtask->abortLog();
    _logtask = nullptr;
  }

  if (_logwriter) {
    delete _logwriter;
    _logwriter = nullptr;
  }

  if (_logclient) {
    delete _logclient;
    _logclient = nullptr;
  }
}
// Defined in MdlSimDriver.cc (sim target only)
extern int simPollKey() __attribute__((weak));

int Supervisor::_readKey() {
  // Try GLFW key buffer first (sim target)
  if (simPollKey) {
    int k = simPollKey();
    if (k >= 0) return k;
  }
  // Fall back to stdin (non-sim targets or no GLFW key pending)
  struct termios orig, raw;
  tcgetattr(0, &orig);
  raw = orig;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &raw);
  char ch;
  int nread = read(0, &ch, 1);
  tcsetattr(0, TCSANOW, &orig);
  return (nread == 1) ? ch : -1;
}

void Supervisor::init() {
  DBGPRINT("Supervisor::init\n");

  // LogClient does not do its own enet initialization, so we must do it here
  rtclient::enet_initialize();

  _logserver = (LogServer*) _mgr->findModule(LOGSERVER_NAME, 0);
  _wm = (MdlDrawSquare*) _mgr->findModule(WALKMODULE_NAME, 0);
  _trot = (MdlTrot *)_mgr->findModule(TROTMODULE_NAME, 0);
  _stand = (MdlStand *)_mgr->findModule(STANDMODULE_NAME, 0);
  _sit = (MdlSit *)_mgr->findModule(SITMODULE_NAME, 0);
  _posvel = (MdlPosVelEstimator *)_mgr->findModule(POSVELMODULE_NAME, 0);

  ConfigTable config;
  bool hasConfig = _mgr->getConfigTable("supervisor", config);

  if (hasConfig) {
    _exitTime = config.getDouble("exit_time", 0.0);
    _standHeight = config.getDouble("stand_height", 0.08);
    _autorunTrotDuration = config.getDouble("autorun_trot_duration", 0.0);
    if (!std::isfinite(_autorunTrotDuration) || _autorunTrotDuration < 0.0) {
      _mgr->warning("Supervisor", "Invalid autorun_trot_duration; disabling autorun");
      _autorunTrotDuration = 0.0;
    }

    // Process logging configuration
    ConfigTable logconfig;
    bool hasLog = config.getTable("log", logconfig);
    if (hasLog) {
      bool logenable_config = logconfig.getBool("enable", false);
      _logstart = logconfig.getDouble("start", 0.0);
      _logfile = logconfig.getString("file_name", "supervisor.log");
      _logperiod = logconfig.getInt("period", 1);
      _logformat = logconfig.getString("file_format", "ascii");
      if (_logformat != "ascii" && _logformat != "raw" && _logformat != "matlab") {
        DBGPRINT("Supervisor: Unknown log format '%s'. Using 'ascii'.\n", _logformat.c_str());
        _logformat = "ascii";
      }

      _logvars.clear();

      // Read and process list of variables to log
      ConfigArray logvars;
      if (logconfig.getArray("vars", logvars)) {
        for (int i = 0; i < logvars.size(); i++) {
          std::string varname = logvars.getStringAt(i);
          if (!varname.empty()) {
            DBGPRINT("Supervisor: Adding variable %s to logging task.\n", varname.c_str());
            _logvars.push_back(varname);
          } else {
            DBGPRINT("Supervisor: Empty variable name at index %d\n", i);
          }
        }
      } else {
        DBGPRINT("Supervisor: No variables specified for logging.\n");
      }

      _logenable = false; // Disable until all parameters check out
      if (logenable_config && _logvars.size() != 0) {
        _mgr->message("Supervisor: Logging enabled to %s with %d variables", _logfile.c_str(), 
                      (int)_logvars.size());
        _logclient = new rtclient::LogClient("localhost", _logserver->getPort(), _logserver->getChannel());
        if (_logclient) {
          _logenable = true;
          _logstarted = false;
          start( "locallog", 0 ); // Start logging at low priority
        }
      }
    } else {
      DBGPRINT("Supervisor: No logging configuration found.\n");
    }
  }
}

void Supervisor::uninit() {
  DBGPRINT("Supervisor::uninit\n");

  terminate();
}

void Supervisor::activate() {
  _state = S_IDLE;
  _autorunStarted = false;
  _autorunTrotStarted = false;
  _autorunStopIssued = false;
  _autorunDone = false;
  printf("\n  [S]tand  [W]alk(draw)  [T]rot  [R]un-stop  [D]own(sit)  [Q]uit\n\n");
}

void Supervisor::deactivate() {
  if (_state == S_STAND)
    _mgr->releaseModule(_stand, this);
  if (_state == S_SIT)
    _mgr->releaseModule(_sit, this);
  if (_state == S_DRAW || _state == S_DRAW_STOPPING)
    _mgr->releaseModule(_wm, this);
  if (_state == S_TROT || _state == S_TROT_STOPPING)
    _mgr->releaseModule(_trot, this);
}

// The position/velocity filter only has something to measure while the feet
// carry load. Sitting and idling put the robot's weight on its hocks -- the
// simulator's own contact forces read under 1 N there, so no detector tuning
// brings the feet back -- and with nothing trusted the filter dead reckons on
// the accelerometer alone, running away quadratically (2.5 m over a 15 s sit).
//
// So it is gated on the behavior rather than on a contact signal. Standing is
// the mandatory gateway to every other state, which makes this the one edge
// where the robot is provably up: activate() reseeds through reset(), and the
// warm start waits for a loaded foot on its own. Orientation is a passthrough
// with nothing to integrate, so it keeps running throughout.
//
// Horizontal position survives the outage: nothing in that filter observes
// absolute x and y, so it carries the datum across rather than re-deriving it.
// See MdlPosVelEstimator::_originCarry.
void Supervisor::_setEstimation(bool on) {
  if (!_posvel) return;
  if (on)
    _mgr->activateModule(_posvel);
  else
    _mgr->deactivateModule(_posvel);
}

void Supervisor::update() {
  double t = _mgr->readTime();

  if (_exitTime > 0 && t >= _exitTime) {
    _mgr->message("\nSupervisor: Exiting at t=%.3f s", t);
    _mgr->exitMainLoop();
    return;
  }

  if (_logenable && !_logstarted && t >= _logstart) sendSync();

  int key = _readKey();

  // Synthesize only an otherwise absent key, so an operator can always abort
  // an autorun with Q. The state transitions below remain the single source of
  // behavior ownership; autorun merely exercises the same edges deterministically.
  if (_autorunTrotDuration > 0.0 && key < 0) {
    if (_state == S_IDLE && !_autorunStarted) {
      key = 's';
    } else if (_state == S_IDLE && _autorunDone) {
      _mgr->message("Supervisor: autorun complete at t=%.3f s", t);
      _mgr->exitMainLoop();
      return;
    } else if (_state == S_STAND && _standSettled && !_autorunTrotStarted) {
      key = 't';
    } else if (_state == S_TROT && _autorunTrotStarted && !_autorunStopIssued &&
               t - _autorunTrotStart >= _autorunTrotDuration) {
      key = 'd';
    }
  }

  // Quit from any state
  if (key == 'q' || key == 'Q') {
    _mgr->message("Supervisor: quit");
    if (_state == S_STAND)
      _mgr->releaseModule(_stand, this);
    if (_state == S_SIT)
      _mgr->releaseModule(_sit, this);
    if (_state == S_DRAW || _state == S_DRAW_STOPPING)
      _mgr->releaseModule(_wm, this);
    if (_state == S_TROT || _state == S_TROT_STOPPING)
      _mgr->releaseModule(_trot, this);
    _state = S_EXIT;
    _mark = t;
  }

  switch (_state) {
  case S_IDLE:
    if (key == 's' || key == 'S') {
      _mgr->message("Supervisor: -> S_STAND");
      _mgr->grabModule(_stand, this);
      _stand->setTargetHeight(_standHeight);
      _standSettled = false;
      _setEstimation(true);
      if (_autorunTrotDuration > 0.0) _autorunStarted = true;
      _state = S_STAND;
    }
    break;

  case S_STAND:
    if (_stand->getStatus() == MdlStand::ERROR) {
      _mgr->message("Supervisor: ERROR during stand");
      _mgr->releaseModule(_stand, this);
      _setEstimation(false);
      if (_autorunStarted) _autorunDone = true;
      _state = S_IDLE;
    } else {
      if (_stand->getStatus() == MdlStand::SETTLED && !_standSettled) {
        _mgr->message("Supervisor: standing settled");
        _standSettled = true;
      }
      // Only allow transitions once standing is stable
      if (_standSettled) {
        if (key == 'w' || key == 'W') {
          _mgr->message("Supervisor: -> S_DRAW");
          _mgr->releaseModule(_stand, this);
          _mgr->grabModule(_wm, this);
          _state = S_DRAW;
        } else if (key == 't' || key == 'T') {
          _mgr->message("Supervisor: -> S_TROT");
          _mgr->releaseModule(_stand, this);
          _mgr->grabModule(_trot, this);
          if (_autorunTrotDuration > 0.0) {
            _autorunTrotStarted = true;
            _autorunTrotStart = t;
          }
          _state = S_TROT;
        } else if (key == 'd' || key == 'D') {
          _mgr->message("Supervisor: -> S_SIT");
          _mgr->releaseModule(_stand, this);
          _mgr->grabModule(_sit, this);
          _state = S_SIT;
        }
      }
    }
    break;

  case S_DRAW:
    // MdlDrawSquare runs indefinitely; press D to ask it to stop and recenter
    if (key == 'd' || key == 'D') {
      _mgr->message("Supervisor: -> S_DRAW_STOPPING");
      _wm->stopDrawing();
      _state = S_DRAW_STOPPING;
    }
    break;

  case S_DRAW_STOPPING:
    if (_wm->isStopped()) {
      _mgr->message("Supervisor: -> S_SIT (from draw)");
      _mgr->releaseModule(_wm, this);
      _mgr->grabModule(_sit, this);
      _state = S_SIT;
    }
    break;

  case S_TROT:
    // MdlTrot runs indefinitely. R stops and stands back up, D stops and sits
    // down; either way MdlTrot ramps the gait down first, so the key only picks
    // the destination.
    if (_trot->getStatus() == MdlTrot::ERROR) {
      _mgr->message("Supervisor: ERROR during trot, sitting down");
      _mgr->releaseModule(_trot, this);
      _mgr->grabModule(_sit, this);
      if (_autorunStarted) _autorunStopIssued = true;
      _state = S_SIT;
    } else if (key == 'r' || key == 'R' || key == 'd' || key == 'D') {
      _trotStopToStand = (key == 'r' || key == 'R');
      _mgr->message("Supervisor: -> S_TROT_STOPPING (to %s)",
                    _trotStopToStand ? "stand" : "sit");
      _trot->stopTrotting();
      if (_autorunStarted) _autorunStopIssued = true;
      _state = S_TROT_STOPPING;
    }
    break;

  case S_TROT_STOPPING:
    // An error on the way down is not a state to stand up out of, so it always
    // sits regardless of which key started the stop.
    if (_trot->getStatus() == MdlTrot::ERROR) {
      _mgr->message("Supervisor: ERROR during trot stop, sitting down");
      _mgr->releaseModule(_trot, this);
      _mgr->grabModule(_sit, this);
      _state = S_SIT;
    } else if (_trot->isStopped()) {
      _mgr->releaseModule(_trot, this);
      if (_trotStopToStand) {
        // Zero delta, not _standHeight. setTargetHeight() takes an offset from
        // the pose the module is activated in, and MdlTrot has just centered the
        // robot at its trot height; asking for another _standHeight on top of
        // that puts the target outside the leg's workspace and MdlStand errors
        // out. The stand-up delta belongs on the edge out of a sit, which is the
        // only place the robot is actually low.
        _mgr->message("Supervisor: -> S_STAND (from trot)");
        _mgr->grabModule(_stand, this);
        _stand->setTargetHeight(0.0);
        _standSettled = false;
        _state = S_STAND;
      } else {
        _mgr->message("Supervisor: -> S_SIT (from trot)");
        _mgr->grabModule(_sit, this);
        _state = S_SIT;
      }
    }
    break;

  case S_SIT:
    if (_sit->getStatus() == MdlSit::ERROR) {
      _mgr->message("Supervisor: ERROR during sit");
      _mgr->releaseModule(_sit, this);
      _setEstimation(false);
      if (_autorunStarted) _autorunDone = true;
      _state = S_IDLE;
    } else if (_sit->getStatus() == MdlSit::SETTLED) {
      _mgr->message("Supervisor: sitting settled, releasing");
      _mgr->releaseModule(_sit, this);
      _setEstimation(false);
      if (_autorunStarted) _autorunDone = true;
      _state = S_IDLE;
    }
    break;

  case S_EXIT:
    if (t - _mark > 0.5)
      _mgr->exitMainLoop();
    break;
  }
}
