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
#include "rtcore/ModuleManager.hh"
#include "rtcore/LogServer.hh"

#include "rtclient/LogClient.hh"
#include "rtclient/WriteASCII.hh"
#include "rtclient/WriteRaw.hh"
#include "rtclient/WriteML.hh"

#include "quadruped/MdlDrawSquare.hh"
#include "quadruped/MdlStand.hh"

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
  _stand = (MdlStand *)_mgr->findModule(STANDMODULE_NAME, 0);

  ConfigTable config;
  bool hasConfig = _mgr->getConfigTable("supervisor", config);

  if (hasConfig) {
    _exitTime = config.getDouble("exit_time", 0.0);
    _standHeight = config.getDouble("stand_height", 0.08);
    _sitHeight = config.getDouble("sit_height", -0.05);

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
  printf("\n  [S]tand  [D]own(sit)  [Q]uit\n\n");
}

void Supervisor::deactivate() {
  if (_state == S_STAND || _state == S_SIT)
    _mgr->releaseModule(_stand, this);
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

  // Quit from any state
  if (key == 'q' || key == 'Q') {
    _mgr->message("Supervisor: quit");
    if (_state == S_STAND || _state == S_SIT)
      _mgr->releaseModule(_stand, this);
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
      _state = S_STAND;
    }
    break;

  case S_STAND:
    if (_stand->getStatus() == MdlStand::ERROR) {
      _mgr->message("Supervisor: ERROR during stand");
      _mgr->releaseModule(_stand, this);
      _state = S_IDLE;
    } else if (_stand->getStatus() == MdlStand::SETTLED && !_standSettled) {
      _mgr->message("Supervisor: standing settled");
      _standSettled = true;
    }
    if (key == 'd' || key == 'D') {
      _mgr->message("Supervisor: -> S_SIT");
      _stand->setTargetHeight(0.0);  // return to activation pose
      _state = S_SIT;
    }
    break;

  case S_SIT:
    if (_stand->getStatus() == MdlStand::ERROR) {
      _mgr->message("Supervisor: ERROR during sit");
      _mgr->releaseModule(_stand, this);
      _state = S_IDLE;
    } else if (_stand->getStatus() == MdlStand::SETTLED) {
      _mgr->message("Supervisor: sitting settled, releasing");
      _mgr->releaseModule(_stand, this);
      _state = S_IDLE;
    }
    break;

  case S_EXIT:
    if (t - _mark > 0.5)
      _mgr->exitMainLoop();
    break;
  }
}
