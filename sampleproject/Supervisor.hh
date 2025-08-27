/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef _SUPERVISOR_HH
#define _SUPERVISOR_HH

#include "rtcore/Module.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ThreadedLoop.hh"

#include "rtclient/LogClient.hh"
#include "rtclient/LogWriter.hh"

class MdlDrawSquare;

/** \brief Top-level supervisor module for quadruped control

  This class implements the top level behavioral supervision functionality in
  the form of a Module. It has an internal state machine with states defined by
  Supervisor::_state_t, which governs the activation/deactivation of various
  behavioral modules available to the platform. 

  In addition, it also incorporates a ThreadedLoop instance, which is used to
  perform local logging of data variables to a file, primarily intended for
  simulation environments. Various components of this logging system can be
  configured through the supervisor.log table entry in the global ModuleManager
  configuration database.

  See supervisor.toml file for configuration options and default values.

 */
class Supervisor : public rtcore::Module, rtcore::ThreadedLoop {
public:
  Supervisor( );
  ~Supervisor();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  // The threaded loop is for local logging
  void threadEnter();
  void threadLoop();
  void threadExit();

private:
  /** \brief Possible states for the supervisory state machine */
  typedef enum { S_INIT, S_WALK, S_EXIT } _state_t;
  /** \brief Current state */
  int _state = S_INIT;
  
  MdlDrawSquare *_wm = nullptr;
  double _mark = 0; // Temporary variable to store time of state transitions
  double _last_print = 0; // Track last time we printed the current time
  double _exitTime = 0;   // Exit ModuleManager main loop after this much time

  // Configuration and components for local data logging
  rtcore::LogServer *_logserver = nullptr;
  bool _logenable = false;
  bool _logstarted = false;
  int _logretries = 0;
  // Starting time in seconds for logging. 0 means start immediately
  double _logstart = 0;
  unsigned int _logperiod = 1; // Default log period in milliseconds
  // Name of the log file to write to
  std::string _logfile;
  // Log format to use, can be "ascii", "raw", or "matlab"
  std::string _logformat = "ascii";
  // List of variables configured through supervisor.log.vars
  std::vector<std::string> _logvars;

  rtclient::LogClient *_logclient = nullptr;
  rtclient::LogTask   *_logtask = nullptr;
  rtclient::LogWriter *_logwriter = nullptr;
};

#endif
