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

#include "rtcore/Module.hh"
#include "rtcore/ThreadUtil.hh"
#include "rtcore/ModuleManager.hh"

#include "hardware/MotorHW.hh"
#include "quadruped/CoreModules.hh"
#include "quadruped/MdlSineTest.hh"

#include "app_common.hh"
#include "Supervisor.hh"

using namespace rtcore;

// This module monitors the keyboard, if any key is hit, exits the
// module manager loop. To this end, it reconfigures the terminal to
// immediately issue all keyboard presses.
class SafetyModule : public Module {

public:
  SafetyModule ( void ) : Module( "safety", 0, false ) {
    peek = -1;
    tcgetattr( 0, &origTerm );
    newTerm = origTerm;
    newTerm.c_lflag &= ~ICANON; //Disable canonical mode.(Do not buffer by line )
    newTerm.c_lflag &= ~ECHO;   // Do not echo input characters
    newTerm.c_cc[ VMIN ] = 1;
    newTerm.c_cc[ VTIME ] = 0;
    tcsetattr( 0, TCSANOW, &newTerm );
  };
  ~SafetyModule ( void ) {
    tcsetattr( 0, TCSANOW, &origTerm );
  }

  int kbhit( void ) {
    char ch;
    int nread;

    if( peek != -1 ) return 1;
    newTerm.c_cc[ VMIN ] = 0;
    tcsetattr( 0, TCSANOW, &newTerm );
    nread = read( 0, &ch, 1 );
    newTerm.c_cc[ VMIN ] = 1;
    tcsetattr( 0, TCSANOW, &newTerm );
    if ( nread == 1 ) {
      peek = ch;
      return 1;
    }
    return 0;
  }
    
  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void ) {

  if ( kbhit() ) {
     _mgr->message( "User interrupt: Shutting down!" );
     tcsetattr( 0, TCSANOW, &origTerm );
     _mgr->exitMainLoop( );
   }
  }
private:
  struct termios origTerm, newTerm;
  int peek;
};

int main(int argc, char **argv) {

  std::string config_string;
  bool safety;
  if (!parseArgs(argc, argv, config_string, safety))
    return 0;

  ModuleManager mm;
  installSignalHandlers(&mm);
  loadConfig(mm, config_string);

  initHardware(&mm);

  AddCoreModules(&mm);
  ActivateCoreModules(&mm);

  SafetyModule *smod = nullptr;
  if (safety) {
    smod = new SafetyModule;
    mm.addModule( smod, 10, 0, OTHER_MODULES );
    mm.activateModule( smod );
  }
  
  Supervisor *sm = new Supervisor;
  mm.addModule(sm, 1, 0, USER_CONTROLLERS);
  mm.activateModule(sm);

  // MdlSineTest *mst = new MdlSineTest;
  // mm.addModule(mst, 1, 0, BEHAVIORAL_CONTROLLERS);
  // mm.activateModule(mst);

  mm.message("\n** Current list of modules:");
  mm.printModules();
  mm.message("\n** Current list of threads:");
  ThreadUtil::printThreads();

  mm.message("\n** Entering main loop...");
  mm.mainLoop();
  mm.message("\n** Main loop exited...");

  mm.deactivateModule(sm);
  mm.removeModule(sm);
  delete sm;

  DeactivateCoreModules(&mm);
  RemoveCoreModules(&mm);

  mm.message("** Shutting down...");
  cleanupHardware();
  mm.shutdown();

  return 0;
}

