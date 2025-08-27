/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <getopt.h>

#include "rtcore/Module.hh"
#include "rtcore/ThreadUtil.hh"
#include "rtcore/ModuleManager.hh"
#include "rtcore/ConfigTable.hh"

#include "hardware/MotorHW.hh"
#include "quadruped/CoreModules.hh"

#include "Supervisor.hh"

using namespace rtcore;

static ModuleManager * _mgr = nullptr;

void print_usage(const char* program_name) {
  printf("Usage: %s [OPTIONS]\n", program_name);
  printf("Options:\n");
  printf("  -c, --config CONFIG_STRING  Specify configuration string\n");
  printf("  -h, --help                  Show this help message and exit\n");
}

void exit_on_ctrl_c(int) {
  static bool control_c_invoked = false;
  if (control_c_invoked) return;
  control_c_invoked = true;
  if (!_mgr) exit(0);
  
  _mgr->message( "User Ctrl-C: Shutting down!" );
  _mgr->exitMainLoop();
}

int main( int argc, char **argv) {

  // Parse command line arguments
  std::string config_string;
  int option;
  struct option long_options[] = {
    {"config", required_argument, 0, 'c'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };
  
  while ((option = getopt_long(argc, argv, "c:h", long_options, nullptr)) != -1) {
    switch (option) {
      case 'c':
        
        config_string += optarg;
        config_string += "\n";
        break;
      case 'h':
        print_usage(argv[0]);
        return 0;
      case '?':
        print_usage(argv[0]);
        return 1;
      default:
        break;
    }
  }

  ModuleManager mm;

  // This is so that the Ctrl-C signal handler can access the module manager.
  _mgr = &mm;
  
  signal( SIGINT, exit_on_ctrl_c );
  signal( SIGTERM, exit_on_ctrl_c );

  // Print debug message if config string was provided
  if (config_string.length() > 0) {
    printf("Custom configuration string:\n%s", config_string.c_str());
  }

  mm.clearConfig();
  bool res;
  res = mm.appendConfigFile("list.toml");
  res = res && mm.appendConfigFile("versionlist.toml");
  res = res && mm.appendConfigFile("robotlist.toml");
  if (!mm.appendConfigFile("localoverrides.toml")) {
    mm.warning("main", "Could not find localoverrides.toml, skipping");
  }
  res = res && mm.appendConfigString(config_string.c_str());
  if (!res) mm.fatalError( "main", "Could not find one or more configuration files!");
  if (! mm.finalizeConfig() )
    mm.fatalError( "main", "Error readingconfiguration files!");

  //printf("*** CURRENT CONFIG ***\n");
  //mm.getConfigRoot()->print();
  //printf("**********************\n");

  initHardware( &mm );
  
  AddCoreModules( &mm );
  ActivateCoreModules( &mm );

  // This activates the supervisor, which in turn activates other modules
  Supervisor *sm = new Supervisor;
  mm.addModule(sm, 1, 0, USER_CONTROLLERS);
  mm.activateModule( sm );

  mm.message("\n** Current list of modules:");
  mm.printModules();
  mm.message("\n** Current list of threads:");
  ThreadUtil::printThreads();

  //mm.setStepPeriod( 2000 ); // OPTIONAL: Sets the update period to 1ms = 1000us
  mm.message("\n** Entering main loop...");
  mm.mainLoop();
  mm.message("\n** Main loop exited...");
  
  // This should also deactivate other modules
  mm.deactivateModule( sm );
  mm.removeModule( sm );
  delete sm;

  DeactivateCoreModules( &mm );
  RemoveCoreModules( &mm );
  
  mm.message("** Shutting down...");

  cleanupHardware();

  mm.shutdown();

  return 0;
}

