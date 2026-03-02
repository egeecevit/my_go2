#include <stdio.h>

#include "rtcore/Module.hh"
#include "rtcore/ThreadUtil.hh"
#include "rtcore/ModuleManager.hh"

#include "hardware/MotorHW.hh"
#include "quadruped/CoreModules.hh"
#include "quadruped/MdlHWTest.hh"

#include "app_common.hh"

using namespace rtcore;

int main(int argc, char **argv) {

  std::string config_string;
  bool safety; // unused in test binary, but parseArgs expects it
  if (!parseArgs(argc, argv, config_string, safety))
    return 0;

  ModuleManager mm;
  installSignalHandlers(&mm);
  loadConfig(mm, config_string);

  initHardware(&mm);

  AddCoreModules(&mm);
  ActivateCoreModules(&mm);

  // No SafetyModule — MdlHWTest handles keyboard directly
  MdlHWTest *hwtest = new MdlHWTest;
  mm.addModule(hwtest, 1, 0, USER_CONTROLLERS);
  mm.activateModule(hwtest);

  mm.message("\n** Current list of modules:");
  mm.printModules();
  mm.message("\n** Current list of threads:");
  ThreadUtil::printThreads();

  mm.message("\n** Entering main loop...");
  mm.mainLoop();
  mm.message("\n** Main loop exited...");

  mm.deactivateModule(hwtest);
  mm.removeModule(hwtest);
  delete hwtest;

  DeactivateCoreModules(&mm);
  RemoveCoreModules(&mm);

  mm.message("** Shutting down...");
  cleanupHardware();
  mm.shutdown();

  return 0;
}
