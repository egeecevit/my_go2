#ifndef APP_COMMON_HH
#define APP_COMMON_HH

#include "rtcore/ModuleManager.hh"
#include <string>

// Parse standard CLI args: -c config_string, -h help.
// Returns false if program should exit (e.g. --help).
bool parseArgs(int argc, char **argv, std::string &config_string);

// Load the standard config file chain (list.toml, versionlist.toml, etc.)
// and append any extra config string. Calls fatalError on failure.
void loadConfig(rtcore::ModuleManager &mm, const std::string &config_string);

// Install SIGINT/SIGTERM handlers that call mm.exitMainLoop().
void installSignalHandlers(rtcore::ModuleManager *mm);

#endif
