#ifndef APP_COMMON_HH
#define APP_COMMON_HH

#include "rtcore/ModuleManager.hh"
#include <string>

// Parse standard CLI args: -c config_string, -n nosafety, -h help.
// Returns false if program should exit (e.g. --help).
// config_string and safety are populated by reference.
bool parseArgs(int argc, char **argv, std::string &config_string, bool &safety);

// Load the standard config file chain (list.toml, versionlist.toml, etc.)
// and append any extra config string. Calls fatalError on failure.
void loadConfig(rtcore::ModuleManager &mm, const std::string &config_string);

// Install SIGINT/SIGTERM handlers that call mm.exitMainLoop().
void installSignalHandlers(rtcore::ModuleManager *mm);

#endif
