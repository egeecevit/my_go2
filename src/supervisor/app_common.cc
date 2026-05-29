#include "app_common.hh"

#include <stdio.h>
#include <signal.h>
#include <getopt.h>

using namespace rtcore;

static ModuleManager *_signal_mgr = nullptr;

static void exit_on_ctrl_c(int) {
  static bool control_c_invoked = false;
  if (control_c_invoked) return;
  control_c_invoked = true;
  if (!_signal_mgr) exit(0);
  _signal_mgr->message("User Ctrl-C: Shutting down!");
  _signal_mgr->exitMainLoop();
}

static void print_usage(const char *program_name) {
  printf("Usage: %s [OPTIONS]\n", program_name);
  printf("Options:\n");
  printf("  -c, --config CONFIG_STRING  Specify configuration string\n");
  printf("  -n, --nosafety              Disable safety exit with the keyboard\n");
  printf("  -h, --help                  Show this help message and exit\n");
}

bool parseArgs(int argc, char **argv, std::string &config_string, bool &safety) {
  safety = true;
  config_string.clear();

  int option;
  struct option long_options[] = {
      {"config", required_argument, 0, 'c'},
      {"help", no_argument, 0, 'h'},
      {0, 0, 0, 0}};

  // Reset getopt for potential re-use
  optind = 1;

  while ((option = getopt_long(argc, argv, "nc:h", long_options, nullptr)) != -1) {
    switch (option) {
    case 'c':
      config_string += optarg;
      config_string += "\n";
      break;
    case 'n':
      safety = false;
      break;
    case 'h':
      print_usage(argv[0]);
      return false;
    case '?':
      print_usage(argv[0]);
      return false;
    default:
      break;
    }
  }
  return true;
}

void loadConfig(ModuleManager &mm, const std::string &config_string) {
  if (config_string.length() > 0) {
    printf("Custom configuration string:\n%s", config_string.c_str());
  }

  mm.clearConfig();
  bool res;
  res = mm.appendConfigFile("list.toml");
  res = res && mm.appendConfigFile("versionlist.toml");
  res = res && mm.appendConfigFile("robotlist.toml");
  if (!mm.appendConfigFile("localconf.toml")) {
    mm.warning("main", "Could not find localconf.toml, continuing.");
  }
  res = res && mm.appendConfigString(config_string.c_str());
  if (!res)
    mm.fatalError("main", "Could not find one or more configuration files!");
  if (!mm.finalizeConfig())
    mm.fatalError("main", "Error reading configuration files!");
}

void installSignalHandlers(ModuleManager *mm) {
  _signal_mgr = mm;
  signal(SIGINT, exit_on_ctrl_c);
  signal(SIGTERM, exit_on_ctrl_c);
}
