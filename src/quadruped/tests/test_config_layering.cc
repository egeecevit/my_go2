#include <cstdio>
#include <cstdlib>

#include "rtcore/ModuleManager.hh"

// No hardware library in this test; provide the singleton statics a
// <Target>HW.cc would normally instantiate.
using namespace rtcore;
HARDWARE_IMPL(ClockHW);

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                  \
  do {                                                                 \
    if (!(cond)) {                                                     \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
      return 1;                                                        \
    }                                                                  \
  } while (0)

// Loads the real shipped config tree the way app_common does and checks
// that version-specific values actually reach the merged table. Guards
// against the first-match-wins shadowing where a same-named file in
// config/default silently hid config/versions/<v>/threads.toml.
//
// One chain per invocation (the config search path is process-global):
//   test_config_layering <config-root> <robotv1|sim|go1>
int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s <path-to-config-root> <robotv1|sim|go1>\n",
            argv[0]);
    return 1;
  }
  std::string cfg = argv[1];
  std::string chain = argv[2];

  std::string version, robot;
  if (chain == "robotv1") { version = "robotv1"; robot = "robotv1r1"; }
  else if (chain == "sim") { version = "sim"; robot = "sim"; }
  else if (chain == "go1") { version = "go1"; robot = "go1r1"; }
  else { fprintf(stderr, "unknown chain '%s'\n", chain.c_str()); return 1; }

  setenv("CONFIG_DIR", (cfg + "/default").c_str(), 1);
  setenv("VERSION_DIR", (cfg + "/versions/" + version).c_str(), 1);
  setenv("ROBOT_DIR", (cfg + "/robots/" + robot).c_str(), 1);

  ModuleManager mm;
  mm.clearConfig();
  T_CHECK(mm.appendConfigFile("list.toml"));
  T_CHECK(mm.appendConfigFile("versionlist.toml"));
  T_CHECK(mm.appendConfigFile("robotlist.toml"));
  T_CHECK(mm.finalizeConfig());

  // [mpc] has to reach every chain. MdlConvexMPC treats a missing or malformed
  // entry as a fatal initialization error rather than defaulting, on the
  // grounds that a wrong mass or inertia does not fail loudly -- it just
  // produces confidently wrong forces -- so a chain that drops the table
  // cannot start the supervisor at all. Checked here because it arrives
  // through list.toml and is therefore exactly the kind of thing a same-named
  // file in a version or robot directory could shadow away.
  ConfigTable mpc;
  T_CHECK(mm.getConfigTable("mpc", mpc));
  T_CHECK(mpc.getDouble("dt", -1.0) > 0.0);
  T_CHECK(mpc.getDouble("mass", -1.0) > 0.0);
  T_CHECK(mpc.getDouble("gravity", -1.0) > 0.0);
  T_CHECK(mpc.getDouble("friction", -1.0) > 0.0);
  T_CHECK(mpc.getDouble("force_weight", -1.0) > 0.0);
  T_CHECK(mpc.getDouble("force_max", -1.0) > mpc.getDouble("force_min", -1.0));

  ConfigArray com;
  T_CHECK(mpc.getArray("com_offset_body", com));
  T_CHECK(com.size() == 3);

  ConfigArray inertia;
  T_CHECK(mpc.getArray("inertia_body", inertia));
  T_CHECK(inertia.size() == 9);

  ConfigArray weights;
  T_CHECK(mpc.getArray("state_weights", weights));
  T_CHECK(weights.size() == 12);

  ConfigTable threads;
  bool hasThreads = mm.getConfigTable("threads", threads);

  if (chain == "robotv1") {
    // These live only in versions/robotv1/threads.toml — if the shadowing
    // ever returns, they vanish and the fallback priorities run instead.
    T_CHECK(hasThreads);
    T_CHECK(threads.getInt("main_priority", -1) == 99);
    T_CHECK(threads.getInt("can_priority", -1) == 95);
    T_CHECK(threads.getInt("vn100_priority", -1) == 91);
    ConfigArray cpuset;
    T_CHECK(threads.getArray("main_cpuset", cpuset));
    T_CHECK(cpuset.size() == 2);
  } else if (chain == "sim") {
    // sim ships an empty [threads] table; the chain must still load
    T_CHECK(hasThreads);
  } else {
    // go1 has no threads.toml anywhere; the chain must load without it
    T_CHECK(!hasThreads);
  }

  printf("test_config_layering(%s): all checks passed\n", chain.c_str());
  return 0;
}
