# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Behavioral control software for quadruped robots (METU ATLAS Lab), built on the
external **rtrobot** real-time framework (`ModuleManager`, `Module`, `LogServer`,
`ThreadedLoop`, TOML config). rtrobot supplies the scheduler and hardware base
classes; this repo supplies drivers, kinematics, behaviors and a supervisor.

`RTROBOT_DIR` defaults to `$HOME/rtrobot`. This checkout is configured against a
sibling `../rtrobot` (see `RTROBOT_DIR` in `build/CMakeCache.txt`). Its headers
live in `$RTROBOT_DIR/include/rtcore` and `.../rtclient` — read them when a
`_mgr->` call is unclear.

## Build

Exactly one hardware target is compiled at a time; `HARDWARE_TARGET` selects
which `src/hardware/*` subdirectory is added and which binaries exist.

```
./build.sh simulation   # -> bin/simulation, bin/sim.sh   (MuJoCo, fetched by cmake)
./build.sh go1          # -> bin/go1, bin/go1test + scripts (vendored Unitree SDK)
./build.sh robot        # -> bin/robot, bin/robot.sh      (CAN)
./build.sh clean        # rm -rf build/
```

`build.sh` configures, builds, then wipes and repopulates `bin/` and `lib/`, and
refreshes the `compile_commands.json` symlink. Manual cmake when you need flags:

```
cmake -B build -DHARDWARE_TARGET=simulation -DRTROBOT_DIR=/path/to/rtrobot \
      -DCMAKE_BUILD_TYPE=Debug -DDEBUG_ASAN=ON
cmake --build build && cmake --install build
```

The default build type is Release, so `assert()` is compiled out — tests use a
`T_CHECK` macro that prints and `exit(1)`s instead. Follow that pattern.

## Test

Tests build with the normal build and run under ctest from `build/`:

```
cd build && ctest --output-on-failure
ctest -R test_trot_gait --output-on-failure   # single test
./src/quadruped/tests/test_trot_gait          # run the binary directly
```

There is no test framework — each test is a plain executable that returns
non-zero on failure. Which tests exist depends on the configured target:
`src/hardware/mujocohw/tests` only builds under `simulation`,
`src/hardware/robothw/tests` only under `robot`; `src/quadruped/tests` always
builds. `test_config_layering` takes `<config-root> <robotv1|sim|go1>` and is
registered three times, once per chain, because the config search path is
process-global.

## Run

Binaries must be launched via the wrapper scripts **from inside `bin/`** — they
export `CONFIG_DIR`/`VERSION_DIR`/`ROBOT_DIR`, and `simulation.model_path` is
relative (`../models/...`). Running the raw binary elsewhere aborts on missing
config. All binaries accept `-c "<toml>"` to append config overrides:

```
cd bin && ./sim.sh -c "simulation.headless = true" -c "supervisor.log.enable = true"
```

`Supervisor` reads keys from stdin: **S** stand, **W** draw square, **T** trot,
**D** stop/sit, **Q** quit. `go1test` (Go1 only) is a separate interactive
hardware-validation binary driven by `MdlHWTest`.

## Architecture

### Module scheduling

`ModuleManager` runs registered modules each cycle in `order` sequence.
`Module` subclasses implement `init/uninit/activate/deactivate/update`.
Ordering bands come from rtrobot (`LOGGING_MODULES=0`, `SENSING_MODULES=10`,
`USER_CONTROLLERS=10000`, `BEHAVIORAL_CONTROLLERS=20000`, `ACTUATOR_MODULES=30000`).

**A module's period/offset/order is hardcoded in `include/quadruped/ModuleConfig.hh`,
not in TOML.** `CREATE_MODULE` (`ModuleDefs.hh`) fatal-errors if a module name is
missing from that table, so any new module needs an entry there first.

### Ownership

Prefer `grabModule/releaseModule` over `activateModule/deactivateModule`. Most
modules are `SINGLE_USER`, which enforces the chain: `Supervisor` grabs one
behavior at a time; a behavior (`MdlStand`, `MdlTrot`, `MdlDrawSquare`, `MdlSit`)
grabs all four `MdlLegControl` instances; `MdlLegControl` grabs its 3 motor axes.
`MdlStateEstimator` is the exception — it is activated in `ActivateCoreModules`
and always runs, as a sensor for everything else.

Wiring for the whole module set lives in `src/quadruped/CoreModules.cc`
(Add/Activate/Deactivate/Remove); `Supervisor::update()` is the behavior state
machine on top of it.

### Hardware abstraction

`MotorHW`, `IMUHW`, `ClockHW` are rtrobot `Hardware<T>` singletons. Each target's
`<Target>HW.cc` (`SimHW.cc`, `Go1HW.cc`, `RobotHW.cc`) declares `HARDWARE_IMPL(...)`,
defines `initHardware()`/`cleanupHardware()`, and registers the concrete instances.
`src/supervisor/main.cc` is target-agnostic and links whichever hw library the
target selected — keep target-specific code out of `src/quadruped/`.

`MotorHW` is MIT-mode: each command carries `pos, vel, tau, kp, kd`.
Motor index = `3 * leg + joint`. Leg order is FL, FR, RL, RR (`QuadrupedKinematics::LegIndex`);
joints are abduction, flexion, knee. Body frame is +X forward, +Y left, +Z up.
Kinematic parameters are chosen at runtime from the `hardware.library` config
string (`go1hw` → `createGo1Config()`, otherwise `createGo2Config()`), not from
the build target.

### Configuration layering

Three tiers searched in order, **first match wins**:
`config/default/` (`CONFIG_DIR`) → `config/versions/<version>/` (`VERSION_DIR`)
→ `config/robots/<robot>/` (`ROBOT_DIR`). A file in `default/` therefore *shadows*
a same-named file in a version or robot directory — this has silently broken
`threads.toml` before, which is what `test_config_layering` guards.

`app_common::loadConfig` appends, in order: `list.toml`, `versionlist.toml`,
`robotlist.toml`, optional `localconf.toml`, then the `-c` string (last wins).
`list.toml` `%include`s the per-behavior files; `versionlist.toml`/`robotlist.toml`
include version- and robot-local ones. Adding a behavior config means creating
`config/default/<name>.toml` **and** adding a `%include` to `list.toml`.

Modules read config in `init()` via `_mgr->getConfigTable("<section>", t)` and
`t.getDouble(key, default)`; every key should have a sensible default in code.

### Logging

Modules register variables with `LogServer` in `init()`
(`_logserver->registerVar(...)`, matched by `deleteVar` in `uninit()`), producing
names like `MdlStateEstimator_state`, `MdlSimDriver_qpos`. `Supervisor` runs a
`ThreadedLoop` that subscribes to the names listed in `supervisor.log.vars` and
writes ascii/raw/matlab output — mainly for offline comparison of estimator output
against simulator ground truth.

## Adding a behavior module

1. `include/quadruped/MdlFoo.hh` + `src/quadruped/MdlFoo.cc`, with a
   `#define FOOMODULE_NAME "MdlFoo"`.
2. Add the `.cc` to `CONTROLSRC` in `src/quadruped/CMakeLists.txt`.
3. Add a `{"MdlFoo", -1}` entry to `moduleConfig` in `ModuleConfig.hh`.
4. `CREATE_MODULE`/`DESTROY_MODULE` in `src/quadruped/CoreModules.cc`.
5. `config/default/foo.toml` + `%include foo.toml` in `config/default/list.toml`.
6. Grab/release it from a `Supervisor` state.

A test linking a `Mdl*` object file must also link `rtcore rtclient`, since the
object carries the `Module` base class along with the math under test.

## Conventions

Files are prefixed with the RoboMETU copyright header. Two-space indent, private
members `_underscored`, `Mdl*` prefix for modules, `.hh`/`.cc` extensions, Eigen
for all vector math. Comments in this codebase explain *why* a value or ordering
was chosen (see `config/default/trot.toml` for the register) — match that when
touching tuned parameters.

## Implementation practice mode

Trigger: when I say "scaffold only" or "leave a chunk for me."

Instead of a full working solution:
1. Write everything around the core: signatures, imports, docstrings, I/O,
   test harness / plotting scaffolding, error handling.
2. Pick one meaningful chunk — a function, a computation block, a control
   step — and leave it as `# TODO(ege): implement`, with a stub return of
   the correct shape/type so the rest of the file still runs.
3. Above the stub, sketch the logical steps involved: 2-4 short bullets on
   what has to happen, in what order, and any gotchas/edge cases worth
   knowing — enough to orient me, not a line-by-line recipe. No pseudocode
   that's really just code with different syntax.
4. Stop there. Don't hide a reference implementation in a comment,
   docstring, or fallback branch.
5. When I bring back an attempt, review it like a real code review —
   correctness, edge cases, style — but don't rewrite it unless I
   explicitly ask.

Default (no trigger phrase): implement fully, as normal.
