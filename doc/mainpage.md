# quadcontrol: Behavioral control software for quadruped robots

## 1. Overview

quadcontrol is the control software for quadruped robots developed at
METU ATLAS Lab. It builds on top of the **rtrobot** real-time control
framework, which provides module scheduling, threading, hardware
abstraction and TOML-based configuration.

quadcontrol adds the robot-specific pieces: hardware drivers, kinematics,
behavioral controllers and a supervisor to manage them at runtime.

## 2. Architecture

The software has three layers:

```
┌─────────────────────────────────────────────────┐
│  Behavioral Controllers                         │
│  (Supervisor, MdlLegControl, MdlDrawSquare,     │
│   MdlStand, MdlSit, MdlHWTest)                  │
├─────────────────────────────────────────────────┤
│  Quadruped Library                              │
│  (QuadrupedKinematics, CoreModules)             │
├─────────────────────────────────────────────────┤
│  Hardware Drivers                               │
│  (MotorHW, IMUHW, ClockHW singletons)          │
├──────────┬──────────────┬───────────────────────┤
│ mujocohw │   go1hw      │      robothw          │
│ (MuJoCo) │ (Unitree SDK)│   (CAN bus)           │
└──────────┴──────────────┴───────────────────────┘
         rtrobot (ModuleManager, ThreadedLoop, ...)
```

**Key modules:**

| Module | Description |
|--------|-------------|
| MdlSimDriver | Interfaces with MuJoCo simulation for motor commands and state readback |
| MdlGo1 | Communicates with the Unitree Go1 over UDP via the Legged SDK |
| QuadrupedKinematics | Forward/inverse kinematics and Jacobian for configurable quadruped geometries |
| MdlLegControl | Joint-level or task-level control of individual legs |
| MdlDrawSquare | Example behavioral controller that coordinates all four legs |
| MdlStand | Stand-up motion relative to the pose at activation |
| MdlSit | Sit-down motion to an absolute pose from config |
| MdlHWTest | Interactive hardware test with incremental motor validation |
| Supervisor | Runtime module manager that switches between behavioral controllers |

## 3. Prerequisites

- **rtrobot** — The real-time control framework. By default expected at
  `$HOME/rtrobot`. Set `RTROBOT_DIR` to override.
- **Eigen3** — Required for kinematics computations.
- **MuJoCo** — Fetched automatically by cmake when building the `simulation` target.
- **Unitree Legged SDK** — Vendored in `src/hardware/go1hw/unitree_legged_sdk/`.
  No extra installation needed for the `go1` target.

## 4. Building

Only one hardware target can be built at a time: `simulation`, `go1` or `robot`.

### Using build.sh (recommended)

```
./build.sh go1          # Build Go1 binaries (go1, go1test)
./build.sh simulation   # Build MuJoCo simulation binary
./build.sh robot        # Build CAN robot binary
./build.sh clean        # Remove the build directory
```

After a successful build, binaries and launch scripts are installed
to the `bin/` directory.

### Manual cmake

```
cd ~/quadcontrol
mkdir -p build && cd build
cmake .. -DHARDWARE_TARGET=go1
cmake --build .
cmake --install .
```

Replace `go1` with `simulation` or `robot` as needed. If `RTROBOT_DIR`
is not at the default location, pass it explicitly:

```
cmake .. -DHARDWARE_TARGET=go1 -DRTROBOT_DIR=/path/to/rtrobot
```

### Build outputs

| Target | Binaries | Launch scripts |
|--------|----------|----------------|
| `simulation` | `simulation` | `sim.sh` |
| `go1` | `go1`, `go1test` | `go1.sh`, `go1test.sh` |
| `robot` | `robot` | `robot.sh` |

## 5. Configuration

Configuration is managed through TOML files organized in three tiers
under `config/`:

```
config/
├── default/              # Shared across all targets
├── versions/
│   ├── sim/              # Simulation-specific settings
│   ├── go1/              # Go1-specific settings
│   └── robotv1/          # CAN robot-specific settings
└── robots/
    ├── sim/              # Simulation instance data
    ├── go1r1/            # Go1 instance (calibration, motor IDs, network)
    └── robotv1r1/        # Robot instance (calibration)
```

Each launch script sets three environment variables that point to the
appropriate directories:

- **CONFIG_DIR** — Always `config/default/`. Contains settings shared
  by all targets. Nothing here may share a filename with version or
  robot files — the config search checks this directory first.
- **VERSION_DIR** — Points to a subdirectory of `config/versions/`
  (e.g. `go1`, `sim`). Contains settings tied to a robot type:
  hardware parameters, gains, kinematic model, and thread priorities
  (`threads.toml`, included from `versionlist.toml`).
- **ROBOT_DIR** — Points to a subdirectory of `config/robots/`
  (e.g. `go1r1`, `sim`). Contains instance-specific data like
  calibration offsets and motor mappings.

If these variables are not set, the application will exit with errors
about missing configuration files.

### Go1 configuration

Network and motor settings are in `config/robots/go1r1/go1.toml`:

```toml
[go1]
motor_ids = [ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ]
remote_ip = "192.168.123.10"
remote_port = 8007
local_port = 8090
```

Hardware test parameters can be added in a `[hwtest]` section:

```toml
[hwtest]
joint = 0                   # Motor index for SINGLE_JOINT state
leg = "FR"                  # Leg for SINGLE_LEG (FR, FL, RR, RL)
amplitude = 0.2             # Sine amplitude (rad)
frequency = 0.2             # Sine frequency (Hz)
kp = 5.0                    # Position gain
kd = 1.0                    # Damping gain
tracking_error_limit = 0.5  # Safety cutoff (rad)
```

## 6. Running

### Launch scripts

From the install directory:

```
cd ~/quadcontrol/bin
./sim.sh                # MuJoCo simulation
./go1.sh                # Go1 robot (Supervisor)
./go1test.sh            # Go1 hardware test
./robot.sh              # CAN robot
```

### Command-line options

All executables accept the following flags:

```
-c, --config STRING     Append a TOML configuration string
-h, --help              Show help
```

Configuration overrides are useful for quick parameter changes without
editing files. Examples:

```bash
# Disable gravity in simulation
./sim.sh -c "simulation.gravity = [0, 0, 0]"

# Run headless
./sim.sh -c "simulation.headless = true"

# Multiple overrides
./sim.sh -c "simulation.framerate=60" -c "simulation.gravity=[0,0,-9.81]"

# Include an extra config file
./sim.sh -c "%include configoverride.toml"
```

### Go1 hardware test (go1test)

`go1test` is an interactive tool for validating Go1 hardware. It walks
through five states, each adding more motor activity:

| State | What it does |
|-------|-------------|
| **READBACK** | Read-only. Displays motor positions, velocities, torques, temperatures and IMU data. |
| **HOLD** | Activates all 12 motors, captures current positions and holds them. Ramps to `home_position` if one is configured. |
| **SINGLE_JOINT** | Sine wave on one motor; all other motors keep holding. |
| **SINGLE_LEG** | Sine on one leg's thigh joint; everything else keeps holding. |
| **ALL_LEGS** | All four thighs follow a sine wave; hips and calves keep holding. |

**Controls:** Press **N** to advance to the next state, **B** to go back
to READBACK, **S** to snapshot the current pose as `home_position` into
the version's `gains.toml`, **Q** to quit.

A tracking error check runs continuously during motor states. If any
motor deviates from its command by more than `tracking_error_limit`,
the system automatically falls back to READBACK.

## 7. Debugging

### Debug builds

To compile with debug symbols and no optimization:

```
cd ~/quadcontrol/build
cmake -DHARDWARE_TARGET=go1 -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
cmake --install .
```

This enables full gdb/lldb debugging with source-level stepping.

### Address Sanitizer (ASAN)

ASAN is a runtime memory error detector. It catches:

- **Buffer overflows** — reading or writing past the end of arrays
  and heap allocations
- **Use-after-free** — accessing memory that has already been deallocated
- **Double-free** — freeing the same memory twice
- **Memory leaks** — allocations that are never freed

To enable ASAN:

```
cd ~/quadcontrol/build
cmake -DHARDWARE_TARGET=go1 -DDEBUG_ASAN=ON ..
cmake --build .
cmake --install .
```

ASAN adds runtime overhead (~2x slower), so it is not suitable for
real-time operation on hardware. Use it during development and testing.

When the program exits, ASAN prints a report of any issues it found
to stderr, including stack traces pointing to the source of each error.
If no issues are detected, no output is produced.

See https://github.com/google/sanitizers/wiki/addresssanitizer for
detailed documentation.
