# quadcontrol: Behavioral control software for quadruped robots

## Overview

This repository contains software for controlling dynamic behaviors
for quadruped robots. It uses the rtrobot real-time control framework
and provides the following components:

- Hardware layer implementation for  "physical" and "simulated" quadruped robot instances
- MdlSimDriver module interfacing with a MuJoCo simulation environment to send
  motor commands and retrieve system states
- The QuadrupedKinematics utility  class to implement forward and inverse
  kinematics and Jacobian computations for a sommon class of quadruped robots.
- An example The MdlLegControl module for joint-level or task-level
  control of individual legs
- An example MdlDrawSquare module as a simple example of a behavioral controller
  module, coordinating the motion of all four legs

## Compilation

This repository uses cmake for its build process. As such, compilation
is best done in a separate build directory with:

```
export RTROBOT_DIR=$HOME/rtrobot
cd ~/quadcontrol
mkdir build
cd build
cmake ..
cmake --install .
```

Note that you may need to change the `RTROBOT_DIR` definition to where
you have compiled your rtrobot libraries.  Upon successful
compilation, the `lib` directory should contain the `quadruped`
library, whereas the `bin` directory should contain two executables,
`robot` and `simulation`, containing supervisory code to be executed
on the physical and the simulated robot instances,
respectively. Following compilation, the resulting executables can be
run with

```
cd ~/quadcontrol/bin
./sim.sh
./robot.sh
```

The scripts `sim.sh` and `robot.sh` set three environment variables,
`CONFIG_DIR`, `VERSION_DIR` and `ROBOT_DIR`, which are used by the
example code to configure search paths for the extended TOML
configuration files. through the ConfigInput::addDirectoryLast()
method:

- `CONFIG_DIR` is always set to `~/rtrobot/config/default` and is
  expected to contain configuration files common to all robot version
  and instances.
- `VERSION_DIR` is always set to a subdirectory of
  `~/rtrobot/versions` (e.g `robotv1` vs `sim`) and is expected to
  contain configuration files differentiated by the "version" of the
  robot corresponding to a particular morphology, weight distribution,
  available components etc.
- `ROBOT_DIR` is always set to a subdirectory of `~/rtrobot/robots`
  (e.g `robotv1r1` vs `simr1`) and is expected to contain
  configuration files that are specific to a particular instance of a
  particular robot version. This is usually where calibration data is
  placed.
  
If these environment variables are not set, the example executables
will exit with errors indicating that various configuration files
cannot be opened.

Both executables, `simulation` and `robot`, support the following options:
```
Usage: ./simulation [OPTIONS]
Options:
  -c, --config TOML_STRING  Specify configuration string to be appended
  -h, --help                Show this help message and exit
```

In particular, these example executable allow the user to supply additional 
configuration TOML strings to override configuration entries in the default 
files described above. For example, it is possible to override the gravity setting for the simulation with:

```
./sim.sh -c "simulation.gravity = [0, 0, 0]"
```

or, run the simulation in headless mode with

```
./sim.sh -c "simulation.headless = true"
```

You can also include additional configuration files with

```
./sim.sh -c "%include configoverride.toml"
```

Finally, if you want to include multiple lines of TOML configuration within a single option, you can use newline characters `\n` within the `$'...'` format as

```
./sim.sh -c $'simulation.framerate=60\nsimulation.gravity=[0,0,-9.81]'
```

as well as equivalently using multiple configuration statements as

```
./sim.sh -c "simulation.framerate=60" -c "simulation.gravity=[0,0,-9.81]"
```

## Debugging

Debugging builds can be enabled in the usual cmake way with

```
cd ~/quadcontrol/build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j8 install
```

which will turn off optimizations and result in debugging symbols
being included in the final executable. We also support compilation
with the Address Sanitizer (ASAN) with the following option:

```
cd ~/quadcontrol/build
cmake -DDEBUG_ASAN=ON ..
make -j8 install
```

which will generate an libraries and executables with address
sanitization enabled. When the program exits, issues identified by the
address sanitizer will be printed out.  See
https://github.com/google/sanitizers/wiki/addresssanitizer for more
information.
