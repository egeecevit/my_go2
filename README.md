# quadcontrol: Control of dynamic quadruped behaviors

This repository contains software for behavioral control of dynamic
quadruped robots. It builds on the
[rtrobot](https://github.com/metuatlas/rtrobot) real-time control
framework and supports three hardware targets:

- **simulation** — MuJoCo physics simulation
- **go1** — Unitree Go1 (UDP via Unitree Legged SDK)
- **robot** — CAN-based physical robot

## Quick start

```
./build.sh go1          # Build Go1 binaries
./build.sh simulation   # Build MuJoCo simulation
./build.sh clean        # Remove build directory
```

See [doc/mainpage.md](doc/mainpage.md) for architecture, configuration
and usage details.
