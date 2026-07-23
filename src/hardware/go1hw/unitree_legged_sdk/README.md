# unitree_legged_sdk (vendored, trimmed)

Upstream: https://github.com/unitreerobotics/unitree_legged_sdk, release v3.8.6 (Go1 only).

Trimmed to what the go1hw build actually uses:
- `include/` — SDK headers
- `lib/cpp/{amd64,arm64}/libunitree_legged_sdk.a` — prebuilt static libs, one linked per arch
- `LICENSE`

Removed: examples, python wrapper (with its vendored pybind11), prebuilt python `.so`s, and the
SDK's own CMake/catkin files. Nothing here is built from source — `go1hw/CMakeLists.txt` imports
the headers and one `.a` directly. To restore anything, pull the v3.8.6 tag from upstream.
