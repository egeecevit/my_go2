#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

usage() {
  echo "Usage: $0 <command>"
  echo ""
  echo "Commands:"
  echo "  simulation   Build MuJoCo simulation binary"
  echo "  go1          Build Go1 binaries (go1, go1test)"
  echo "  robot        Build CAN robot binary"
  echo "  clean        Remove build directory"
  exit 1
}

if [ $# -eq 0 ]; then
  usage
fi

case "$1" in
  clean)
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    echo "Done."
    ;;
  simulation|go1|robot)
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    echo "Configuring for target: $1"
    cmake .. -DHARDWARE_TARGET="$1" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ln -sf build/compile_commands.json "$SCRIPT_DIR/compile_commands.json"
    echo "Building..."
    cmake --build .
    echo "Installing..."
    rm -rf "$SCRIPT_DIR/bin" "$SCRIPT_DIR/lib"
    cmake --install .
    echo ""
    echo "Done. Binaries installed to $SCRIPT_DIR/bin/"
    ;;
  *)
    echo "Unknown command: $1"
    usage
    ;;
esac
