#! /bin/bash
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.
#
# Wrapper around plot_mpc.py that runs it out of the local virtualenv,
# creating that virtualenv on first use. Runs from any directory.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$SCRIPT_DIR/.venv"
PY="$VENV/bin/python"
PLOTTER="$SCRIPT_DIR/plot_mpc.py"
# Kept in step with the imports at the top of plot_mpc.py.
DEPS=(numpy scipy matplotlib)

usage() {
  cat <<EOF
Plot MdlTrot / MPC tracking and torque saturation against a Supervisor log.

Usage: $(basename "$0") [options] [logfile1 [logfile2]]

Writes the following figures for each log. With one log, the original
filenames are used:

  torque_FL.png  torque_FR.png  torque_RL.png  torque_RR.png
  body_velocity_tracking.png  body_angular_velocity_tracking.png
  body_reference.png
  foot_reference_FL.png  foot_reference_FR.png
  foot_reference_RL.png  foot_reference_RR.png

Any figure whose log variables are missing is skipped with a printed note
instead of aborting the run, so an older log still plots what it can.

With two logs, each filename gains a <LOG> suffix, using the corresponding log
filename without its extension, for example torque_FL_mpcrun.png.

Arguments:
  logfile1            Supervisor .mat log to read
                      (default: <repo>/bin/mpcrun.mat)
  logfile2            Optional second Supervisor .mat log

Options:
  -o, --outdir DIR    Write the figures here (default: $SCRIPT_DIR)
  -s, --start SEC     Drop samples before SEC. The log begins several seconds
                      before the robot stands; --start 9 typically cuts
                      everything up to the trot
  -e, --stop SEC      Drop samples after SEC
  -w, --show          Open the figures in a window as well as writing them
      --dpi N         Figure resolution in dots per inch (default: 200)
      --setup         Create or refresh the virtualenv, then exit
  -h, --help          Show this message

Windowing is applied before anything is computed, so the RMS and saturation
annotations on each subplot describe exactly the interval plotted.

Recording a log (logging is off by default, and the run must exit cleanly via Q):

  cd bin
  ./sim.sh -c 'supervisor.log.enable = true' \\
           -c 'supervisor.log.file_name = "mpcrun.mat"' \\
           -c 'supervisor.log.vars = ["MdlSimDriver_qpos","MdlSimDriver_qvel","MdlSimDriver_ctrl","MdlTrot_torquereq","MdlTrot_footref","MdlPosVelEstimator_foottruth","MdlTrot_contact","MdlTrot_bodyref"]'

See README.md in this directory for the full variable list and its size limit.

Examples:
  $(basename "$0")
  $(basename "$0") run1.mat
  $(basename "$0") bin/mpcrun.mat bin/mpcrun2.mat
  $(basename "$0") --start 9 run1.mat run2.mat
  $(basename "$0") -s 20 -e 21.5 -w run1.mat run2.mat
  $(basename "$0") --dpi 300 -o /tmp/mpcfigs run1.mat

See README.md in this directory for how to read the figures.
EOF
}

setup_venv() {
  echo "Creating virtualenv in $VENV"
  python3 -m venv "$VENV"
  echo "Installing ${DEPS[*]}"
  "$VENV/bin/pip" install --quiet --upgrade pip
  "$VENV/bin/pip" install --quiet "${DEPS[@]}"
  echo "Done."
}

LOGFILES=()
ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)   usage; exit 0 ;;
    --setup)     setup_venv; exit 0 ;;
    -o|--outdir) [[ $# -ge 2 ]] || { echo "$1 needs a directory" >&2; exit 2; }
                 ARGS+=(--outdir "$2"); shift 2 ;;
    -s|--start)  [[ $# -ge 2 ]] || { echo "$1 needs a time in seconds" >&2; exit 2; }
                 ARGS+=(--start "$2"); shift 2 ;;
    -e|--stop|--end)
                 [[ $# -ge 2 ]] || { echo "$1 needs a time in seconds" >&2; exit 2; }
                 ARGS+=(--stop "$2"); shift 2 ;;
    -w|--show)   ARGS+=(--show); shift ;;
    --dpi)       [[ $# -ge 2 ]] || { echo "$1 needs a number" >&2; exit 2; }
                 ARGS+=(--dpi "$2"); shift 2 ;;
    --)          shift; LOGFILES+=("$@"); break ;;
    -*)          echo "Unknown option: $1" >&2; echo >&2
                 usage >&2; exit 2 ;;
    *)           LOGFILES+=("$1"); shift ;;
  esac
done

if [[ ${#LOGFILES[@]} -gt 2 ]]; then
  echo "At most two logfiles may be given" >&2
  echo >&2
  usage >&2
  exit 2
fi

[[ -x "$PY" ]] || setup_venv

# The venv's interpreter directly rather than `source .venv/bin/activate`: the
# activation would be undone the moment this subshell exits, so it buys nothing,
# and its unset PS1 reference trips over `set -u`.
exec "$PY" "$PLOTTER" "${ARGS[@]}" -- "${LOGFILES[@]}"
