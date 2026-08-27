#! /bin/bash
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.
#
# Wrapper around plot_estimation.py that runs it out of the local virtualenv,
# creating that virtualenv on first use. Runs from any directory.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$SCRIPT_DIR/.venv"
PY="$VENV/bin/python"
PLOTTER="$SCRIPT_DIR/plot_estimation.py"
# Kept in step with the imports at the top of plot_estimation.py.
DEPS=(numpy scipy matplotlib)

usage() {
  cat <<EOF
Plot the state estimate against simulator ground truth.

Usage: $(basename "$0") [options] [logfile1 [logfile2]]

Writes one figure per quantity for each log, each with x, y and z subplots.
With one log, the original filenames are used:

  body_position.png  body_velocity.png  body_orientation.png
  body_angular_velocity.png
  foot_position_FL.png  foot_position_FR.png  foot_position_RL.png
  foot_position_RR.png

The last two need MdlOrientationEstimator_filter, which logs recorded before it
was added do not carry; they are skipped with a note in that case:

  gyro_bias.png  filtered_acceleration.png

With two logs, each filename gains a <LOG> suffix, using the corresponding log
filename without its extension, for example body_position_estrun.png.

Arguments:
  logfile1            Supervisor .mat log to read
                      (default: <repo>/bin/estrun.mat)
  logfile2            Optional second Supervisor .mat log

Options:
  -o, --outdir DIR    Write the figures here (default: $SCRIPT_DIR)
  -s, --start SEC     Drop samples before SEC. The log begins several seconds
                      before the robot stands; --start 9 typically cuts
                      everything up to the trot
  -e, --stop SEC      Drop samples after SEC
  -w, --show          Open the figures in a window as well as writing them
  -a, --heading-aligned
                      Subtract the drifted heading's contribution from the
                      position figures. The yaw gyroscope bias is unobservable,
                      so heading drifts; the filter then renders forward motion
                      as partly lateral, which integrates into a y ramp and
                      splays the front and rear feet to opposite sides. This
                      view leaves what a change to the position filter would
                      actually move
      --setup         Create or refresh the virtualenv, then exit
  -h, --help          Show this message

Windowing is applied before anything is computed, so the per-axis RMS annotation
on each subplot describes exactly the interval plotted.

Recording a log (logging is off by default, and the run must exit cleanly via Q):

  cd bin
  ./sim.sh -c 'supervisor.log.enable = true' \\
           -c 'supervisor.log.file_name = "estrun.mat"'

Examples:
  $(basename "$0")
  $(basename "$0") run1.mat
  $(basename "$0") bin/estrun.mat bin/lpf_estrun.mat
  $(basename "$0") --start 9 run1.mat run2.mat
  $(basename "$0") -s 20 -e 21.5 -w run1.mat run2.mat
  $(basename "$0") -a -o /tmp/aligned run1.mat run2.mat

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
    -a|--heading-aligned)
                 ARGS+=(--heading-aligned); shift ;;
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
