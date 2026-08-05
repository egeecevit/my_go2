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

Usage: $(basename "$0") [options] [logfile]

Writes one figure per quantity, each with x, y and z subplots, ground truth in
solid blue and the estimate in solid red:

  body_position.png  body_velocity.png  body_orientation.png
  foot_position_FL.png  foot_position_FR.png  foot_position_RL.png
  foot_position_RR.png

Arguments:
  logfile             Supervisor .mat log to read
                      (default: <repo>/bin/estrun.mat)

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
  $(basename "$0")                          # plot bin/estrun.mat
  $(basename "$0") --start 9                # skip the stand-up transient
  $(basename "$0") -s 20 -e 21.5 -w         # zoom on one stride, and display it
  $(basename "$0") -a -o /tmp/aligned       # without the heading drift
  $(basename "$0") -o /tmp/figs run2.mat    # a different log, elsewhere

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

LOGFILE=""
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
    --)          shift; break ;;
    -*)          echo "Unknown option: $1" >&2; echo >&2
                 usage >&2; exit 2 ;;
    *)           [[ -z "$LOGFILE" ]] || { echo "Only one logfile may be given" >&2; exit 2; }
                 LOGFILE="$1"; shift ;;
  esac
done

# Anything after -- is a logfile too.
if [[ $# -gt 0 ]]; then
  [[ -z "$LOGFILE" ]] || { echo "Only one logfile may be given" >&2; exit 2; }
  LOGFILE="$1"
fi

[[ -x "$PY" ]] || setup_venv

# The venv's interpreter directly rather than `source .venv/bin/activate`: the
# activation would be undone the moment this subshell exits, so it buys nothing,
# and its unset PS1 reference trips over `set -u`.
exec "$PY" "$PLOTTER" ${LOGFILE:+"$LOGFILE"} "${ARGS[@]+"${ARGS[@]}"}"
