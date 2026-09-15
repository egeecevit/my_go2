#! /bin/bash
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.
#
# Wrapper around plot_log.py that runs it out of the local virtualenv,
# creating that virtualenv on first use. Runs from any directory.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$SCRIPT_DIR/.venv"
PY="$VENV/bin/python"
PLOTTER="$SCRIPT_DIR/plot_log.py"
# Kept in step with the imports at the top of plot_log.py.
DEPS=(numpy scipy matplotlib)

usage() {
  cat <<EOF
Plot a Supervisor .mat log -- whatever it happens to contain.

Usage: $(basename "$0") [options] [logfile1 [logfile2]]

Reads the variables the log actually carries. Variables that hold versions of
the same physical quantity are drawn on one figure, overlaid, with the error
between them on a right-hand axis so its evolution is readable against the
comparison rather than only as a number in the corner.

Which figures get drawn is chosen by --profile. Without one, the profile the
log best supports is picked and named.

  estimator    state estimator against simulator ground truth
                 body_position  body_velocity  body_orientation
                 orientation_quaternion  body_angular_velocity  gyro_bias
                 gravity_reference  attitude_filter  foot_position_<LEG>
                 estimator_cov_{position,velocity,foot}

  controller   MdlTrot / MPC tracking, effort and force
                 torque_<LEG>  foot_reference_<LEG>  grf_<LEG>  grf_total
                 body_velocity  body_yaw  body_yaw_rate  contact

  plant        raw joint state, no comparison
                 joint_angle_<LEG>  joint_velocity_<LEG>

  all          every column of every variable reaches a figure, with a
               coverage report naming each one. Anything the curated profiles
               leave out is written as generic_<variable>_<block>.png

A figure whose quantity resolves only one role is not a comparison and is
dropped, unless its profile says it stands alone (contact, joint state, the
attitude filter, and torque without a logged request).

With two logs, each filename gains a <LOG> suffix taken from the log filename
without its extension, for example body_position_estrun.png; --overlay draws
both logs on one set of axes instead.

Arguments:
  logfile1            Supervisor .mat log to read (default: <repo>/bin/deneme.mat)
  logfile2            Optional second Supervisor .mat log

Options:
  -p, --profile NAME  estimator, controller, plant or all
                      (default: whichever the log best supports)
  -o, --outdir DIR    Write the figures here (default: figs/out/<log>)
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
      --overlay       With two logs, superimpose them on one set of axes
      --only PATTERNS Comma-separated globs; plot only matching figures
      --skip PATTERNS Comma-separated globs; skip matching figures
      --no-generic    Only the comparison figures
      --generic-only  One figure per block of every variable, no comparisons
      --list          Report what the log holds and what would be written,
                      then exit without drawing anything
      --dpi N         Figure resolution in dots per inch (default: 200)
      --ext FMT       png, pdf or svg (default: png)
      --decimate N    Plot every Nth sample (default: auto, about 20k points)
      --setup         Create or refresh the virtualenv, then exit
  -h, --help          Show this message

Windowing is applied before anything is computed, so the RMS and saturation
annotations on each subplot describe exactly the interval plotted.

Recording a log (logging is off by default, and the run must exit cleanly via Q):

  cd bin
  ./sim.sh -c 'supervisor.log.enable = true' \\
           -c 'supervisor.log.file_name = "estrun.mat"'

Which variables get recorded is supervisor.log.vars in
config/default/supervisor.toml; this script plots whichever of them are there.

Examples:
  $(basename "$0")
  $(basename "$0") --profile estimator bin/deneme.mat
  $(basename "$0") --profile controller runs/base2.mat
  $(basename "$0") --list runs/base2.mat
  $(basename "$0") --start 9 bin/estrun.mat
  $(basename "$0") --only 'torque_*,grf_*' runs/base2.mat
  $(basename "$0") --overlay runs/base2.mat runs/lowdamp.mat
  $(basename "$0") -s 20 -e 21.5 -w run1.mat run2.mat
  $(basename "$0") -a -o /tmp/aligned run1.mat run2.mat

See README.md in this directory for the variable reference and for how to read
the figures.
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
    -p|--profile)
                 [[ $# -ge 2 ]] || { echo "$1 needs a profile name" >&2; exit 2; }
                 ARGS+=(--profile "$2"); shift 2 ;;
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
    --overlay)   ARGS+=(--overlay); shift ;;
    --no-generic)   ARGS+=(--no-generic); shift ;;
    --generic-only) ARGS+=(--generic-only); shift ;;
    --list)      ARGS+=(--list); shift ;;
    --only)      [[ $# -ge 2 ]] || { echo "$1 needs a pattern list" >&2; exit 2; }
                 ARGS+=(--only "$2"); shift 2 ;;
    --skip)      [[ $# -ge 2 ]] || { echo "$1 needs a pattern list" >&2; exit 2; }
                 ARGS+=(--skip "$2"); shift 2 ;;
    --dpi)       [[ $# -ge 2 ]] || { echo "$1 needs a number" >&2; exit 2; }
                 ARGS+=(--dpi "$2"); shift 2 ;;
    --ext)       [[ $# -ge 2 ]] || { echo "$1 needs png, pdf or svg" >&2; exit 2; }
                 ARGS+=(--ext "$2"); shift 2 ;;
    --decimate)  [[ $# -ge 2 ]] || { echo "$1 needs a number" >&2; exit 2; }
                 ARGS+=(--decimate "$2"); shift 2 ;;
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
