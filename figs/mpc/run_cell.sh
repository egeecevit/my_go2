#! /bin/bash
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.
#
# One headless, deterministic trot run, written to runs/<name>.{mat,bin,log,manifest}.
#
# Runs are SERIALIZED by an flock: the simulator's LogServer binds a fixed ENet
# port (logserver.server_port, 5500), so two simulators cannot coexist. The lock
# makes that a property of the harness rather than something a caller has to
# remember.
#
# Usage:  figs/mpc/run_cell.sh <name> [extra -c overrides ...]
#   DUR=<seconds>   trot duration, default 40 (includes MdlTrot's WAIT+PREP)
#   OUT=<dir>       output directory, default <repo>/runs
#
# Example:
#   figs/mpc/run_cell.sh base_v095 -c 'trot.forward_velocity = 0.95'
#   DUR=90 figs/mpc/run_cell.sh h10 -c 'mpc.horizon = 10'

set -euo pipefail

[[ $# -ge 1 ]] || { echo "usage: $(basename "$0") <name> [-c 'key = value' ...]" >&2; exit 2; }

NAME="$1"; shift
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${OUT:-$REPO/runs}"
DUR="${DUR:-40}"
mkdir -p "$OUT"

# The full scorecard variable set: 119 doubles against the hard 128-double
# ceiling in rtclient::LogClient::addVar. Overflowing it, or naming a variable
# the binary does not define, produces NO FILE AT ALL while the run still exits
# 0 -- so this list is deliberately kept with headroom and changes to it must be
# re-counted.
#   qpos 19 + qvel 18 + ctrl 12 + grf 12 + forces 12 + solver 2
#   + contact 4 + bodyref 12 + torquereq 12 + torquescale 4 + foottruth 12 = 119
VARS='["MdlSimDriver_qpos","MdlSimDriver_qvel","MdlSimDriver_ctrl","MdlSimDriver_grf","MdlConvexMPC_forces","MdlConvexMPC_solver","MdlTrot_contact","MdlTrot_bodyref","MdlTrot_torquereq","MdlTrot_torquescale","MdlPosVelEstimator_foottruth"]'

exec 9>"$OUT/.simlock"
if ! flock -n 9; then
  echo "run_cell: another simulator run holds the lock; waiting..." >&2
  flock 9
fi

{
  echo "name       $NAME"
  echo "date       $(date -Is)"
  echo "git        $(git -C "$REPO" rev-parse --short HEAD) ($(git -C "$REPO" branch --show-current))"
  echo "dirty      $(git -C "$REPO" status --porcelain | tr '\n' ';')"
  echo "duration   $DUR"
  echo "overrides  $*"
} > "$OUT/$NAME.manifest"

cd "$REPO/bin"
set +e
./sim.sh \
  -c 'simulation.headless = true' \
  -c 'simulation.realtime = false' \
  -c "supervisor.autorun_trot_duration = $DUR" \
  -c 'supervisor.log.enable = true' \
  -c 'supervisor.log.start = 0' \
  -c 'supervisor.log.period = 2' \
  -c "supervisor.log.file_name = \"$OUT/$NAME.mat\"" \
  -c "supervisor.log.vars = $VARS" \
  -c "mpc.dump_path = \"$OUT/$NAME.bin\"" \
  "$@" > "$OUT/$NAME.log" 2>&1
rc=$?
set -e

echo "run_cell: $NAME exit=$rc"
# Exit code alone does not distinguish success from failure: an MdlTrot ERROR
# still reaches a clean autorun exit. Surface the two things that actually tell.
grep -iE 'error|fatal|failed|torque limited' "$OUT/$NAME.log" | head -5 || true
[[ -f "$OUT/$NAME.mat" ]] && echo "run_cell: wrote $OUT/$NAME.mat ($(stat -c%s "$OUT/$NAME.mat") bytes)" \
                          || echo "run_cell: NO .mat WRITTEN -- suspect the 128-double log ceiling or a bad variable name" >&2
exit $rc
