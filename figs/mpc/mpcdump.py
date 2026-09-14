#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Parses the binary prediction-audit dump written by MdlConvexMPC::_dumpSolve().

See src/quadruped/MdlConvexMPC.cc:644-686 for the writer. Little-endian
float64 throughout:

  header (8 doubles): magic=20260828, version=1, horizon H, dt, NUM_STATES=13,
                       NUM_INPUTS=12, NUM_LEGS=4, solve_period
  then one record per solve:
    t (1), x0 (13), xref (H*13), U (H*12), Xpred (H*13), contact (H*4)

State layout (13): [roll, pitch, yaw, p_com_world(3), omega_world(3),
                    v_com_world(3), g_z]. Input layout (12): world-frame GRF
per leg, order FL, FR, RL, RR.

The file is flushed after every record, so a run killed mid-write leaves a
truncated final record rather than a truncated file overall; read_dump()
drops that partial tail rather than raising.
"""

import argparse
import sys

import numpy as np

MAGIC = 20260828.0
HEADER_DOUBLES = 8


def read_dump(path):
    """Returns (header, records) for the dump at path.

    header is a dict: magic, version, horizon, dt, num_states, num_inputs,
    num_legs, solve_period.

    records is a dict of numpy arrays, one row per solve (N = record count):
      t        (N,)
      x0       (N, num_states)
      xref     (N, horizon, num_states)
      U        (N, horizon, num_inputs)
      Xpred    (N, horizon, num_states)
      contact  (N, horizon, num_legs)   -- 1.0 stance, 0.0 swing

    Raises SystemExit if the magic number does not match, since that means
    this is not an MdlConvexMPC dump (wrong file, or a format nobody updated
    this parser for).
    """
    raw = np.fromfile(path, dtype="<f8")
    if raw.size < HEADER_DOUBLES:
        raise SystemExit(f"{path}: only {raw.size} doubles, too short for a header")

    magic, version, horizon, dt, num_states, num_inputs, num_legs, solve_period = \
        raw[:HEADER_DOUBLES]
    if magic != MAGIC:
        raise SystemExit(f"{path}: bad magic {magic!r}, expected {MAGIC!r}; "
                         "this is not an MdlConvexMPC solve dump")

    horizon = int(horizon)
    num_states = int(num_states)
    num_inputs = int(num_inputs)
    num_legs = int(num_legs)
    header = dict(magic=magic, version=int(version), horizon=horizon, dt=float(dt),
                 num_states=num_states, num_inputs=num_inputs, num_legs=num_legs,
                 solve_period=float(solve_period))

    record_doubles = (1 + num_states + horizon * num_states + horizon * num_inputs
                     + horizon * num_states + horizon * num_legs)
    body = raw[HEADER_DOUBLES:]
    n_full = body.size // record_doubles
    leftover = body.size - n_full * record_doubles
    if leftover:
        print(f"note: {path}: dropping a truncated final record "
              f"({leftover} of {record_doubles} doubles present) -- the run "
              "was likely killed mid-write")
    body = body[:n_full * record_doubles].reshape(n_full, record_doubles)

    i = 0
    t = body[:, i]; i += 1
    x0 = body[:, i:i + num_states]; i += num_states
    xref = body[:, i:i + horizon * num_states].reshape(n_full, horizon, num_states)
    i += horizon * num_states
    U = body[:, i:i + horizon * num_inputs].reshape(n_full, horizon, num_inputs)
    i += horizon * num_inputs
    Xpred = body[:, i:i + horizon * num_states].reshape(n_full, horizon, num_states)
    i += horizon * num_states
    contact = body[:, i:i + horizon * num_legs].reshape(n_full, horizon, num_legs)
    i += horizon * num_legs

    records = dict(t=t, x0=x0, xref=xref, U=U, Xpred=Xpred, contact=contact)
    return header, records


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dumpfile", help="path to the binary dump written by MdlConvexMPC")
    args = ap.parse_args()

    header, records = read_dump(args.dumpfile)
    print(f"{args.dumpfile}:")
    for k, v in header.items():
        print(f"  {k}: {v}")
    n = len(records["t"])
    print(f"  records: {n}")
    if n:
        print(f"  t: {records['t'][0]:.3f} .. {records['t'][-1]:.3f} s")


if __name__ == "__main__":
    sys.exit(main())
