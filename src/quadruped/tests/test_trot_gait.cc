/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// Tests for the foot trajectory scheduler in MdlTrot.
//
// The test that matters most is test_stance_constant_velocity. A planted foot
// has to translate at exactly the negated body velocity; anything else drags it
// along the ground. That is bad for the gait, and worse for the state estimator
// this module exists to exercise, whose kinematic measurement assumes a foot in
// contact is a fixed point in the world. A profile that eases in and out over
// stance looks smooth on a plot and quietly breaks that assumption, so it is
// pinned down here.
//
// The continuity and finite difference tests guard the other half: a swing
// trajectory whose end tangents do not match the stance velocity produces a
// commanded velocity step at touchdown, which the leg PD turns into an impact.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlTrot.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

// These tests exercise TrotGait directly, but linking pulls in the whole
// MdlTrot translation unit, whose module layer references the hardware
// singletons. Defining their statics satisfies the linker; none of them is ever
// instantiated here, so instance() simply returns null.
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                   \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << ": " #cond \
                << std::endl;                                           \
      std::exit(1);                                                     \
    }                                                                   \
  } while (0)

#define T_NEAR(a, b, tol)                                                     \
  do {                                                                        \
    double _d = std::fabs((a) - (b));                                         \
    if (!(_d <= (tol))) {                                                     \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b \
                << "| = " << _d << " > " << (tol) << std::endl;               \
      std::exit(1);                                                           \
    }                                                                         \
  } while (0)

static void checkVecNear(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol,
                         const char* what, int line) {
  const double err = (a - b).cwiseAbs().maxCoeff();
  if (!(err <= tol)) {
    std::cerr << "FAIL " << __FILE__ << ":" << line << ": " << what
              << " max abs difference " << err << " > " << tol << "\n"
              << "  got  [" << a.transpose() << "]\n"
              << "  want [" << b.transpose() << "]" << std::endl;
    std::exit(1);
  }
}

#define T_VEC_NEAR(a, b, tol, what) checkVecNear(a, b, tol, what, __LINE__)

// A deliberately asymmetric commanded twist. Equal components or a pure x
// motion would let an axis swap hide.
static const Eigen::Vector3d kU(-0.15, 0.04, 0.0);

static TrotGait makeGait(double duty = 0.5) {
  TrotGait::params_t p;
  p.period = 0.5;
  p.duty = duty;
  p.swing_height = 0.05;
  p.phase_offset[0] = 0.0;
  p.phase_offset[1] = 0.5;
  p.phase_offset[2] = 0.5;
  p.phase_offset[3] = 0.0;

  TrotGait g;
  g.setParams(p);
  return g;
}

// ---------------------------------------------------------------------------

void test_phase_schedule() {
  std::cout << "test_phase_schedule..." << std::endl;

  const TrotGait g = makeGait();
  const double T = g.getParams().period;

  for (int k = 0; k <= 400; k++) {
    const double t = k * T / 400.0;

    // Diagonals move together: FL with RR, FR with RL.
    T_NEAR(g.legPhase(0, t), g.legPhase(3, t), 1e-15);
    T_NEAR(g.legPhase(1, t), g.legPhase(2, t), 1e-15);

    // And the two pairs are exactly half a cycle apart. Compare mod 1, since
    // the difference wraps.
    double diff = g.legPhase(0, t) - g.legPhase(1, t);
    diff -= std::floor(diff);
    T_NEAR(diff, 0.5, 1e-12);
  }

  // Phase stays in [0, 1) and wraps cleanly, for negative times too.
  for (int k = -20; k <= 20; k++) {
    const double t = k * T * 0.37;
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      const double phi = g.legPhase(leg, t);
      T_CHECK(phi >= 0.0 && phi < 1.0);
      T_NEAR(g.legPhase(leg, t + T), phi, 1e-12);
    }
  }

  std::cout << "  PASS" << std::endl;
}

void test_duty_factor() {
  std::cout << "test_duty_factor..." << std::endl;

  for (double duty : {0.5, 0.6, 0.75}) {
    const TrotGait g = makeGait(duty);
    const double T = g.getParams().period;

    const int N = 20000;
    int stanceCount[TrotGait::NUM_LEGS] = {0, 0, 0, 0};
    int minSupport = TrotGait::NUM_LEGS;

    for (int k = 0; k < N; k++) {
      const double t = (k + 0.5) * T / N;
      int support = 0;
      for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
        if (g.inStance(leg, t)) {
          stanceCount[leg]++;
          support++;
        }
      }
      if (support < minSupport) minSupport = support;
    }

    // Each leg is planted for the configured fraction of the cycle.
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++)
      T_NEAR((double)stanceCount[leg] / N, duty, 1e-3);

    // With diagonals half a cycle apart and duty at or above 0.5, there are
    // always at least two feet down. A trot with no support interval would be
    // a hopping gait, not a walking one.
    T_CHECK(minSupport >= 2);
  }

  std::cout << "  PASS" << std::endl;
}

void test_stance_constant_velocity() {
  std::cout << "test_stance_constant_velocity..." << std::endl;

  const TrotGait g = makeGait();
  const double T = g.getParams().period;
  const double beta = g.getParams().duty;

  for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
    int samples = 0;
    for (int k = 0; k < 5000; k++) {
      const double t = (k + 0.5) * T / 5000.0;
      if (!g.inStance(leg, t)) continue;
      samples++;

      Eigen::Vector3d dp, dv;
      g.sample(leg, t, kU, dp, dv);

      // The whole point: velocity is exactly the stance velocity everywhere in
      // stance, not a profile that eases in and out.
      T_VEC_NEAR(dv, kU, 1e-15, "stance velocity");

      // And the foot stays on the ground while it is there.
      T_NEAR(dp.z(), 0.0, 1e-15);
    }
    T_CHECK(samples > 0);
  }

  // Stance sweeps exactly one stride, from the front of the stroke to the back.
  const double stanceTime = beta * T;
  const TrotGait::params_t& p = g.getParams();
  const double phi0 = p.phase_offset[0];
  // Time at which leg 0's stance begins and ends, given its offset.
  const double tStart = (1.0 - phi0) * T;  // phase wraps to 0 here
  Eigen::Vector3d dpStart, dpEnd, dv;
  g.sample(0, tStart + 1e-9, kU, dpStart, dv);
  g.sample(0, tStart + stanceTime - 1e-9, kU, dpEnd, dv);

  T_VEC_NEAR(dpStart, -kU * stanceTime * 0.5, 1e-6, "stance start position");
  T_VEC_NEAR(dpEnd, kU * stanceTime * 0.5, 1e-6, "stance end position");
  T_VEC_NEAR(dpEnd - dpStart, kU * stanceTime, 1e-6, "stride swept during stance");

  std::cout << "  PASS" << std::endl;
}

void test_cycle_continuity() {
  std::cout << "test_cycle_continuity..." << std::endl;

  for (double duty : {0.5, 0.65}) {
    const TrotGait g = makeGait(duty);
    const double T = g.getParams().period;
    const double eps = 1e-7;

    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      // Walk a full cycle at fine resolution and assert nothing ever jumps.
      // This catches both transitions without having to locate them.
      const int N = 100000;
      Eigen::Vector3d prevP, prevV;
      g.sample(leg, 0.0, kU, prevP, prevV);

      for (int k = 1; k <= N; k++) {
        const double t = k * T / N;
        Eigen::Vector3d dp, dv;
        g.sample(leg, t, kU, dp, dv);

        // Bound the step by what the largest speed in the cycle can cover in
        // one sample, with room to spare. A true discontinuity is orders of
        // magnitude larger than this.
        T_CHECK((dp - prevP).norm() < 0.01);
        T_CHECK((dv - prevV).norm() < 0.05);

        prevP = dp;
        prevV = dv;
      }

      // The cycle closes on itself in both position and velocity.
      Eigen::Vector3d p0, v0, p1, v1;
      g.sample(leg, 0.123, kU, p0, v0);
      g.sample(leg, 0.123 + T, kU, p1, v1);
      T_VEC_NEAR(p1, p0, 1e-12, "position periodicity");
      T_VEC_NEAR(v1, v0, 1e-12, "velocity periodicity");

      // Pin the two transitions exactly. Leg 0's phase is t/T, so stance runs
      // to duty and swing from there back around to 0.
      Eigen::Vector3d pBefore, vBefore, pAfter, vAfter;
      const double phi0 = g.getParams().phase_offset[leg];
      const double tLiftoff = (duty - phi0 + 1.0) * T;

      g.sample(leg, tLiftoff - eps, kU, pBefore, vBefore);
      g.sample(leg, tLiftoff + eps, kU, pAfter, vAfter);
      T_VEC_NEAR(pAfter, pBefore, 1e-6, "position across liftoff");
      T_VEC_NEAR(vAfter, vBefore, 1e-5, "velocity across liftoff");

      const double tTouchdown = (1.0 - phi0 + 1.0) * T;
      g.sample(leg, tTouchdown - eps, kU, pBefore, vBefore);
      g.sample(leg, tTouchdown + eps, kU, pAfter, vAfter);
      T_VEC_NEAR(pAfter, pBefore, 1e-6, "position across touchdown");
      T_VEC_NEAR(vAfter, vBefore, 1e-5, "velocity across touchdown");

      // Touchdown must happen at the stance velocity, or the foot lands with
      // horizontal slip and scuffs.
      T_VEC_NEAR(vAfter, kU, 1e-5, "velocity at touchdown");
    }
  }

  std::cout << "  PASS" << std::endl;
}

void test_swing_clearance() {
  std::cout << "test_swing_clearance..." << std::endl;

  const TrotGait g = makeGait();
  const double T = g.getParams().period;
  const double beta = g.getParams().duty;
  const double h = g.getParams().swing_height;
  const double swingTime = (1.0 - beta) * T;

  const int leg = 0;
  const double phi0 = g.getParams().phase_offset[leg];
  const double tLiftoff = (beta - phi0 + 1.0) * T;

  double zmax = -1.0;
  double zmaxAt = 0.0;
  const int N = 10000;
  for (int k = 0; k <= N; k++) {
    const double us = (double)k / N;
    Eigen::Vector3d dp, dv;
    g.sample(leg, tLiftoff + us * swingTime, kU, dp, dv);

    // The foot never digs into the ground during swing.
    T_CHECK(dp.z() >= -1e-12);
    if (dp.z() > zmax) {
      zmax = dp.z();
      zmaxAt = us;
    }
  }

  // Peak clearance is the configured height, reached at mid swing.
  T_NEAR(zmax, h, 1e-6);
  T_NEAR(zmaxAt, 0.5, 1e-3);

  // Both ends of swing sit on the ground with zero vertical rate, so the foot
  // is set down rather than driven down.
  Eigen::Vector3d dp, dv;
  g.sample(leg, tLiftoff + 1e-9, kU, dp, dv);
  T_NEAR(dp.z(), 0.0, 1e-9);
  T_NEAR(dv.z(), 0.0, 1e-5);

  g.sample(leg, tLiftoff + swingTime - 1e-9, kU, dp, dv);
  T_NEAR(dp.z(), 0.0, 1e-9);
  T_NEAR(dv.z(), 0.0, 1e-5);

  std::cout << "  PASS" << std::endl;
}

void test_swing_returns_the_stride() {
  std::cout << "test_swing_returns_the_stride..." << std::endl;

  const TrotGait g = makeGait();
  const double T = g.getParams().period;
  const double beta = g.getParams().duty;
  const double stanceTime = beta * T;
  const double swingTime = (1.0 - beta) * T;

  const int leg = 1;
  const double phi0 = g.getParams().phase_offset[leg];
  const double tLiftoff = (beta - phi0 + 1.0) * T;

  Eigen::Vector3d pLift, pLand, dv;
  g.sample(leg, tLiftoff + 1e-9, kU, pLift, dv);
  g.sample(leg, tLiftoff + swingTime - 1e-9, kU, pLand, dv);

  // Swing undoes exactly what stance did, so the foot returns to where the
  // next stance expects it and the gait neither creeps nor shrinks.
  T_VEC_NEAR(pLand - pLift, -kU * stanceTime, 1e-6, "displacement over swing");

  std::cout << "  PASS" << std::endl;
}

void test_velocity_matches_finite_difference() {
  std::cout << "test_velocity_matches_finite_difference..." << std::endl;

  // Differentiating the reported position and comparing against the reported
  // velocity catches sign and scale slips in the Hermite tangents, which are
  // otherwise invisible: a wrong tangent still produces a smooth looking curve
  // through the right endpoints.
  for (double duty : {0.5, 0.65}) {
    const TrotGait g = makeGait(duty);
    const double T = g.getParams().period;
    const double dt = 1e-7;

    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      const int N = 2000;
      for (int k = 0; k < N; k++) {
        const double t = (k + 0.5) * T / N;

        // Skip samples straddling a phase transition, where a central
        // difference spans two different analytic pieces.
        const double phi = g.legPhase(leg, t);
        const double margin = dt / T * 4.0;
        if (phi < margin || std::fabs(phi - duty) < margin || phi > 1.0 - margin)
          continue;

        Eigen::Vector3d pPlus, pMinus, dp, dv, unused;
        g.sample(leg, t + dt, kU, pPlus, unused);
        g.sample(leg, t - dt, kU, pMinus, unused);
        g.sample(leg, t, kU, dp, dv);

        const Eigen::Vector3d numeric = (pPlus - pMinus) / (2.0 * dt);
        T_VEC_NEAR(dv, numeric, 1e-4, "velocity against finite differences");
      }
    }
  }

  std::cout << "  PASS" << std::endl;
}

void test_zero_command_steps_in_place() {
  std::cout << "test_zero_command_steps_in_place..." << std::endl;

  // With no commanded twist the gait must still lift and set down the feet
  // without translating them. MdlTrot relies on this: its stride ramp starts at
  // zero, and the PREP handoff assumes every leg's offset is zero at t = 0.
  const TrotGait g = makeGait();
  const double T = g.getParams().period;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  bool sawLift = false;
  for (int k = 0; k <= 2000; k++) {
    const double t = k * T / 2000.0;
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      Eigen::Vector3d dp, dv;
      g.sample(leg, t, zero, dp, dv);

      T_NEAR(dp.x(), 0.0, 1e-15);
      T_NEAR(dp.y(), 0.0, 1e-15);
      T_NEAR(dv.x(), 0.0, 1e-15);
      T_NEAR(dv.y(), 0.0, 1e-15);
      T_CHECK(dp.z() >= -1e-15);
      if (dp.z() > 0.01) sawLift = true;
    }
  }
  T_CHECK(sawLift);

  // Every leg is exactly on its nominal footprint at t = 0, whatever the duty.
  for (double duty : {0.5, 0.65, 0.8}) {
    const TrotGait gd = makeGait(duty);
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      Eigen::Vector3d dp, dv;
      gd.sample(leg, 0.0, zero, dp, dv);
      T_VEC_NEAR(dp, zero, 1e-15, "offset at gait start");
    }
  }

  std::cout << "  PASS" << std::endl;
}

void test_degenerate_params_are_clamped() {
  std::cout << "test_degenerate_params_are_clamped..." << std::endl;

  // A zero period or a duty at either extreme would divide by zero somewhere in
  // the schedule. Config comes from a hand edited TOML file, so this is a
  // realistic thing to be handed.
  TrotGait::params_t bad;
  bad.period = 0.0;
  bad.duty = 0.0;
  bad.swing_height = -1.0;

  TrotGait g;
  g.setParams(bad);
  T_CHECK(g.getParams().period > 0.0);
  T_CHECK(g.getParams().duty > 0.0 && g.getParams().duty < 1.0);
  T_CHECK(g.getParams().swing_height >= 0.0);

  bad.duty = 1.5;
  g.setParams(bad);
  T_CHECK(g.getParams().duty < 1.0);

  // And whatever comes out must still produce finite commands.
  for (int k = 0; k < 100; k++) {
    Eigen::Vector3d dp, dv;
    g.sample(0, k * 0.01, kU, dp, dv);
    T_CHECK(dp.allFinite());
    T_CHECK(dv.allFinite());
  }

  std::cout << "  PASS" << std::endl;
}

// Stance progress is what the state estimator's contact trust ramp is written
// against, so it has to mean exactly what that ramp assumes: how far through
// stance a foot is, and nothing at all while it is airborne.
void test_stance_progress() {
  std::cout << "test_stance_progress..." << std::endl;

  const TrotGait g = makeGait();
  const double T = g.getParams().period;
  const double beta = g.getParams().duty;

  // Leg 0 has no phase offset, so its stance runs over the first half cycle.
  T_NEAR(g.stanceProgress(0, 0.0), 0.0, 1e-12);
  T_NEAR(g.stanceProgress(0, 0.125), 0.5, 1e-12);
  T_NEAR(g.stanceProgress(0, 0.2499), 0.9996, 1e-9);
  T_NEAR(g.stanceProgress(0, 0.25), 0.0, 1e-12);  // liftoff
  T_NEAR(g.stanceProgress(0, 0.4), 0.0, 1e-12);   // mid swing

  // Always a valid phase, for any time, including negative ones.
  std::mt19937 rng(20260804);
  std::uniform_real_distribution<double> uni(-100.0, 100.0);
  for (int k = 0; k < 10000; k++) {
    const double t = uni(rng);
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      const double s = g.stanceProgress(leg, t);
      T_CHECK(std::isfinite(s));
      T_CHECK(s >= 0.0 && s < 1.0);

      // Nonzero exactly when the leg is planted and has been for a moment.
      // The instant of touchdown reports zero even though the leg is in
      // stance, which is deliberate rather than an off-by-one: a foot that has
      // only just arrived is the least trustworthy thing in the gait, and the
      // estimator's ramp is built to start it at nothing.
      const double phi = g.legPhase(leg, t);
      T_CHECK((s > 0.0) == (g.inStance(leg, t) && phi > 0.0));

      // And it is the stance fraction, not the cycle fraction.
      if (s > 0.0) T_NEAR(s, phi / beta, 1e-12);
    }
  }

  // Every leg reaches full stance once per cycle, so nothing is stuck at zero.
  for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
    double peak = 0.0;
    for (int k = 0; k <= 1000; k++)
      peak = std::max(peak, g.stanceProgress(leg, k * T / 1000.0));
    T_CHECK(peak > 0.99);
  }

  std::cout << "  PASS" << std::endl;
}

// The MPC horizon is built by querying this schedule at elapsed + k*dt_mpc for
// a fixed number of steps ahead, which is only legitimate because TrotGait is a
// pure function of time: nothing here caches a phase or advances an internal
// clock. This test pins that property down, and pins the alignment of the
// horizon's first interval to the present, which is the classic place for a
// controller to end up reacting exactly one interval late.
void test_future_horizon_samples() {
  std::cout << "test_future_horizon_samples..." << std::endl;

  const TrotGait g = makeGait();
  const double T = g.getParams().period;
  const double beta = g.getParams().duty;
  const double dt = 0.04;  // mpc.dt
  const int H = 10;        // MdlConvexMPC::HORIZON

  for (double elapsed : {0.0, 0.0173, 0.0731, 0.31, 1.234, 7.77}) {
    bool now[TrotGait::NUM_LEGS];
    Eigen::Vector3d nowPos[TrotGait::NUM_LEGS], nowVel[TrotGait::NUM_LEGS];
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      now[leg] = g.inStance(leg, elapsed);
      g.sample(leg, elapsed, kU, nowPos[leg], nowVel[leg]);
    }

    // Walk the whole horizon, twice, interleaving the two passes so a cached
    // phase or a mutated datum would show up as a disagreement.
    bool horizon[H][TrotGait::NUM_LEGS];
    for (int k = 0; k < H; k++) {
      const double t = elapsed + k * dt;
      for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
        horizon[k][leg] = g.inStance(leg, t);

        Eigen::Vector3d p1, v1, p2, v2;
        g.sample(leg, t, kU, p1, v1);
        g.stanceProgress(leg, t);
        g.legPhase(leg, t);
        g.sample(leg, t, kU, p2, v2);
        T_VEC_NEAR(p2, p1, 0.0, "repeated future sample position");
        T_VEC_NEAR(v2, v1, 0.0, "repeated future sample velocity");
        T_CHECK(g.inStance(leg, t) == horizon[k][leg]);

        // A foot the horizon calls planted has to be on the ground, or the
        // moment arm handed to the QP describes a foot in the air pushing.
        if (horizon[k][leg]) {
          T_NEAR(p1.z(), 0.0, 1e-15);
          T_VEC_NEAR(v1, kU, 1e-15, "future stance velocity");
        }
      }
    }

    // Querying the future left the present exactly where it was.
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      T_CHECK(g.inStance(leg, elapsed) == now[leg]);
      Eigen::Vector3d p, v;
      g.sample(leg, elapsed, kU, p, v);
      T_VEC_NEAR(p, nowPos[leg], 0.0, "present position after future queries");
      T_VEC_NEAR(v, nowVel[leg], 0.0, "present velocity after future queries");

      // Interval zero of the horizon is now, not one step from now. If this
      // ever slips the controller applies interval one's forces during
      // interval zero, which reads as a badly tuned gait rather than as a bug.
      T_CHECK(horizon[0][leg] == now[leg]);
    }

    // Diagonals stay in step and the pairs stay opposed at every horizon step,
    // for a pure trot. The MPC's support polygon is built from this.
    for (int k = 0; k < H; k++) {
      T_CHECK(horizon[k][0] == horizon[k][3]);
      T_CHECK(horizon[k][1] == horizon[k][2]);
      T_CHECK(horizon[k][0] != horizon[k][1]);
    }

    // The horizon is 0.4 s against a 0.5 s cycle, so every leg must both lift
    // and land within it. A schedule that reported a constant mask over the
    // horizon would leave the solver planning for a support state that never
    // changes, which is the failure this catches.
    for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
      int changes = 0;
      for (int k = 1; k < H; k++)
        if (horizon[k][leg] != horizon[k - 1][leg]) changes++;
      T_CHECK(changes >= 1);
    }
  }

  // Transition alignment: contact flips exactly at the analytic liftoff and
  // touchdown instants, so a horizon step landing arbitrarily close to one gets
  // the side it is actually on.
  const double eps = 1e-9;
  for (int leg = 0; leg < TrotGait::NUM_LEGS; leg++) {
    const double phi0 = g.getParams().phase_offset[leg];
    const double tLiftoff = (beta - phi0 + 1.0) * T;
    const double tTouchdown = (1.0 - phi0 + 1.0) * T;

    T_CHECK(g.inStance(leg, tLiftoff - eps));
    T_CHECK(!g.inStance(leg, tLiftoff + eps));
    T_CHECK(!g.inStance(leg, tTouchdown - eps));
    T_CHECK(g.inStance(leg, tTouchdown + eps));
  }

  std::cout << "  PASS" << std::endl;
}

int main() {
  std::cout << "=== Trot Gait Tests ===" << std::endl;
  test_phase_schedule();
  test_stance_progress();
  test_duty_factor();
  test_stance_constant_velocity();
  test_cycle_continuity();
  test_swing_clearance();
  test_swing_returns_the_stride();
  test_velocity_matches_finite_difference();
  test_zero_command_steps_in_place();
  test_degenerate_params_are_clamped();
  test_future_horizon_samples();
  std::cout << "All trot gait tests passed." << std::endl;
  return 0;
}
