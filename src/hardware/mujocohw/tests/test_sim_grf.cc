/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// Sanity-anchors MdlSimDriver_grf: stands the Go2 model up on its own (via the
// same MdlLegControl feedforward chain test_stance_force_sign.cc uses to pin
// the controller-side sign) and checks that MdlSimDriver::getFootGRF() reports
// four non-negative, world-frame-vertical ground reaction forces that agree
// with physical reality. This is the ground-truth half of the comparison the
// instrumentation gap was about -- getFootGRF() has to agree with physical
// reality (weight support), independently of whatever MdlConvexMPC commanded.
//
// The static check alone is not enough: a robot settled into a stationary
// stance sits at a contact distance that happens to be negative, and never
// exercises the case that actually broke getFootGRF() during logging (a foot
// well within go2.xml's 1mm margin, dist between 0 and margin, still carrying
// real load). So a second, dynamic phase drives each foot's commanded vertical
// force through an oscillation while the robot stands, and checks two things:
//
//  - vertical momentum balance over the window: integral(sum(Fz) - m*g) dt
//    must equal m*(vz_end - vz_start) (Newton's second law on the trunk,
//    integrated). Unlike "time-average of sum(Fz) == m*g", this holds even
//    while the trunk is accelerating -- e.g. sinking under an open-loop
//    feedforward stance with realistic (low) joint damping, which is exactly
//    what this scenario does. It would still have caught the original bug,
//    which was dropping real contact impulse to exact zero, not physics.
//  - no foot's force drops to exactly zero for one sample while solidly
//    loaded on both sides of it -- the discontinuous-dropout signature the
//    dist>0.0 filter produced. This check is damping-independent and is the
//    one that most directly targets that bug.
//
// A separate precondition on the trunk's own vertical drift during the
// dynamic phase distinguishes "the invariant was violated" from "the test
// scenario itself fell over"; the two must not be conflated, or a future
// change to model parameters (joint damping, contact stiffness, ...) that
// destabilizes this open-loop stance would show up as a mystifying force
// discrepancy instead of a clear "scenario destabilised" failure.
//
// Usage: test_sim_grf <path-to-scene.xml>

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../MdlSimDriver.hh"
#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

// MdlSimDriver links mujocohw, which already carries SimHW.o's
// HARDWARE_IMPL(ClockHW/MotorHW/IMUHW) -- declaring them again here, the way
// test_stance_force_sign.cc does for its mujoco-only link, would collide.
using namespace rtcore;

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                 \
  do {                                                                \
    if (!(cond)) {                                                    \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                       \
    }                                                                 \
  } while (0)

// Joint sign between this code base and the MuJoCo model, per motor index
// 3*leg + joint. Identical to MdlSimDriver::_polarity and to
// test_stance_force_sign.cc's POLARITY (same underlying convention).
static const double POLARITY[12] = {1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1};

// trot.toml's nominal footprint: hip position shifted by [0, +-0.10, -0.28].
static Eigen::Vector3d nominalFoot(const QuadrupedKinematics &kin, int leg) {
  Eigen::Vector3d p = kin.getKinematicParams().hip_positions.row(leg).transpose();
  p[1] += 0.10 * (leg % 2 == 0 ? 1.0 : -1.0);
  p[2] += -0.28;
  return p;
}

// Foot sphere radius from go2.xml's `foot` class; the foot centre sits at the
// nominal -0.28, so this is how high the base has to start for the feet to be
// just touching flat ground.
static const double kFootRadius = 0.022;
static const double kStartHeight = 0.28 + kFootRadius;

// Joint damping the real command carries alongside the feedforward torque;
// see trot.toml. Included so the torque applied here is the one the robot
// would actually get, not a cleaner version of it.
static const double kJointDamping = 0.2;

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <path-to-scene.xml>\n", argv[0]);
    return 1;
  }

  char error[1024] = "";
  mjModel *m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) {
    fprintf(stderr, "FAIL: could not load %s: %s\n", argv[1], error);
    return 1;
  }
  mjData *d = mj_makeData(m);
  T_CHECK(d != nullptr);
  T_CHECK(m->nu == 12);

  // Total mass of the model, so the expected support force matches the robot
  // the simulator is actually integrating rather than a number out of a TOML.
  double mass = 0.0;
  for (int b = 1; b < m->nbody; b++) mass += m->body_mass[b];
  T_CHECK(mass > 10.0 && mass < 20.0);
  // g from the model's own gravity setting (not a hardcoded 9.81): the
  // momentum-balance check below is an exact identity, and should use the
  // exact g this simulation is actually integrating with.
  const double g = -m->opt.gravity[2];
  const double weight = mass * g;

  QuadrupedKinematics kin(createGo2Config());

  // Level attitude throughout (R_BW is the identity), so the body/world
  // transform from MdlTrot's stance branch is a plain negation: an upward
  // ground reaction force at the foot becomes a downward force commanded at
  // the foot. This is the exact chain test_stance_force_sign.cc pins the sign
  // of, reused here (and re-evaluated every step below with a time-varying
  // force) to get a robot that actually stands rather than one hand-posed to
  // look like it does.
  const Eigen::Matrix3d R_BW = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  MdlLegControl *lc[4];
  Eigen::Vector3d q_nom[4], p_nom[4];
  for (int leg = 0; leg < 4; leg++) {
    lc[leg] = new MdlLegControl(leg);
    lc[leg]->init();
    p_nom[leg] = nominalFoot(kin, leg);
    T_CHECK(kin.inverseKinematics(leg, p_nom[leg], q_nom[leg]));
  }

  // Modest Cartesian position/velocity gains around the fixed nominal
  // footprint p_nom, on top of the feedforward force below -- the same shape
  // of command MdlStand/MdlTrot actually issue (feedforward doing the bulk of
  // the work, a PD term correcting drift), rather than test_stance_force_sign.cc's
  // kp = kd = 0 (which is deliberately open-loop there, to isolate a pure sign
  // question over a fraction of a second). At go2.xml's now-realistic joint
  // damping (0.2, was a mujoco_menagerie house default of 2.0), pure
  // feedforward evaluated at a fixed q is not enough to hold this rig up over
  // the multi-second window this test needs: the actual joint state drifts
  // from q_nom, the fixed-q feedforward stops matching what is actually needed,
  // and the trunk runs away. These gains are damped conservatively (kd large
  // relative to kp) precisely because the first, stiffer attempt (kp=100,
  // kd=5) instead pumped an underdamped bounce. 50/25 held the footprint only
  // because go2.xml's feet then had the menagerie's 0.01 m rolling friction,
  // which pinned them; on the rolling-free feet the controller needs (condim 3)
  // the spheres creep outward on every unload and the trunk sinks 3.5 cm over
  // the window. 200/40 (damping ratio ~0.7 at ~4 kg per leg) keeps the drift
  // under 3 mm without a bounce.
  const Eigen::Vector3d kp_cart(200.0, 200.0, 200.0);
  const Eigen::Vector3d kd_cart(40.0, 40.0, 40.0);
  auto measuredQ = [&](int leg) {
    Eigen::Vector3d q;
    for (int j = 0; j < 3; j++) q[j] = POLARITY[3 * leg + j] * d->qpos[7 + 3 * leg + j];
    return q;
  };
  auto measuredQdot = [&](int leg) {
    Eigen::Vector3d qd;
    for (int j = 0; j < 3; j++) qd[j] = POLARITY[3 * leg + j] * d->qvel[6 + 3 * leg + j];
    return qd;
  };

  // tau_ff(fz[]): joint torque for a per-foot vertical ground reaction force
  // fz[leg] newtons plus the Cartesian PD correction above, evaluated at the
  // *measured* joint state every call -- not a constant computed once, since
  // the whole point of closing this loop is to react to where the leg
  // actually is.
  auto tauFor = [&](const double fz[4], double tau_ff[12]) {
    for (int leg = 0; leg < 4; leg++) {
      const Eigen::Vector3d f_grf_world(0.0, 0.0, fz[leg]);
      const Eigen::Vector3d f_foot_body = -R_BW.transpose() * f_grf_world;
      Eigen::Vector3d tau;
      lc[leg]->computeCartesianForceCommand(measuredQ(leg), measuredQdot(leg), p_nom[leg], zero,
                                            kp_cart, kd_cart, f_foot_body, tau);
      for (int j = 0; j < 3; j++) tau_ff[3 * leg + j] = tau[j];
    }
  };

  // Pose the robot standing, feet just touching.
  d->qpos[0] = 0.0;
  d->qpos[1] = 0.0;
  d->qpos[2] = kStartHeight;
  d->qpos[3] = 1.0;  // identity quaternion, w first
  d->qpos[4] = d->qpos[5] = d->qpos[6] = 0.0;
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++) d->qpos[7 + 3 * leg + j] = POLARITY[3 * leg + j] * q_nom[leg][j];
  mj_forward(m, d);

  MdlSimDriver driver;
  driver.setModelForTesting(m, d);

  // ---- Phase 1: static stance -----------------------------------------
  // Let it settle under a constant per-foot weight/4 command, recomputed
  // every step from the measured joint state (see tauFor above), plus joint
  // damping -- MdlSimDriver's control law with the commanded position equal
  // to the measured one, the way setCartesianForceCommand() emits it.
  const double fz_static[4] = {weight / 4.0, weight / 4.0, weight / 4.0, weight / 4.0};

  const double settleDuration = 1.0;  // [s]
  const int settleSteps = (int)(settleDuration / m->opt.timestep);
  for (int s = 0; s < settleSteps; s++) {
    double tau_static[12];
    tauFor(fz_static, tau_static);
    for (int i = 0; i < 12; i++) {
      const double vel = POLARITY[i] * d->actuator_velocity[i];
      const double tau = -kJointDamping * vel + tau_static[i];
      d->ctrl[i] = POLARITY[i] * tau;
    }
    mj_step(m, d);
  }

  // The robot must still be standing (not collapsed) for the GRF checks below
  // to mean anything.
  printf("settled base height: %.4f m (start %.4f m)\n", d->qpos[2], kStartHeight);
  T_CHECK(d->qpos[2] > 0.25);

  double grf[4][3];
  T_CHECK(driver.getFootGRF(grf));

  double sum_z = 0.0;
  for (int i = 0; i < 4; i++) {
    printf("static foot %d GRF: [% .4f, % .4f, % .4f] N\n", i, grf[i][0], grf[i][1], grf[i][2]);
    // A little slack below zero for solver/friction-cone numerical noise; the
    // point of this check is "no foot is pulling the robot down", not
    // "exactly zero or positive to the last bit".
    T_CHECK(grf[i][2] > -1e-3);
    sum_z += grf[i][2];
  }

  printf("static sum Fz = %.4f N, expected weight = %.4f N (mass %.4f kg)\n", sum_z, weight,
         mass);

  // Within a few percent of the static weight.
  T_CHECK(std::fabs(sum_z - weight) < 0.05 * weight);

  // ---- Phase 2: dynamic load/unload, checked as a momentum balance --------
  // Oscillate each foot's commanded vertical force around weight/4, with the
  // two diagonal pairs (FL+RR, leg indices 0 and 3; FR+RL, indices 1 and 2 --
  // QuadrupedKinematics::LegIndex order FL, FR, RL, RR) a half period out of
  // phase with each other, the way a trot shifts support between diagonal
  // pairs. Each diagonal pair's two forces move together, and the pairs move
  // oppositely, so the *commanded* total is pinned at the static weight at
  // every instant even though each foot individually swings from a small
  // fraction of nominal up past double -- this is a load redistribution
  // across feet (what an actual gait does), not a whole-body bounce or a
  // net tipping moment. Driving all four feet in unison, and phasing them a
  // quarter period apart, were both tried first and pumped either a bounce
  // resonance or an asymmetric tipping moment; the diagonal pairing keeps the
  // support pattern balanced.
  //
  // The check is *not* "time-average of sum(Fz) == weight": that only holds
  // when the trunk's average vertical acceleration is zero, which is not
  // guaranteed here (a low, realistic joint damping plus an open-loop
  // Cartesian PD is not a real height controller, and the trunk can drift
  // during the window). What is always true, regardless of gait or drift, is
  // Newton's second law on the trunk integrated over the window:
  //
  //   integral( sum(Fz) - m*g ) dt  ==  m * (vz_end - vz_start)
  //
  // i.e. the net impulse the ground actually delivered (beyond just
  // supporting weight) equals the trunk's own momentum change. That is what
  // is asserted below, in N*s, rather than comparing a time-average force to
  // m*g directly.
  const double freq = 2.0;       // [Hz]
  const double amplitude = 0.6;  // fraction of nominal, i.e. fz in [0.4, 1.6] x nominal
  const int periods = 2;
  const double dynDuration = periods / freq;  // [s]
  const int dynSteps = (int)(dynDuration / m->opt.timestep);
  const double dt = m->opt.timestep;

  std::vector<double> footZ[4];
  for (int i = 0; i < 4; i++) footZ[i].reserve(dynSteps);

  // FL(0) and RR(3) share one diagonal's phase; FR(1) and RL(2) the other,
  // pi out of phase with the first.
  const double legPhase[4] = {0.0, M_PI, M_PI, 0.0};

  const double hDynStart = d->qpos[2];
  const double vzStart = d->qvel[2];
  double maxDrift = 0.0;
  double impulseMeasured = 0.0;  // integral(sum(Fz) - weight) dt

  for (int s = 0; s < dynSteps; s++) {
    const double t = s * dt;
    double fz[4];
    for (int leg = 0; leg < 4; leg++) {
      fz[leg] =
          (weight / 4.0) * (1.0 + amplitude * std::sin(2.0 * M_PI * freq * t + legPhase[leg]));
    }

    double tau_dyn[12];
    tauFor(fz, tau_dyn);
    for (int i = 0; i < 12; i++) {
      const double vel = POLARITY[i] * d->actuator_velocity[i];
      const double tau = -kJointDamping * vel + tau_dyn[i];
      d->ctrl[i] = POLARITY[i] * tau;
    }
    mj_step(m, d);

    double grf_dyn[4][3];
    T_CHECK(driver.getFootGRF(grf_dyn));
    double s_z = 0.0;
    for (int i = 0; i < 4; i++) {
      footZ[i].push_back(grf_dyn[i][2]);
      s_z += grf_dyn[i][2];
    }
    impulseMeasured += (s_z - weight) * dt;

    const double drift = std::fabs(d->qpos[2] - hDynStart);
    if (drift > maxDrift) maxDrift = drift;
  }

  const double vzEnd = d->qvel[2];

  printf("post-oscillation base height: %.4f m (dynamic-phase start %.4f m)\n", d->qpos[2],
         hDynStart);
  printf("dynamic-phase max vertical drift: %.4f m\n", maxDrift);

  // Scenario precondition: this is "did the test rig stay standing", not "was
  // the invariant satisfied". Keeping the two separate means a future change
  // that destabilises this open-loop stance (a different joint damping, a
  // stiffer or softer contact, ...) reports plainly as that, rather than as a
  // baffling momentum-balance residual computed over a robot that was busy
  // falling over.
  const double kMaxAllowedDrift = 0.03;  // [m], "a few cm"
  if (maxDrift > kMaxAllowedDrift) {
    fprintf(stderr,
            "FAIL: scenario destabilised -- trunk drifted %.4f m during the dynamic phase "
            "(limit %.4f m). The momentum-balance check below would not mean anything run "
            "against a robot that is falling over; fix the test rig (stance gains, "
            "oscillation amplitude/frequency), not the tolerance.\n",
            maxDrift, kMaxAllowedDrift);
    return 1;
  }

  const double impulseExpected = mass * (vzEnd - vzStart);
  const double residual = impulseMeasured - impulseExpected;
  printf("dynamic-phase impulse: measured (integral of sum(Fz)-weight) = %.4f N*s\n",
         impulseMeasured);
  printf("dynamic-phase impulse: expected (mass * delta vz) = %.4f N*s (vz %.4f -> %.4f m/s)\n",
         impulseExpected, vzStart, vzEnd);
  printf("dynamic-phase impulse residual = %.4f N*s\n", residual);

  // Tolerance in N*s rather than a fraction of a possibly-near-zero average
  // force: a few percent of the gross impulse scale (weight integrated over
  // the window) this scenario is working with. Not exact even with a
  // correct GRF measurement, since vz is the free joint's (trunk) velocity
  // only and ignores the legs' own (much smaller) momentum, but the residual
  // from a correct measurement should be small against that scale; the
  // original bug lost about a quarter of the impulse, which this would catch
  // easily.
  const double impulseTolerance = 0.05 * weight * dynDuration;
  T_CHECK(std::fabs(residual) < impulseTolerance);

  // No foot may read an exact zero while sandwiched between two solidly
  // loaded samples on the same foot -- the discontinuous-dropout signature
  // the dist>0.0 filter produced (~72 N, one sample of exactly 0, back to
  // ~71 N, all within milliseconds). A genuine liftoff does not look like
  // this: with the load never commanded below 40% of nominal here, no foot
  // should separate from the ground at all.
  const double kLoadedThreshold = 50.0;  // [N]
  int minSamplesAboveThreshold = 0;
  for (int i = 0; i < 4; i++) {
    for (size_t t = 1; t + 1 < footZ[i].size(); t++) {
      if (footZ[i][t] == 0.0 && footZ[i][t - 1] > kLoadedThreshold &&
          footZ[i][t + 1] > kLoadedThreshold) {
        fprintf(stderr,
                "FAIL: foot %d dropout at sample %zu: %.2f -> 0.00 -> %.2f N\n", i, t,
                footZ[i][t - 1], footZ[i][t + 1]);
        return 1;
      }
      if (footZ[i][t] > kLoadedThreshold) minSamplesAboveThreshold++;
    }
  }
  // Sanity on the check itself: the oscillation must actually load the feet
  // above the dropout-detector's threshold at some point, or the absence of
  // dropouts above would be vacuous.
  T_CHECK(minSamplesAboveThreshold > 0);

  for (int leg = 0; leg < 4; leg++) {
    lc[leg]->uninit();
    delete lc[leg];
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  printf("test_sim_grf: all checks passed\n");
  return 0;
}
