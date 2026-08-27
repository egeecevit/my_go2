/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// Locks down the sign of the stance ground reaction force, end to end.
//
// The convex MPC returns f_grf^W, the force the ground applies *to the robot*,
// so a standing robot has positive z. What the leg controller wants is the
// force the actuators apply *at the foot*, which is the opposite and in the
// body frame:
//
//     f_foot_B = -R_BW^T * f_grf_W
//
// and the joint torque is J^T f_foot_B. There are three chances to flip a sign
// in that sentence -- the negation, the transpose, and the joint polarity
// between this code base and the MuJoCo model -- and getting any of them wrong
// produces a controller that pulls the robot into the floor while every unit
// test that stops at the Jacobian still passes. So this one does not stop
// there: it loads the actual Go2 model, applies the torque the chain produces,
// and asks the simulator whether the robot stayed up.
//
// The test is a comparison rather than an absolute threshold. Feedforward alone
// cannot hold a pose indefinitely -- there is no loop closed around anything,
// and the leg segments' own weight is uncompensated, since the single rigid
// body model has no term for it -- so the question asked is whether the
// intended sign keeps the robot standing while the flipped one collapses it.
// That difference is enormous and does not depend on fine tuning.
//
// Usage: test_stance_force_sign <path-to-scene.xml>
//
// NOTE: this is meaningful only once MdlLegControl's Cartesian force helper is
// implemented. Against its stub the commanded torque is zero for both signs,
// the two runs are identical, and the comparison below fails -- which is the
// correct report for "the sign has not been verified".

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

// MuJoCo is driven directly here rather than through MdlSimDriver: this test
// wants physics, not a window, a scheduler or a hardware abstraction. So the
// singleton statics are declared locally, exactly as the quadruped tests do,
// instead of being pulled in with SimHW.o.
using namespace rtcore;
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                 \
  do {                                                                \
    if (!(cond)) {                                                    \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                       \
    }                                                                 \
  } while (0)

// Joint sign between this code base and the MuJoCo model, per motor index
// 3*leg + joint. Identical to MdlSimDriver::_polarity and to the abduction
// entries of QuadrupedKinematics' joint_directions, which is not a coincidence:
// both express the same convention difference on the two right-hand legs.
static const double POLARITY[12] = {1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1};

// trot.toml's nominal footprint: hip position shifted by [0, +-0.10, -0.28].
static Eigen::Vector3d nominalFoot(const QuadrupedKinematics &kin, int leg) {
  Eigen::Vector3d p = kin.getKinematicParams().hip_positions.row(leg).transpose();
  p[1] += 0.10 * (leg % 2 == 0 ? 1.0 : -1.0);
  p[2] += -0.28;
  return p;
}

// Foot sphere radius from go2.xml's `foot` class. The foot centre sits at the
// nominal -0.28, so this is how high the base has to start for the feet to be
// just touching flat ground.
static const double kFootRadius = 0.022;
static const double kStartHeight = 0.28 + kFootRadius;

// Joint damping the real command carries alongside the feedforward torque; see
// trot.toml. Included so the torque applied here is the one the robot would
// actually get, not a cleaner version of it.
static const double kJointDamping = 0.2;

// Runs the model for `duration` with a constant feedforward joint torque and
// returns the base height it ends at. `sign` flips the ground reaction force,
// which is the whole experiment.
static double runStance(mjModel *m, mjData *d, const double tau_ff[12], double sign,
                        double duration) {
  mj_resetData(m, d);

  d->qpos[0] = 0.0;
  d->qpos[1] = 0.0;
  d->qpos[2] = kStartHeight;
  d->qpos[3] = 1.0;  // identity quaternion, w first
  d->qpos[4] = d->qpos[5] = d->qpos[6] = 0.0;

  // Joint angles were written into tau_ff's caller's frame; the pose is
  // restored from the same nominal configuration here.
  QuadrupedKinematics kin(createGo2Config());
  for (int leg = 0; leg < 4; leg++) {
    Eigen::Vector3d q;
    kin.inverseKinematics(leg, nominalFoot(kin, leg), q);
    for (int j = 0; j < 3; j++) d->qpos[7 + 3 * leg + j] = POLARITY[3 * leg + j] * q[j];
  }

  mj_forward(m, d);

  const int steps = (int)(duration / m->opt.timestep);
  for (int s = 0; s < steps; s++) {
    for (int i = 0; i < 12; i++) {
      // MdlSimDriver's control law with kp = 0 and the commanded position equal
      // to the measured one, which is exactly what setCartesianForceCommand()
      // emits: damping plus feedforward torque, no position term at all.
      const double vel = POLARITY[i] * d->actuator_velocity[i];
      const double tau = -kJointDamping * vel + sign * tau_ff[i];
      d->ctrl[i] = POLARITY[i] * tau;
    }
    mj_step(m, d);
  }

  return d->qpos[2];
}

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

  // Total mass of the model, so the support force matches the robot the
  // simulator is actually integrating rather than the number in mpc.toml.
  double mass = 0.0;
  for (int b = 1; b < m->nbody; b++) mass += m->body_mass[b];
  T_CHECK(mass > 10.0 && mass < 20.0);
  const double weight = mass * 9.81;

  QuadrupedKinematics kin(createGo2Config());

  // The feedforward torque, built the way MdlTrot's stance branch builds it.
  // Level attitude, so R_BW is the identity and the transform is a plain
  // negation: an upward ground reaction force becomes a downward force applied
  // at the foot, which is what pressing on the ground means.
  const Eigen::Vector3d f_grf_world(0.0, 0.0, weight / 4.0);
  const Eigen::Matrix3d R_BW = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d f_foot_body = -R_BW.transpose() * f_grf_world;
  T_CHECK(f_foot_body.z() < 0.0);

  double tau_ff[12] = {0};
  bool anyTorque = false;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  for (int leg = 0; leg < 4; leg++) {
    MdlLegControl lc(leg);
    lc.init();

    Eigen::Vector3d q;
    T_CHECK(kin.inverseKinematics(leg, nominalFoot(kin, leg), q));

    // Zero Cartesian gains: a planted foot tracks no position, only the force.
    Eigen::Vector3d tau;
    T_CHECK(lc.computeCartesianForceCommand(q, zero, zero, zero, zero, zero, f_foot_body,
                                            tau));
    for (int j = 0; j < 3; j++) {
      tau_ff[3 * leg + j] = tau[j];
      if (std::fabs(tau[j]) > 1e-9) anyTorque = true;
    }
    lc.uninit();
  }

  if (!anyTorque) {
    // Not a separate failure mode from the comparison below, but worth saying
    // plainly: with a zero torque the two runs are the same experiment.
    fprintf(stderr,
            "NOTE: the Cartesian force helper returned zero torque, so the sign "
            "comparison below cannot distinguish anything.\n");
  }

  const double duration = 0.25;  // [s]
  const double zUp = runStance(m, d, tau_ff, +1.0, duration);
  const double zDown = runStance(m, d, tau_ff, -1.0, duration);

  printf("start height %.4f m, after %.2f s: intended sign %.4f m, flipped %.4f m\n",
         kStartHeight, duration, zUp, zDown);

  // The intended sign holds the robot up...
  T_CHECK(zUp > 0.25);
  // ...and the flipped one does not, by a margin no plausible tuning covers.
  //
  // For scale: with the intended sign the robot finishes at about 0.306 m,
  // essentially where it started; with the sign flipped it ends at about
  // 0.094 m, which is the belly on the floor. The thresholds sit in the wide
  // gap between those, so they are not measuring anything delicate. The
  // uncommanded torque case lands at 0.222 m, well below the first check,
  // which is why a stubbed force helper reports a failure here rather than
  // quietly passing.
  T_CHECK(zUp - zDown > 0.05);

  mj_deleteData(d);
  mj_deleteModel(m);
  printf("test_stance_force_sign: all checks passed\n");
  return 0;
}
