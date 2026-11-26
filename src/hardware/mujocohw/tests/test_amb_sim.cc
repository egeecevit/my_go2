/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

// Posed-state balance test for MdlAMB (plan "Tests" section 2). The Go2 model
// is put into the diagonal posture by IK with the swing feet already at their
// lifted targets, tilted rigidly about the stance-foot line, and
// MdlAMB::computeTick drives it through MdlSimDriver's MIT law one physics
// step per tick. Three runs: theta1(0) = +0.3 deg must balance, the AMB-off
// ablation must fall, theta1(0) = -0.3 deg must balance (sign symmetry).
//
// Driving computeTick directly, rather than the module through a
// ModuleManager, is what lets the test pose the robot: the full stack only
// reaches this posture through MdlDiagonalStance, whose own hand-off errors
// would be mixed into the balance question asked here.
//
// Usage: test_amb_sim <path-to-scene.xml>

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>

#include "quadruped/MdlAMB.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"

// mujocohw already carries SimHW.o's HARDWARE_IMPL(ClockHW/MotorHW/IMUHW), so
// they are not declared again here (test_sim_grf.cc).

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                 \
  do {                                                                \
    if (!(cond)) {                                                    \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                       \
    }                                                                 \
  } while (0)

// Joint sign between this code base and the MuJoCo model, per motor index
// 3*leg + joint (MdlSimDriver::_polarity).
static const double POLARITY[12] = {1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1};
static const double DEG = M_PI / 180.0;
static const double FOOT_RADIUS = 0.022;  // go2.xml foot geom
static const double BASE_Z = 0.30;

// Nominal diagonal posture, body frame: stance feet on the centreline, swing
// feet already at MdlAMB's lifted targets (amb.toml stance_x / base_z /
// swing_rel; the same triples test_amb_template.cc pins the IK of).
static const Eigen::Vector3d FOOT_NOM[4] = {{0.15, 0.0, -0.278},
                                            {0.2634, -0.142, -0.18},
                                            {-0.2634, 0.142, -0.18},
                                            {-0.15, 0.0, -0.278}};

struct rig_t {
  mjModel *m = nullptr;
  mjData *d = nullptr;
  QuadrupedLegDynamics *dyn = nullptr;
  Eigen::Vector3d q_nom[4];
  int floor = -1, foot[4] = {-1, -1, -1, -1};
};

struct result_t {
  double theta1_posed = 0.0;   // [deg] estimated after the tilt
  double max_theta1 = 0.0;     // [deg] over t in [3, 10] s
  double theta1_at3 = 0.0;     // [deg] first sample at t >= 3 s
  double theta1_end = 0.0;     // [deg] last sample
  double ch_end = 0.0;         // [m] whole-body CoM offset from the line, last sample
  double min_z = 1.0;          // [m] base height over t in [3, 10] s
  double sat_frac = 0.0;       // ticks with any |tau| beyond its ctrlrange
  double max_cyy = 0.0;        // [m/s^2] template CoM vertical acceleration
  double t_20deg = -1.0;       // [s] first |theta1| > 20 deg, -1 if never
  double t_end = 0.0;          // [s] where the run stopped
  bool tick_ok = true;         // computeTick never returned false
  bool kd_ok = true;           // every commanded kd positive (sim rewrite guard)
  bool contact_ok = true;      // only FL/RR-vs-floor contacts ever appeared
};

// Controller-convention state from the free-joint layout (MdlSimDriver.cc:199).
static void readInput(const mjData *d, MdlAMB::input_t &in) {
  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      const int i = 3 * leg + j;
      in.q[leg](j) = POLARITY[i] * d->qpos[7 + i];
      in.qd[leg](j) = POLARITY[i] * d->qvel[6 + i];
    }
  }
  in.R = Eigen::Quaterniond(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]).toRotationMatrix();
  in.w_body = Eigen::Vector3d(d->qvel[3], d->qvel[4], d->qvel[5]);
  in.t = d->time;
}

// MdlSimDriver::_controllerCallback for one command set (without its kd <= 0
// rewrite, which the test asserts never has to fire). Returns true when any
// joint asked for more than its motor limit; MuJoCo clips it to ctrlrange.
static bool applyCommands(const mjModel *m, mjData *d, const MdlAMB::cmd_t cmd[4]) {
  bool saturated = false;
  for (int i = 0; i < 12; i++) {
    const int leg = i / 3, j = i % 3;
    const double pos = POLARITY[i] * d->actuator_length[i];
    const double vel = POLARITY[i] * d->actuator_velocity[i];
    const double tau = -cmd[leg].kp(j) * (pos - cmd[leg].a(j)) -
                       cmd[leg].kd(j) * (vel - cmd[leg].adot(j)) + cmd[leg].tau(j);
    if (tau < m->actuator_ctrlrange[2 * i] || tau > m->actuator_ctrlrange[2 * i + 1])
      saturated = true;
    d->ctrl[i] = POLARITY[i] * tau;
  }
  return saturated;
}

// Rigid rotation by phi about the line through the stance foot centres
// (posture.py _rigid_tilt): the spheres stay tangent to the floor. The axis
// is s = FL -> RR = -x, so that a positive phi reads as a positive theta1 and
// the posing iteration below contracts instead of running away.
static void tilt(rig_t &r, double phi) {
  const Eigen::Vector3d line(0.0, 0.0, FOOT_RADIUS);
  const Eigen::Vector3d rel = Eigen::Vector3d(0.0, 0.0, BASE_Z) - line;
  const double c = std::cos(phi), s = std::sin(phi);
  r.d->qpos[0] = line(0) + rel(0);
  r.d->qpos[1] = line(1) + c * rel(1) + s * rel(2);
  r.d->qpos[2] = line(2) - s * rel(1) + c * rel(2);
  r.d->qpos[3] = std::cos(0.5 * phi);
  r.d->qpos[4] = -std::sin(0.5 * phi);
  r.d->qpos[5] = r.d->qpos[6] = 0.0;
  mj_forward(r.m, r.d);
}

static int run(rig_t &r, double theta1_deg, bool enabled, double duration, result_t &res) {
  mjModel *m = r.m;
  mjData *d = r.d;
  mj_resetData(m, d);
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++) d->qpos[7 + 3 * leg + j] = POLARITY[3 * leg + j] * r.q_nom[leg](j);

  // The estimated theta1 is not exactly the rigid roll (the contact points
  // move with the tilt), so the tilt is iterated until the estimate hits the
  // target, as posture.py does with brentq.
  MdlAMB::params_t p;
  p.enabled = enabled;
  // The swing feet are already at p_sw1, so the lift (and the x3 ramp) is
  // over after one tick.
  p.lift_duration = 1e-3;
  MdlAMB::input_t in;
  MdlAMB::proj_t pr;
  double phi = theta1_deg * DEG;
  for (int it = 0; it < 4; it++) {
    tilt(r, phi);
    readInput(d, in);
    T_CHECK(MdlAMB::project(p, in, *r.dyn, pr));
    phi += theta1_deg * DEG - pr.theta(0);
  }
  res.theta1_posed = pr.theta(0) / DEG;

  // A few ms of a plain joint hold (legcontrol's default 100 / 5 gains) lets
  // the soft contacts seat before the first tick latches the height.
  for (int s = 0; s < 5; s++) {
    for (int i = 0; i < 12; i++) {
      const double pos = POLARITY[i] * d->actuator_length[i];
      const double vel = POLARITY[i] * d->actuator_velocity[i];
      d->ctrl[i] = POLARITY[i] * (-100.0 * (pos - r.q_nom[i / 3](i % 3)) - 5.0 * vel);
    }
    mj_step(m, d);
  }

  MdlAMB amb;
  amb.init();
  amb.setParams(p);
  int ticks = 0, sat_ticks = 0;
  const double t_start = d->time;
  while (d->time - t_start < duration) {
    readInput(d, in);
    MdlAMB::cmd_t cmd[4];
    if (!amb.computeTick(in, cmd)) {
      res.tick_ok = false;
      break;
    }
    for (int leg = 0; leg < 4; leg++)
      if ((cmd[leg].kd.array() <= 0.0).any() || (cmd[leg].kp.array() < 0.0).any()) res.kd_ok = false;
    if (applyCommands(m, d, cmd)) sat_ticks++;
    ticks++;
    mj_step(m, d);

    const double *log = amb.logState();
    const double theta1 = log[0] / DEG, t = d->time - t_start;
    if (std::abs(theta1) > 20.0 && res.t_20deg < 0.0) res.t_20deg = t;
    if (t >= 3.0) {
      if (res.theta1_at3 == 0.0) res.theta1_at3 = theta1;
      res.max_theta1 = std::max(res.max_theta1, std::abs(theta1));
      res.min_z = std::min(res.min_z, d->qpos[2]);
    }
    res.theta1_end = theta1;
    res.ch_end = log[7];
    res.max_cyy = std::max(res.max_cyy, std::abs(log[17]));
    for (int c = 0; c < d->ncon; c++) {
      const mjContact &con = d->contact[c];
      if (con.exclude != 0) continue;
      const bool stance = (con.geom1 == r.foot[0] || con.geom1 == r.foot[3] ||
                           con.geom2 == r.foot[0] || con.geom2 == r.foot[3]);
      const bool floor = (con.geom1 == r.floor || con.geom2 == r.floor);
      if (!(stance && floor)) res.contact_ok = false;
    }
  }
  res.t_end = d->time - t_start;
  res.sat_frac = ticks > 0 ? (double)sat_ticks / ticks : 0.0;
  amb.uninit();

  printf("run theta1(0)=%+.1f deg enabled=%d: posed theta1 %.3f deg, stopped at %.3f s, "
         "max|theta1| over [3,10] s = %.3f deg (%.3f at 3 s, %.3f at the end), c_h at the end = "
         "%.2f mm, min base z = %.4f m, saturation %.2f%%, max|cy_ddot| = %.3f m/s^2, "
         "|theta1| > 20 deg at %.3f s, tick_ok=%d kd_ok=%d contact_ok=%d\n",
         theta1_deg, enabled, res.theta1_posed, res.t_end, res.max_theta1, res.theta1_at3,
         res.theta1_end, 1e3 * res.ch_end, res.min_z, 100.0 * res.sat_frac, res.max_cyy,
         res.t_20deg, res.tick_ok, res.kd_ok, res.contact_ok);
  return 0;
}

// The law regulates the whole-body CoM over the support line (c_h), not
// theta1: at rest the swing legs hold an asymmetric x2 and the {trunk, stance
// legs} CoM angle theta1 sits a fraction of a degree off vertical and creeps
// with x2, which nothing regulates (the Python reference settles at x1 = 0.69
// deg from +0.3 deg; this rig gives theta1 = +0.44 .. +0.50 and -0.5 deg). So
// "balanced" is the regulated c_h within 2 mm at the end plus a 1 deg bound on
// theta1, not the 0.5 deg the posed tilt started under.
static int checkBalanced(const result_t &res) {
  T_CHECK(res.tick_ok);
  T_CHECK(res.kd_ok);
  T_CHECK(res.max_theta1 < 1.0);
  T_CHECK(std::abs(res.ch_end) < 2e-3);
  T_CHECK(res.min_z > 0.25);
  T_CHECK(res.contact_ok);
  T_CHECK(res.sat_frac < 0.05);
  return 0;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <path-to-scene.xml>\n", argv[0]);
    return 1;
  }

  rig_t r;
  char error[1024] = "";
  r.m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!r.m) {
    fprintf(stderr, "FAIL: could not load %s: %s\n", argv[1], error);
    return 1;
  }
  r.d = mj_makeData(r.m);
  T_CHECK(r.d != nullptr);
  T_CHECK(r.m->nu == 12);
  // One physics step per 1 kHz controller tick, as MdlSimDriver runs it.
  r.m->opt.timestep = 0.001;

  r.floor = mj_name2id(r.m, mjOBJ_GEOM, "floor");
  const char *footNames[4] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; i++) {
    r.foot[i] = mj_name2id(r.m, mjOBJ_GEOM, footNames[i]);
    T_CHECK(r.foot[i] >= 0);
  }
  T_CHECK(r.floor >= 0);

  QuadrupedKinematics kin(createGo2Config());
  for (int leg = 0; leg < 4; leg++) T_CHECK(kin.inverseKinematics(leg, FOOT_NOM[leg], r.q_nom[leg]));
  QuadrupedLegDynamics dyn(createGo2DynamicsConfig());
  T_CHECK(dyn.isValid());
  r.dyn = &dyn;

  // 1. +0.3 deg balances.
  result_t pos;
  if (run(r, 0.3, true, 10.0, pos) || checkBalanced(pos)) return 1;

  // 2. Same lift with the swing rolls PD-held falls past 20 deg before 1.5 s:
  // this is what shows the balance above is AMB's doing, not the stance PD's.
  result_t off;
  if (run(r, 0.3, false, 1.5, off)) return 1;
  T_CHECK(off.t_20deg >= 0.0 && off.t_20deg < 1.5);

  // 3. -0.3 deg balances (sign symmetry of the whole chain).
  result_t neg;
  if (run(r, -0.3, true, 10.0, neg) || checkBalanced(neg)) return 1;

  mj_deleteData(r.d);
  mj_deleteModel(r.m);
  printf("test_amb_sim: all checks passed\n");
  return 0;
}
