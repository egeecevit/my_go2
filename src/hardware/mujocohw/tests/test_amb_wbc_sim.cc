/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

// Whole-body realization of MdlAMB (thesis Ch. 6) in MuJoCo, rig of
// test_amb_sim.cc. The Go2 starts on four feet in the posture
// MdlDiagonalStance hands over (FL, RR on the centreline, FR, RL where the
// stand left them), and a bare MdlAMB in WBC mode, with a bare MdlConvexMPC
// for its stance forces, lifts FR and RL and balances through MdlSimDriver's
// MIT law, one physics step per tick. Two runs, the Sec. 6.6.4 ablation:
// AMB on must balance 10 s, AMB off (same lift, tasks, forces and WBC, no
// tangential feedforward) must fall.
//
// The simulator also supplies what the controller cannot see, which is
// reported: the base velocity it derives from the planted stance feet against
// qvel, and the solved stance normal forces against the contact forces.
//
// Usage: test_amb_wbc_sim <path-to-scene.xml>

#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <cstdio>

#include "quadruped/MdlAMB.hh"
#include "quadruped/MdlConvexMPC.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"

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
static const double BASE_Z = 0.30;
// Thesis Sec. 6.6.1 fall criterion, fixed before any run.
static const double FALL_Z = 0.16;
static const double FALL_TILT = 45.0 * DEG;
// Metrics start after the lift (amb.toml lift_duration 0.4 s) and its
// transient have passed.
static const double T_METRIC = 1.0;

// Hand-off posture, body frame: FL and RR on the centreline one foot radius
// above the ground at base_z (MdlDiagonalStance CENTRE), FR and RL 6.5 cm
// ahead of their hips on the ground, where the full stack's stand leaves them
// (its first MdlAMB_wbc sample has FR, RL at xi_n = -0.066, -0.070 m).
static const Eigen::Vector3d FOOT_HANDOFF[4] = {{0.15, 0.0, -0.278},
                                                {0.2584, -0.142, -0.278},
                                                {-0.1284, 0.142, -0.278},
                                                {-0.15, 0.0, -0.278}};

struct rig_t {
  mjModel *m = nullptr;
  mjData *d = nullptr;
  Eigen::Vector3d q0[4];
  int floor = -1, foot[4] = {-1, -1, -1, -1};
};

struct result_t {
  double t_end = 0.0;          // [s] where the run stopped
  double t_fall = -1.0;        // [s] fall criterion first met, -1 if never
  double t_tick_fail = -1.0;   // [s] first failed computeTick, -1 if never
  double max_theta1 = 0.0;     // [deg] after T_METRIC
  double min_z = 1.0;          // [m] base height after T_METRIC
  double sat_frac = 0.0;       // ticks with any |tau| beyond its ctrlrange
  double ff_limited = 0.0;     // swing-leg ticks with the feedforward norm-limited
  double ff_off = 0.0;         // swing-leg ticks without feedforward (rejected or AMB off)
  // [s] wall time of computeTick, on the ticks that re-solve the MPC and the others
  double tick_mpc_mean = 0.0, tick_mpc_max = 0.0, tick_mean = 0.0, tick_max = 0.0;
  // [m/s] stance-kinematic base velocity vs qvel over the whole run, lift
  // included, and the true speed for scale
  double vb_rms = 0.0, vb_max = 0.0, v_rms = 0.0, v_max = 0.0;
  double fz_rms[2] = {0.0, 0.0};  // [N] solved vs simulated normal force, FL, RR, after T_METRIC
  double fz_sim[2] = {0.0, 0.0}, fz_wbc[2] = {0.0, 0.0};  // [N] their means
  bool kd_ok = true;           // every commanded kd positive (sim rewrite guard)
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

// World-frame contact force on each foot (MdlSimDriver::getFootGRF).
static void footForces(const rig_t &r, Eigen::Vector3d f[4]) {
  for (int i = 0; i < 4; i++) f[i].setZero();
  for (int c = 0; c < r.d->ncon; c++) {
    const mjContact &con = r.d->contact[c];
    if (con.exclude != 0) continue;
    for (int i = 0; i < 4; i++) {
      // The wrench acts on geom2; reverse its sign when the foot is geom1.
      double sign;
      if (con.geom2 == r.foot[i]) sign = 1.0;
      else if (con.geom1 == r.foot[i]) sign = -1.0;
      else continue;
      mjtNum wrench[6], fw[3];
      mj_contactForce(r.m, r.d, c, wrench);
      mju_mulMatTVec(fw, con.frame, wrench, 3, 3);
      f[i] += sign * Eigen::Vector3d(fw[0], fw[1], fw[2]);
    }
  }
}

// [mpc] of config/default/mpc.toml, which a bare solver never reads.
static MdlConvexMPC::params_t mpcParams() {
  MdlConvexMPC::params_t p;
  p.dt = 0.01;
  p.horizon = 15;
  p.solve_period = 0.01;
  p.mass = 15.2064;
  p.com_offset_body = Eigen::Vector3d(-0.00172, 0.0, -0.02227);
  p.inertia_body << 0.177718, 0.000122, -0.016809, 0.000122, 0.488065, -0.000031, -0.016809,
      -0.000031, 0.535074;
  p.friction = 0.6;
  p.force_min = 0.0;
  p.force_max = 200.0;
  return p;
}

static int run(rig_t &r, bool enabled, double duration, result_t &res) {
  mjModel *m = r.m;
  mjData *d = r.d;
  mj_resetData(m, d);
  d->qpos[2] = BASE_Z;
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++) d->qpos[7 + 3 * leg + j] = POLARITY[3 * leg + j] * r.q0[leg](j);
  mj_forward(m, d);
  // A plain joint hold (legcontrol's default 100 / 5 gains) lets the soft
  // contacts seat before the first tick latches the references. Dropped on
  // four feet the stance feet still sink at up to 4 cm/s for some 60 ms, which
  // the stance-kinematic base velocity cannot see; the full stack hands over
  // from a settled stance.
  for (int s = 0; s < 100; s++) {
    for (int i = 0; i < 12; i++) {
      const double pos = POLARITY[i] * d->actuator_length[i];
      const double vel = POLARITY[i] * d->actuator_velocity[i];
      d->ctrl[i] = POLARITY[i] * (-100.0 * (pos - r.q0[i / 3](i % 3)) - 5.0 * vel);
    }
    mj_step(m, d);
  }

  MdlConvexMPC mpc;
  mpc.setParams(mpcParams());
  T_CHECK(mpc.reset());
  MdlAMB amb;
  amb.init();
  MdlAMB::params_t p;
  p.wbc = true;
  p.enabled = enabled;
  // amb.toml's fore offset (the header default 0.07 lets the RL foot land on
  // the RR calf during recoveries).
  p.swing_rel(0) = 0.18;
  amb.setParams(p);
  amb.setSolver(&mpc);
  // The MPC re-solves every solve_period, on the first tick and every
  // mpc_every-th after it.
  const int mpc_every = (int)std::lround(mpcParams().solve_period / m->opt.timestep);

  MdlAMB::cmd_t cmd[4];
  int ticks = 0, mpc_ticks = 0, sat_ticks = 0, n_run = 0, n_metric = 0, ff_lim = 0, ff_off = 0;
  double vb_sq = 0.0, v_sq = 0.0, fz_sq[2] = {0.0, 0.0};
  const double t_start = d->time;
  while (d->time - t_start < duration) {
    MdlAMB::input_t in;
    readInput(d, in);
    const Eigen::Vector3d v_true(d->qvel[0], d->qvel[1], d->qvel[2]);
    const double t = d->time - t_start;
    const bool commanding = res.t_tick_fail < 0.0;
    if (commanding) {
      const auto t0 = std::chrono::steady_clock::now();
      const bool ok = amb.computeTick(in, cmd);
      const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (ticks % mpc_every == 0) {
        res.tick_mpc_mean += dt;
        res.tick_mpc_max = std::max(res.tick_mpc_max, dt);
        mpc_ticks++;
      } else {
        res.tick_mean += dt;
        res.tick_max = std::max(res.tick_max, dt);
      }
      ticks++;
      // A failed tick leaves the motors on their last command, as MotorHW
      // does, so the fall criterion still decides what happened to the robot.
      if (!ok) res.t_tick_fail = t;
    }
    for (int leg = 0; leg < 4; leg++)
      if ((cmd[leg].kd.array() <= 0.0).any() || (cmd[leg].kp.array() < 0.0).any()) res.kd_ok = false;
    if (applyCommands(m, d, cmd) && commanding) sat_ticks++;
    mj_step(m, d);

    const double *st = amb.logState(), *wb = amb.logWbc();
    if (commanding && res.t_tick_fail < 0.0) {
      n_run++;
      const Eigen::Vector3d e = Eigen::Vector3d(wb[35], wb[36], wb[37]) - v_true;
      vb_sq += e.squaredNorm();
      v_sq += v_true.squaredNorm();
      res.vb_max = std::max(res.vb_max, e.norm());
      res.v_max = std::max(res.v_max, v_true.norm());
      ff_lim += (wb[20] == 1.0) + (wb[27] == 1.0);
      ff_off += (wb[20] == 2.0) + (wb[27] == 2.0);
      if (t >= T_METRIC) {
        n_metric++;
        res.max_theta1 = std::max(res.max_theta1, std::abs(st[0]) / DEG);
        res.min_z = std::min(res.min_z, d->qpos[2]);
        Eigen::Vector3d f[4];
        footForces(r, f);
        for (int i = 0; i < 2; i++) {
          const double fz_sim = f[i == 0 ? 0 : 3].z(), fz_wbc = wb[10 + 3 * i];
          fz_sq[i] += (fz_sim - fz_wbc) * (fz_sim - fz_wbc);
          res.fz_sim[i] += fz_sim;
          res.fz_wbc[i] += fz_wbc;
        }
      }
    }
    const Eigen::Matrix3d Rb =
        Eigen::Quaterniond(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]).toRotationMatrix();
    const double roll = std::atan2(Rb(2, 1), Rb(2, 2)), pitch = std::asin(-Rb(2, 0));
    if (d->qpos[2] < FALL_Z || std::abs(roll) > FALL_TILT || std::abs(pitch) > FALL_TILT) {
      res.t_fall = d->time - t_start;
      break;
    }
  }
  res.t_end = d->time - t_start;
  res.sat_frac = ticks > 0 ? (double)sat_ticks / ticks : 0.0;
  res.tick_mpc_mean = mpc_ticks > 0 ? res.tick_mpc_mean / mpc_ticks : 0.0;
  res.tick_mean = ticks > mpc_ticks ? res.tick_mean / (ticks - mpc_ticks) : 0.0;
  if (n_run > 0) {
    res.vb_rms = std::sqrt(vb_sq / n_run);
    res.v_rms = std::sqrt(v_sq / n_run);
    res.ff_limited = ff_lim / (2.0 * n_run);
    res.ff_off = ff_off / (2.0 * n_run);
  }
  // A run that ended before T_METRIC has none of these.
  if (n_metric == 0) res.max_theta1 = res.min_z = NAN;
  for (int i = 0; i < 2; i++) {
    res.fz_rms[i] = n_metric > 0 ? std::sqrt(fz_sq[i] / n_metric) : NAN;
    res.fz_sim[i] = n_metric > 0 ? res.fz_sim[i] / n_metric : NAN;
    res.fz_wbc[i] = n_metric > 0 ? res.fz_wbc[i] / n_metric : NAN;
  }
  amb.uninit();

  printf("run enabled=%d: stopped at %.3f s, fall at %.3f s, tick failed at %.3f s\n"
         "  after %.1f s: max|theta1| %.3f deg, min base z %.4f m\n"
         "  torque saturation %.2f%% of ticks; feedforward norm-limited on %.2f%%, absent on "
         "%.2f%% of swing-leg ticks\n"
         "  computeTick: MPC ticks mean %.1f us, max %.1f us; other ticks mean %.1f us, max %.1f "
         "us\n"
         "  base velocity from the stance feet vs qvel: error rms %.5f m/s, max %.5f m/s "
         "(true speed rms %.4f, max %.4f)\n"
         "  after %.1f s, normal force simulated / solved: FL %.2f / %.2f N (rms diff %.3f), "
         "RR %.2f / %.2f N (rms diff %.3f); kd_ok=%d\n",
         enabled, res.t_end, res.t_fall, res.t_tick_fail, T_METRIC, res.max_theta1, res.min_z,
         100.0 * res.sat_frac, 100.0 * res.ff_limited, 100.0 * res.ff_off,
         1e6 * res.tick_mpc_mean, 1e6 * res.tick_mpc_max, 1e6 * res.tick_mean, 1e6 * res.tick_max,
         res.vb_rms, res.vb_max, res.v_rms, res.v_max, T_METRIC, res.fz_sim[0], res.fz_wbc[0],
         res.fz_rms[0], res.fz_sim[1], res.fz_wbc[1], res.fz_rms[1], res.kd_ok);
  return 0;
}

// Appended: the experiment inputs of thesis Sec. 6.6 (Table 6.3) on the same
// rig. One whole-body run with params p, sampling every tick what the checks
// need: x3 and its reference, the body ZYX angles and the orientation task's
// reference R_d = exp(e) R (e = MdlAMB_wbc 31-33), the support-line momentum
// L, and the orientation error about the support line (FL -> RR feet).
struct series_t {
  std::vector<double> t, x3, x3d, L, e_s, rpy[3], rpy_d[3];
  double t_fall = -1.0, t_tick_fail = -1.0;
  bool ff_ok = true;  // the AMB feedforward reached the WBC on every tick
};

static Eigen::Vector3d zyx(const Eigen::Matrix3d &R) {
  return Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)), std::asin(-R(2, 0)),
                         std::atan2(R(1, 0), R(0, 0)));
}

static int runInputs(rig_t &r, const MdlAMB::params_t &p, double duration, series_t &s) {
  mjModel *m = r.m;
  mjData *d = r.d;
  mj_resetData(m, d);
  d->qpos[2] = BASE_Z;
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++) d->qpos[7 + 3 * leg + j] = POLARITY[3 * leg + j] * r.q0[leg](j);
  mj_forward(m, d);
  // Same seating hold as run().
  for (int k = 0; k < 100; k++) {
    for (int i = 0; i < 12; i++) {
      const double pos = POLARITY[i] * d->actuator_length[i];
      const double vel = POLARITY[i] * d->actuator_velocity[i];
      d->ctrl[i] = POLARITY[i] * (-100.0 * (pos - r.q0[i / 3](i % 3)) - 5.0 * vel);
    }
    mj_step(m, d);
  }
  MdlConvexMPC mpc;
  mpc.setParams(mpcParams());
  T_CHECK(mpc.reset());
  MdlAMB amb;
  amb.init();
  amb.setParams(p);
  amb.setSolver(&mpc);

  MdlAMB::cmd_t cmd[4];
  const double t_start = d->time;
  while (d->time - t_start < duration) {
    MdlAMB::input_t in;
    readInput(d, in);
    if (s.t_tick_fail < 0.0 && !amb.computeTick(in, cmd)) s.t_tick_fail = d->time - t_start;
    applyCommands(m, d, cmd);
    mj_step(m, d);
    const double *st = amb.logState(), *wb = amb.logWbc();
    const Eigen::Vector3d e(wb[31], wb[32], wb[33]);
    const Eigen::Vector3d sl = (Eigen::Vector3d(d->geom_xpos + 3 * r.foot[3]) -
                                Eigen::Vector3d(d->geom_xpos + 3 * r.foot[0])).normalized();
    const Eigen::Vector3d a = zyx(in.R);
    const Eigen::Vector3d a_d = zyx(Eigen::AngleAxisd(e.norm(), e.normalized()) * in.R);
    s.t.push_back(in.t - t_start);
    s.x3.push_back(st[1] - st[2]);
    s.x3d.push_back(st[23]);
    s.L.push_back(st[9]);
    s.e_s.push_back(e.dot(sl));
    for (int i = 0; i < 3; i++) {
      s.rpy[i].push_back(a(i));
      s.rpy_d[i].push_back(a_d(i));
    }
    if (wb[20] == 2.0 || wb[27] == 2.0) s.ff_ok = false;
    if (d->qpos[2] < FALL_Z || std::abs(a(0)) > FALL_TILT || std::abs(a(1)) > FALL_TILT) {
      s.t_fall = d->time - t_start;
      break;
    }
  }
  amb.uninit();
  return 0;
}

// Least-squares fit y = a sin(w (t - 2)) + b cos(w (t - 2)) + c over t >= t0,
// w the sinusoids' 0.1 Hz, phase origin at their 2 s delay: (a, b).
static Eigen::Vector2d fitSinusoid(const series_t &s, const std::vector<double> &y, double t0) {
  const double w = 2.0 * M_PI * 0.1;
  Eigen::Matrix3d AtA = Eigen::Matrix3d::Zero();
  Eigen::Vector3d Aty = Eigen::Vector3d::Zero();
  for (size_t k = 0; k < s.t.size(); k++) {
    if (s.t[k] < t0) continue;
    const Eigen::Vector3d row(std::sin(w * (s.t[k] - 2.0)), std::cos(w * (s.t[k] - 2.0)), 1.0);
    AtA += row * row.transpose();
    Aty += row * y[k];
  }
  return AtA.ldlt().solve(Aty).head<2>();
}

// Largest |v| over t >= t0.
static double maxAbs(const series_t &s, const std::vector<double> &v, double t0) {
  double m = 0.0;
  for (size_t k = 0; k < s.t.size(); k++)
    if (s.t[k] >= t0) m = std::max(m, std::abs(v[k]));
  return m;
}

// Appended: thesis Sec. 6.6.7 on the same rig. Balance t_stop seconds, call
// stopBalancing(), and run until SETTLED plus a second of the hold that
// follows, recording what Table 6.6 and the fall criterion need.
struct stop_t {
  double t_place = -1.0, t_transfer = -1.0, t_hold = -1.0, t_settled = -1.0;  // [s], -1 never
  double t_fall = -1.0, t_tick_fail = -1.0;
  bool stop_ok = false, kd_ok = true;
  // Normal force RMS differences per foot (FL FR RL RR) [N], in the transfer
  // (k = 0) and the final hold up to SETTLED (k = 1): desired - solved,
  // solved - simulated, desired - simulated; and the means.
  double rms_ds[2][4] = {}, rms_sm[2][4] = {}, rms_dm[2][4] = {};
  double mean_d[2][4] = {}, mean_s[2][4] = {}, mean_m[2][4] = {};
  // FR, RL from their promotion to SETTLED: simulated normal force on the
  // promotion tick [N] (a foot promoted before it touches reads zero), the
  // largest solved minus simulated normal force [N] (a contact asked to push
  // while not bearing that load), and the largest kinematic height [m].
  double fz_promo[2] = {-1.0, -1.0}, push_max[2] = {-INFINITY, -INFINITY};
  double h_max[2] = {-INFINITY, -INFINITY};
  double fz_end[4] = {};          // [N] simulated, last tick
  double pos_rms_final = 0.0;     // [m] body position error, final hold
  double xn_rms[2] = {}, rho_rms[2] = {};  // [m] FR, RL normal / radial error, two-foot part
  double tick_max_4 = 0.0;        // [s] computeTick from the transfer on (four-foot MPC)
};

static int runStop(rig_t &r, const MdlAMB::params_t &p, double t_stop, stop_t &o) {
  mjModel *m = r.m;
  mjData *d = r.d;
  mj_resetData(m, d);
  d->qpos[2] = BASE_Z;
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++) d->qpos[7 + 3 * leg + j] = POLARITY[3 * leg + j] * r.q0[leg](j);
  mj_forward(m, d);
  // Same seating hold as run().
  for (int k = 0; k < 100; k++) {
    for (int i = 0; i < 12; i++) {
      const double pos = POLARITY[i] * d->actuator_length[i];
      const double vel = POLARITY[i] * d->actuator_velocity[i];
      d->ctrl[i] = POLARITY[i] * (-100.0 * (pos - r.q0[i / 3](i % 3)) - 5.0 * vel);
    }
    mj_step(m, d);
  }
  MdlConvexMPC mpc;
  mpc.setParams(mpcParams());
  T_CHECK(mpc.reset());
  MdlAMB amb;
  amb.init();
  amb.setParams(p);
  amb.setSolver(&mpc);
  // Desired and solved force slots of MdlAMB_wbc per leg.
  const int DES[4] = {2, 38, 41, 5}, SOL[4] = {8, 44, 47, 11};

  MdlAMB::cmd_t cmd[4];
  int n[2] = {0, 0}, n2[2] = {0, 0}, n_pos = 0;
  const double t_start = d->time;
  // Placement, transfer and hold take at most 2.5 + 3 + 3 s after the stop.
  while (d->time - t_start < t_stop + 10.0) {
    const double t = d->time - t_start;
    if (t >= t_stop && !o.stop_ok) o.stop_ok = amb.stopBalancing();
    MdlAMB::input_t in;
    readInput(d, in);
    if (o.t_tick_fail < 0.0) {
      const auto c0 = std::chrono::steady_clock::now();
      if (!amb.computeTick(in, cmd)) o.t_tick_fail = t;
      const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - c0).count();
      // The promotion tick moves the MPC's solve lattice, so every tick counts.
      if (o.t_transfer >= 0.0) o.tick_max_4 = std::max(o.tick_max_4, dt);
    }
    for (int leg = 0; leg < 4; leg++)
      if ((cmd[leg].kd.array() <= 0.0).any() || (cmd[leg].kp.array() < 0.0).any()) o.kd_ok = false;
    applyCommands(m, d, cmd);
    mj_step(m, d);

    const double *wb = amb.logWbc();
    const int phase = (int)wb[0];
    if (phase == 2 && o.t_place < 0.0) o.t_place = t;
    if (phase == 3 && o.t_transfer < 0.0) o.t_transfer = t;
    if (phase == 4 && o.t_hold < 0.0) o.t_hold = t;
    if (amb.getStatus() == MdlAMB::SETTLED && o.t_settled < 0.0) o.t_settled = t;
    Eigen::Vector3d f[4];
    footForces(r, f);
    // Principal-frame errors of the swing feet before they are contacts:
    // after the lift and its transient (T_METRIC), balance and placement.
    for (int k = 0; k < 2; k++) {
      const double *lg = wb + 14 + 7 * k;
      if (lg[6] == 3.0) {
        if (o.fz_promo[k] < 0.0) o.fz_promo[k] = f[k + 1].z();
        if (o.t_settled < 0.0) {
          o.push_max[k] = std::max(o.push_max[k], wb[SOL[k + 1] + 2] - f[k + 1].z());
          o.h_max[k] = std::max(o.h_max[k], wb[50 + k]);
        }
      } else if (t >= T_METRIC) {
        o.xn_rms[k] += (lg[0] - lg[3]) * (lg[0] - lg[3]);
        o.rho_rms[k] += (lg[1] - lg[4]) * (lg[1] - lg[4]);
        n2[k]++;
      }
    }
    const int iv = (phase == 3) ? 0 : (phase == 4 && o.t_settled < 0.0) ? 1 : -1;
    if (iv >= 0 && o.t_tick_fail < 0.0) {
      n[iv]++;
      for (int leg = 0; leg < 4; leg++) {
        const double fd = wb[DES[leg] + 2], fs = wb[SOL[leg] + 2], fm = f[leg].z();
        o.rms_ds[iv][leg] += (fd - fs) * (fd - fs);
        o.rms_sm[iv][leg] += (fs - fm) * (fs - fm);
        o.rms_dm[iv][leg] += (fd - fm) * (fd - fm);
        o.mean_d[iv][leg] += fd;
        o.mean_s[iv][leg] += fs;
        o.mean_m[iv][leg] += fm;
      }
      if (iv == 1) {
        o.pos_rms_final += Eigen::Vector3d(wb[28], wb[29], wb[30]).squaredNorm();
        n_pos++;
      }
    }
    for (int leg = 0; leg < 4; leg++) o.fz_end[leg] = f[leg].z();
    const Eigen::Vector3d a = zyx(in.R);
    if (d->qpos[2] < FALL_Z || std::abs(a(0)) > FALL_TILT || std::abs(a(1)) > FALL_TILT) {
      o.t_fall = t;
      break;
    }
    if (o.t_settled >= 0.0 && t >= o.t_settled + 1.0) break;
  }
  amb.uninit();
  for (int iv = 0; iv < 2; iv++) {
    for (int leg = 0; leg < 4; leg++) {
      const double nn = std::max(n[iv], 1);
      o.rms_ds[iv][leg] = std::sqrt(o.rms_ds[iv][leg] / nn);
      o.rms_sm[iv][leg] = std::sqrt(o.rms_sm[iv][leg] / nn);
      o.rms_dm[iv][leg] = std::sqrt(o.rms_dm[iv][leg] / nn);
      o.mean_d[iv][leg] /= nn;
      o.mean_s[iv][leg] /= nn;
      o.mean_m[iv][leg] /= nn;
    }
  }
  o.pos_rms_final = std::sqrt(o.pos_rms_final / std::max(n_pos, 1));
  for (int k = 0; k < 2; k++) {
    o.xn_rms[k] = std::sqrt(o.xn_rms[k] / std::max(n2[k], 1));
    o.rho_rms[k] = std::sqrt(o.rho_rms[k] / std::max(n2[k], 1));
  }
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
  for (int leg = 0; leg < 4; leg++) T_CHECK(kin.inverseKinematics(leg, FOOT_HANDOFF[leg], r.q0[leg]));

  // 1. AMB on: lift and balance 10 s, the controller never giving up.
  result_t on;
  if (run(r, true, 10.0, on)) return 1;
  T_CHECK(on.t_fall < 0.0 && on.t_end >= 10.0);
  T_CHECK(on.t_tick_fail < 0.0);
  T_CHECK(on.kd_ok);
  // The AMB feedforward reached the WBC on every tick: a frame rejected now
  // and then (Sec. 6.4.3) would otherwise pass as balance on the Cartesian
  // task alone. The legs work at rho 0.2-0.3 m, far from the 1 cm rejection.
  T_CHECK(on.ff_off == 0.0);
  // What the controller cannot see. The base velocity from the planted feet
  // is exact for rolling without slip, so what is left is the soft contacts'
  // creep, sub-mm/s here. A flipped sign or a wrong Jacobian block errs at the
  // scale of the true motion (about 7 mm/s rms here, a sign error twice that),
  // hence 5 mm/s. Taking the sphere centres as fixed instead would not show:
  // it errs by r omega, some 7% of a roll-induced speed (robotMomentum).
  // The solved normal forces are what the WBC assumed the ground supplies;
  // quasi-static, the simulated ones must agree within a few percent of the
  // 75 N per foot, while a frame or sign error shows as tens of N.
  T_CHECK(on.vb_rms < 5e-3);
  T_CHECK(on.fz_rms[0] < 5.0 && on.fz_rms[1] < 5.0);

  // 2. AMB off (Sec. 6.6.4 ablation): same lift, tasks, forces and WBC without
  // the tangential feedforward, and the robot falls.
  result_t off;
  if (run(r, false, 10.0, off)) return 1;
  T_CHECK(off.t_fall >= 0.0);
  T_CHECK(off.kd_ok);

  // Appended: Sec. 6.6 experiment inputs, AMB on, run() 1.'s parameters.
  // |L| stays near 0.002 kg m^2/s in the balance above and passes 4 in a
  // fall, so 0.02 bounds it with room either way.
  MdlAMB::params_t pe;
  pe.wbc = true;
  pe.swing_rel(0) = 0.18;
  const double L_MAX = 0.02;

  // 3. Sec. 6.6.3: one x3 step at 2 s, -304 -> -250 deg, inside what this rig's
  // hand-off reaches: steps to -340 ... -240 deg balance (at -340 the RL roll
  // already sits on its stop), -360 and -220 fall on the hip-roll stops. The
  // law's x3 channel (kp3 100, kd3 40) settles to 1 deg in 1.6 s, so the last
  // second of a 6 s run is steady.
  pe.x3_schedule = {Eigen::Vector2d(2.0, -250.0)};
  series_t sx;
  if (runInputs(r, pe, 6.0, sx)) return 1;
  double ex = 0.0;
  int nx = 0;
  for (size_t k = 0; k < sx.t.size(); k++) {
    if (sx.t[k] < 5.0) continue;
    ex += sx.x3d[k] - sx.x3[k];
    nx++;
  }
  ex /= std::max(nx, 1);
  printf("x3 step -304 -> -250 deg at 2 s: steady error %.3f deg (mean over 5-6 s), max|L| %.4f "
         "after 1 s, fall %.3f s, tick failed %.3f s\n",
         ex / DEG, maxAbs(sx, sx.L, 1.0), sx.t_fall, sx.t_tick_fail);
  T_CHECK(sx.t_fall < 0.0 && sx.t_tick_fail < 0.0 && sx.ff_ok);
  T_CHECK(std::abs(sx.x3d.back() / DEG + 250.0) < 1e-6);
  T_CHECK(std::abs(ex) < 1.0 * DEG);
  T_CHECK(maxAbs(sx, sx.L, 1.0) < L_MAX);
  pe.x3_schedule.clear();

  // 4. Sec. 6.6.5.1: 5 deg pitch and 3 deg yaw sinusoids, separate trials,
  // fitted at 0.1 Hz over one full period once the envelope is up (4-14 s).
  // The orientation task carries the reference's rate, so what lag is left
  // comes from the MPC's planned forces; 0.25 s is 9 deg of phase.
  for (int axis = 1; axis <= 2; axis++) {
    const double amp = (axis == 1) ? 5.0 : 3.0;
    pe.pitch_amp_deg = (axis == 1) ? amp : 0.0;
    pe.yaw_amp_deg = (axis == 2) ? amp : 0.0;
    series_t so;
    if (runInputs(r, pe, 14.0, so)) return 1;
    const Eigen::Vector2d zr = fitSinusoid(so, so.rpy_d[axis], 4.0);
    const Eigen::Vector2d zm = fitSinusoid(so, so.rpy[axis], 4.0);
    const double ratio = zm.norm() / zr.norm();
    const double lag = (std::atan2(zr(1), zr(0)) - std::atan2(zm(1), zm(0))) / (2.0 * M_PI * 0.1);
    double es_mean = 0.0;
    int ns = 0;
    for (size_t k = 0; k < so.t.size(); k++) {
      if (so.t[k] < 4.0) continue;
      es_mean += so.e_s[k];
      ns++;
    }
    es_mean /= std::max(ns, 1);
    printf("%s %.0f deg at 0.1 Hz: reference amplitude %.3f deg, measured %.3f deg, ratio %.3f, "
           "lag %.3f s; max|L| %.4f after 1 s; orientation error about the support line mean %.3f, "
           "max %.3f deg; fall %.3f s, tick failed %.3f s\n",
           axis == 1 ? "pitch" : "yaw", amp, zr.norm() / DEG, zm.norm() / DEG, ratio, lag,
           maxAbs(so, so.L, 1.0), es_mean / DEG, maxAbs(so, so.e_s, 4.0) / DEG, so.t_fall,
           so.t_tick_fail);
    T_CHECK(so.t_fall < 0.0 && so.t_tick_fail < 0.0 && so.ff_ok);
    T_CHECK(std::abs(zr.norm() / DEG - amp) < 0.02 * amp);
    T_CHECK(ratio > 0.9 && ratio < 1.1);
    T_CHECK(lag > 0.0 && lag < 0.25);
    T_CHECK(maxAbs(so, so.L, 1.0) < L_MAX);
  }

  // 5. Sec. 6.6.7: balance 3 s, stop, place the swing feet, promote them, move
  // the load to four feet over 3 s and hold. Gates, judged with the
  // simulator's truth: no fall and no failed tick through SETTLED; all four
  // feet loaded at the end, their sum the weight; desired, solved and
  // simulated normal forces agreeing during the transfer and the final hold.
  pe.pitch_amp_deg = pe.yaw_amp_deg = 0.0;
  stop_t st;
  if (runStop(r, pe, 3.0, st)) return 1;
  double mass = 0.0;
  for (int b = 1; b < r.m->nbody; b++) mass += r.m->body_mass[b];
  const double weight = mass * 9.81;
  const char *legName[4] = {"FL", "FR", "RL", "RR"};
  printf("stop at 3 s: placement from %.3f s, transfer from %.3f s, hold from %.3f s, SETTLED at "
         "%.3f s; fall %.3f s, tick failed %.3f s; computeTick on four feet max %.1f us\n",
         st.t_place, st.t_transfer, st.t_hold, st.t_settled, st.t_fall, st.t_tick_fail,
         1e6 * st.tick_max_4);
  for (int iv = 0; iv < 2; iv++) {
    for (int leg = 0; leg < 4; leg++)
      printf("  %s %s Fz mean desired / solved / simulated %6.2f / %6.2f / %6.2f N, RMS "
             "desired-solved %.2f, solved-simulated %.2f, desired-simulated %.2f N\n",
             iv == 0 ? "transfer" : "final   ", legName[leg], st.mean_d[iv][leg],
             st.mean_s[iv][leg], st.mean_m[iv][leg], st.rms_ds[iv][leg], st.rms_sm[iv][leg],
             st.rms_dm[iv][leg]);
  }
  printf("  end: simulated Fz %.2f %.2f %.2f %.2f N, sum %.2f N (weight %.2f N); final body "
         "position error rms %.2f mm\n  promoted FR / RL: simulated Fz on the promotion tick "
         "%.2f / %.2f N; until SETTLED, solved - simulated at most %.2f / %.2f N, height at most "
         "%.2f / %.2f mm\n  two-foot principal-frame RMS error: FR normal %.2f, radial %.2f mm; "
         "RL normal %.2f, radial %.2f mm\n",
         st.fz_end[0], st.fz_end[1], st.fz_end[2], st.fz_end[3],
         st.fz_end[0] + st.fz_end[1] + st.fz_end[2] + st.fz_end[3], weight,
         1e3 * st.pos_rms_final, st.fz_promo[0], st.fz_promo[1], st.push_max[0], st.push_max[1],
         1e3 * st.h_max[0], 1e3 * st.h_max[1], 1e3 * st.xn_rms[0], 1e3 * st.rho_rms[0],
         1e3 * st.xn_rms[1], 1e3 * st.rho_rms[1]);
  T_CHECK(st.stop_ok && st.kd_ok);
  T_CHECK(st.t_fall < 0.0 && st.t_tick_fail < 0.0);
  T_CHECK(st.t_place >= 3.0 && st.t_transfer > st.t_place && st.t_settled > st.t_transfer);
  // At rest on four feet the simulated forces carry the weight, and each foot
  // a real share of it. The lightest here is FR, about 9 N: this rig's swing
  // posture (x2 near 58 deg) lands it far out. A foot never promoted, or
  // promoted and never loaded, carries next to nothing; 5 N tells them apart.
  double fz_sum = 0.0;
  for (int leg = 0; leg < 4; leg++) {
    T_CHECK(st.fz_end[leg] > 5.0);
    fz_sum += st.fz_end[leg];
  }
  T_CHECK(std::abs(fz_sum - weight) < 0.02 * weight);
  // Table 6.6 reports desired vs solved RMS errors up to 5.9 N; the plan's
  // target is 10 N. Gated at the thesis' 5.9 N, for the simulated forces too,
  // so the ramp is followed by the robot and not only by the QP.
  for (int iv = 0; iv < 2; iv++)
    for (int leg = 0; leg < 4; leg++)
      T_CHECK(st.rms_ds[iv][leg] < 5.9 && st.rms_dm[iv][leg] < 5.9);
  // Each foot was on the ground when it became a contact (promoted by height
  // alone, they hovered and read zero here).
  T_CHECK(st.fz_promo[0] > 0.0 && st.fz_promo[1] > 0.0);

  mj_deleteData(r.d);
  mj_deleteModel(r.m);
  printf("test_amb_wbc_sim: all checks passed\n");
  return 0;
}
