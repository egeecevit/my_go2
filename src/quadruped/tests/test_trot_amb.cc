/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// AMB inside the trot (TrotAMB) and the support-pair parameter of MdlAMB's
// projection:
//   (i)   the FR-RL diagonal is the mirror image of FL-RR: MdlAMB::project and
//         robotMomentum on a state with pair FL_RR equal the same calls on the
//         mirrored state with pair RL_FR, which is what lets one law, one
//         template and one sign convention serve both diagonals;
//   (ii)  TrotAMB::rotateReference: the velocity and acceleration it returns
//         are the time derivatives of the position it returns;
//   (iii) TrotAMB::tick over a swing: idle outside a two-foot phase and in a
//         phase shorter than min_swing, the offset back at zero at touchdown,
//         state reset on a pair switch, and the pass-through outputs.

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlAMB.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"
#include "quadruped/TrotAMB.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

// Linking MdlAMB.o drags in MdlLegControl.o, which references the hardware
// singletons; none is instantiated here (test_amb_template.cc does the same).
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

static const char* g_case = "(none)";

// NDEBUG-proof check: the build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                             \
  do {                                                                            \
    if (!(cond)) {                                                                \
      std::fprintf(stderr, "FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, \
                   #cond);                                                        \
      std::exit(1);                                                               \
    }                                                                             \
  } while (0)

static void report(const char* what, double worst, double tol) {
  std::printf("  %-66s worst %.2e (tol %.0e)\n", what, worst, tol);
  if (!(worst <= tol)) {
    std::fprintf(stderr, "FAIL [%s] %s: %.3e > %.0e\n", g_case, what, worst, tol);
    std::exit(1);
  }
}

static std::mt19937 g_rng(20261127);

static double uni(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(g_rng);
}

static Eigen::Vector3d uni3(double lo, double hi) {
  return Eigen::Vector3d(uni(lo, hi), uni(lo, hi), uni(lo, hi));
}

static Eigen::Matrix3d rpy(double roll, double pitch, double yaw) {
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

// trot.toml's nominal footprint: hip + [0, +-0.10, -0.28], body frame.
static Eigen::Vector3d nominalFoot(const QuadrupedKinematics& kin, int leg) {
  Eigen::Vector3d p = kin.getKinematicParams().hip_positions.row(leg).transpose();
  p += Eigen::Vector3d(0.0, leg % 2 == 0 ? 0.10 : -0.10, -0.28);
  return p;
}

// A trot two-foot phase: FL and RR near the footprint, FR and RL lifted.
static void trotState(const QuadrupedKinematics& kin, double spread, MdlAMB::input_t& in,
                      Eigen::Vector3d foot[4]) {
  for (int leg = 0; leg < 4; leg++) {
    foot[leg] = nominalFoot(kin, leg) +
                Eigen::Vector3d(uni(-spread, spread), uni(-spread, spread), 0.0);
    if (leg == 1 || leg == 2) foot[leg].z() += uni(0.03, 0.09);
    T_CHECK(kin.inverseKinematics(leg, foot[leg], in.q[leg]));
  }
}

// Left <-> right: reflection in the body's x-z plane.
static const int MIRROR_LEG[4] = {1, 0, 3, 2};
static const Eigen::Matrix3d S = Eigen::Vector3d(1.0, -1.0, 1.0).asDiagonal();

static void mirrorState(const MdlAMB::input_t& in, MdlAMB::input_t& out) {
  for (int leg = 0; leg < 4; leg++) {
    // Controller-convention joints: the abduction is positive outward on
    // both sides (joint_direction), so a mirrored leg has the same angles.
    out.q[MIRROR_LEG[leg]] = in.q[leg];
    out.qd[MIRROR_LEG[leg]] = in.qd[leg];
  }
  out.R = S * in.R * S;
  out.w_body = -(S * in.w_body);  // an axial vector
  out.t = in.t;
}

static void testMirrorPair() {
  g_case = "mirror pair";
  std::printf("(i) FL_RR on a state against RL_FR on its mirror image, 200 random states\n");
  const QuadrupedKinematics kin(createGo2Config());
  const QuadrupedLegDynamics dyn(createGo2DynamicsConfig());
  T_CHECK(dyn.isValid());
  const MdlAMB::params_t p;

  double w_foot = 0.0, w_frame = 0.0, w_theta = 0.0, w_rate = 0.0, w_c = 0.0, w_L = 0.0;
  double L_size = 0.0;
  for (int trial = 0; trial < 200; trial++) {
    MdlAMB::input_t a, b;
    Eigen::Vector3d foot[4];
    trotState(kin, 0.04, a, foot);
    for (int leg = 0; leg < 4; leg++) a.qd[leg] = uni3(-3.0, 3.0);
    a.R = rpy(uni(-0.2, 0.2), uni(-0.2, 0.2), uni(-M_PI, M_PI));
    a.w_body = uni3(-1.5, 1.5);
    a.t = 0.0;
    mirrorState(a, b);

    // The joint mirror really is the geometric one.
    for (int leg = 0; leg < 4; leg++) {
      Eigen::Vector3d pa, pb;
      T_CHECK(dyn.footPosition(leg, a.q[leg], pa));
      T_CHECK(dyn.footPosition(MIRROR_LEG[leg], b.q[MIRROR_LEG[leg]], pb));
      w_foot = std::max(w_foot, (pb - S * pa).norm());
    }

    MdlAMB::proj_t pa, pb;
    T_CHECK(MdlAMB::project(p, a, dyn, pa, MdlAMB::FL_RR));
    T_CHECK(MdlAMB::project(p, b, dyn, pb, MdlAMB::RL_FR));
    const double La =
        MdlAMB::robotMomentum(dyn, a.q, a.qd, a.R, a.w_body, pa.o, pa.s, MdlAMB::FL_RR);
    const double Lb =
        MdlAMB::robotMomentum(dyn, b.q, b.qd, b.R, b.w_body, pb.o, pb.s, MdlAMB::RL_FR);

    // The mirrored line runs the other way (RL -> FR), t_h mirrors as a polar vector.
    w_frame = std::max(w_frame, (pb.s + S * pa.s).norm());
    w_frame = std::max(w_frame, (pb.th - S * pa.th).norm());
    w_frame = std::max(w_frame, (pb.o - S * pa.o).norm());
    w_theta = std::max(w_theta, (pb.theta - pa.theta).cwiseAbs().maxCoeff());
    w_rate = std::max(w_rate, (pb.thetad - pa.thetad).cwiseAbs().maxCoeff());
    w_rate = std::max(w_rate, std::abs(pb.ch_dot - pa.ch_dot));
    w_c = std::max(w_c, std::abs(pb.c_h - pa.c_h));
    w_c = std::max(w_c, std::abs(pb.c_v - pa.c_v));
    w_L = std::max(w_L, std::abs(Lb - La));
    L_size = std::max(L_size, std::abs(La));
    // Legs hanging: theta2 near -pi, theta3 near +pi on both diagonals.
    T_CHECK(pb.theta(1) < -0.5 * M_PI && pb.theta(2) > 0.5 * M_PI);
  }
  std::printf("  largest |L| over the draws: %.3f kg m^2/s\n", L_size);
  report("mirrored foot position [m]", w_foot, 1e-12);
  report("support frame s, t_h, o", w_frame, 1e-12);
  report("theta [rad]", w_theta, 1e-9);
  // Finite-difference rates (fd_step 1e-4).
  report("theta_dot [rad/s], c_h_dot [m/s]", w_rate, 1e-6);
  // The trunk's CoM lies in the mirror plane; its inertia tensor is mirror
  // symmetric only to about 1e-3 of its size (go2.xml inertial quaternion).
  report("c_h, c_v [m]", w_c, 1e-9);
  report("L about the support line [kg m^2/s]", w_L, 5e-3);
}

static void testRotateReference() {
  g_case = "rotate reference";
  std::printf("(ii) rotated reference against finite differences, 100 random motions\n");
  double w_id = 0.0, w_v = 0.0, w_a = 0.0;
  for (int trial = 0; trial < 100; trial++) {
    const Eigen::Vector3d n = uni3(-1.0, 1.0).normalized(), h = uni3(-0.2, 0.2);
    const Eigen::Vector3d p0 = h + uni3(-0.3, 0.3), v0 = uni3(-1.0, 1.0), a0 = uni3(-5.0, 5.0);
    const double al = uni(-0.5, 0.5), ald = uni(-3.0, 3.0), aldd = uni(-30.0, 30.0);
    const auto at = [&](double t, Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d& a) {
      TrotAMB::rotateReference(n, h, al + ald * t + 0.5 * aldd * t * t, ald + aldd * t, aldd,
                               p0 + v0 * t + 0.5 * a0 * t * t, v0 + a0 * t, a0, p, v, a);
    };
    const double dt = 1e-4;
    Eigen::Vector3d pm, p, pp, vm, v, vp, am, a, ap;
    at(-dt, pm, vm, am);
    at(0.0, p, v, a);
    at(dt, pp, vp, ap);
    w_v = std::max(w_v, (v - (pp - pm) / (2.0 * dt)).norm() / (1.0 + v.norm()));
    w_a = std::max(w_a, (a - (vp - vm) / (2.0 * dt)).norm() / (1.0 + a.norm()));

    Eigen::Vector3d q, qv, qa;
    TrotAMB::rotateReference(n, h, 0.0, 0.0, 0.0, p0, v0, a0, q, qv, qa);
    w_id = std::max(w_id, (q - p0).norm() + (qv - v0).norm() + (qa - a0).norm());
  }
  report("zero angle leaves the reference", w_id, 1e-14);
  report("velocity - d(position)/dt (rel)", w_v, 1e-6);
  report("acceleration - d(velocity)/dt (rel)", w_a, 1e-6);
}

// One swing of duration T at a fixed, slightly tilted posture: the state does
// not answer the commands, which is enough to check the schedule.
struct swing_t {
  double alpha_peak = 0.0, alpha_end = 0.0, ref_end = 0.0, ref_peak = 0.0, share_peak = 0.0;
  double share_end = 0.0, dir_err = 0.0;
  int status_seen[5] = {0, 0, 0, 0, 0};
};

static swing_t runSwing(TrotAMB& amb, const QuadrupedKinematics& kin, int pairId, double T,
                        double t0) {
  TrotAMB::input_t in;
  Eigen::Vector3d foot[4];
  MdlAMB::input_t body;
  trotState(kin, 0.0, body, foot);
  if (pairId == 1) {
    MdlAMB::input_t m;
    for (int leg = 0; leg < 4; leg++) body.qd[leg].setZero();
    body.R.setIdentity();
    body.w_body.setZero();
    mirrorState(body, m);
    body = m;
    Eigen::Vector3d f[4];
    for (int leg = 0; leg < 4; leg++) f[MIRROR_LEG[leg]] = S * foot[leg];
    for (int leg = 0; leg < 4; leg++) foot[leg] = f[leg];
  }
  for (int leg = 0; leg < 4; leg++) body.qd[leg].setZero();
  // Tilted about the body's x axis: the CoM is off the support line, so the
  // law has something to answer.
  body.R = rpy(0.03, 0.0, 0.0);
  body.w_body.setZero();
  in.body = body;
  for (int leg = 0; leg < 4; leg++) {
    in.stance[leg] = pairId == 0 ? (leg == 0 || leg == 3) : (leg == 1 || leg == 2);
    in.foot_p[leg] = foot[leg];
    in.foot_v[leg].setZero();
    in.foot_a[leg].setZero();
  }
  swing_t r;
  const int n = (int)std::lround(T / 1e-3);
  for (int i = 0; i <= n; i++) {
    const double t = 1e-3 * i;
    in.body.t = t0 + t;
    for (int leg = 0; leg < 4; leg++) {
      in.swing_time[leg] = t;
      in.swing_left[leg] = T - t;
    }
    TrotAMB::output_t out;
    amb.tick(in, out);
    const double* log = amb.log();
    T_CHECK(log[0] == pairId);
    r.status_seen[(int)log[1]]++;
    double ref = 0.0;
    for (int leg = 0; leg < 4; leg++) {
      ref = std::max(ref, (out.foot_p[leg] - in.foot_p[leg]).norm() + out.foot_v[leg].norm() +
                              out.foot_a[leg].norm());
      if (in.stance[leg]) T_CHECK(out.share[leg] == 0.0 && out.foot_p[leg] == in.foot_p[leg]);
      if (out.share[leg] > 0.0) {
        r.dir_err = std::max(r.dir_err, std::abs(out.dir[leg].norm() - 1.0));
        r.dir_err = std::max(r.dir_err, out.acc[leg].cross(out.dir[leg]).norm());
      }
      r.share_peak = std::max(r.share_peak, out.share[leg]);
      if (i == n) r.share_end = std::max(r.share_end, out.share[leg]);
    }
    r.ref_peak = std::max(r.ref_peak, ref);
    r.alpha_peak = std::max(r.alpha_peak, std::max(std::abs(log[16]), std::abs(log[17])));
    if (i == n) {
      r.alpha_end = std::max(std::abs(log[16]), std::abs(log[17]));
      r.ref_end = ref;
    }
  }
  return r;
}

static void testSchedule() {
  g_case = "schedule";
  std::printf("(iii) TrotAMB over a swing at a fixed tilted posture\n");
  const QuadrupedKinematics kin(createGo2Config());
  TrotAMB amb(createGo2DynamicsConfig());
  TrotAMB::params_t p;  // ramp 0.05, return 0.15, min_swing 0.4
  amb.setParams(p);

  // Four feet down: idle, reference untouched.
  {
    TrotAMB::input_t in;
    Eigen::Vector3d foot[4];
    trotState(kin, 0.0, in.body, foot);
    for (int leg = 0; leg < 4; leg++) {
      in.body.qd[leg].setZero();
      in.foot_p[leg] = foot[leg];
      in.foot_v[leg] = uni3(-1.0, 1.0);
      in.foot_a[leg] = uni3(-1.0, 1.0);
    }
    in.body.R.setIdentity();
    in.body.w_body.setZero();
    in.body.t = 0.0;
    TrotAMB::output_t out;
    amb.tick(in, out);
    T_CHECK(amb.log()[0] == -1.0 && amb.log()[1] == 0.0);
    for (int leg = 0; leg < 4; leg++)
      T_CHECK(out.foot_p[leg] == in.foot_p[leg] && out.foot_v[leg] == in.foot_v[leg] &&
              out.foot_a[leg] == in.foot_a[leg] && out.share[leg] == 0.0);
  }

  // OFFSET, a 1 s swing on each diagonal: the law acts, then the return window
  // brings the offset back to zero by the touchdown.
  for (int pairId = 0; pairId < 2; pairId++) {
    amb.reset();
    const swing_t r = runSwing(amb, kin, pairId, 1.0, 10.0 * pairId);
    std::printf("  offset, pair %d: peak |alpha| %.4f rad, at touchdown %.2e rad, law ticks %d, "
                "return ticks %d\n",
                pairId, r.alpha_peak, r.alpha_end, r.status_seen[1], r.status_seen[2]);
    T_CHECK(r.status_seen[1] > 800 && r.status_seen[2] >= 150 && r.status_seen[3] == 0);
    T_CHECK(r.status_seen[4] == 0);
    T_CHECK(r.alpha_peak > 1e-3 && r.alpha_peak <= p.alpha_max + 1e-12);
    T_CHECK(r.alpha_end < 1e-12 && r.ref_end < 1e-9);
    T_CHECK(r.share_peak == 0.0);
  }

  // A pair switch with no four-foot tick in between still starts fresh.
  {
    amb.reset();
    TrotAMB::params_t q = p;
    q.ret = 1e-3;  // leave the offset standing at the end of the first swing
    amb.setParams(q);
    const swing_t a = runSwing(amb, kin, 0, 0.6, 0.0);
    T_CHECK(a.alpha_peak > 1e-3);
    const swing_t b = runSwing(amb, kin, 1, 0.002, 0.6);
    T_CHECK(b.alpha_peak < 1e-9);
    amb.setParams(p);
  }

  // A swing shorter than min_swing: nothing at all.
  {
    amb.reset();
    const swing_t r = runSwing(amb, kin, 0, 0.3, 0.0);
    T_CHECK(r.status_seen[3] == 301 && r.ref_peak == 0.0 && r.alpha_peak == 0.0);
  }

  // PASSTHROUGH: a unit direction, a feedforward along it, the planned
  // reference untouched, and the share back at zero at touchdown.
  {
    TrotAMB::params_t q = p;
    q.mode = TrotAMB::PASSTHROUGH;
    amb.setParams(q);
    for (int pairId = 0; pairId < 2; pairId++) {
      amb.reset();
      const swing_t r = runSwing(amb, kin, pairId, 1.0, 0.0);
      std::printf("  passthrough, pair %d: peak share %.3f, at touchdown %.2e\n", pairId,
                  r.share_peak, r.share_end);
      T_CHECK(r.ref_peak == 0.0 && r.alpha_peak == 0.0);
      T_CHECK(r.share_peak > 0.999 && r.share_peak <= 1.0 && r.share_end < 1e-9);
      T_CHECK(r.dir_err < 1e-9);
    }
  }
  std::printf("  PASS\n");
}

int main() {
  testMirrorPair();
  testRotateReference();
  testSchedule();
  std::printf("test_trot_amb: all passed\n");
  return 0;
}
