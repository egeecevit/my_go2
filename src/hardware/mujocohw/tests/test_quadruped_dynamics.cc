/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

// QuadrupedDynamics against MuJoCo over random floating-base states (plan
// success criterion 1): M vs mj_fullM, h vs qfrc_bias, foot / hip / rolling
// contact Jacobians vs mj_jac and their bias terms vs mj_jacDot, COM and
// mass, and one in-air inverse-dynamics check against qfrc_inverse. The
// worst error of each family is printed so the margins are visible.
//
// Usage: test_quadruped_dynamics <path-to-scene.xml>

#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedDynamics.hh"

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                 \
  do {                                                                \
    if (!(cond)) {                                                    \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                       \
    }                                                                 \
  } while (0)

typedef QuadrupedDynamics QD;
static const int NV = QD::NV;
static const int SAMPLES = 500;
// Joint sign between this code base and the MuJoCo model, per motor index
// 3*leg + joint (MdlSimDriver::_polarity).
static const double POLARITY[12] = {1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1};
static const char *CALF[4] = {"FL_calf", "FR_calf", "RL_calf", "RR_calf"};
static const char *FOOT[4] = {"FL", "FR", "RL", "RR"};  // foot sphere geoms

// Sign of generalized coordinate k, controller vs MuJoCo (base dofs agree).
static double sign(int k) { return k < 6 ? 1.0 : POLARITY[k - 6]; }

// max |J - mj P|, mj being a row-major 3 x nv MuJoCo Jacobian.
static double jacError(const QD::Jacobian &J, const mjtNum *mj) {
  double e = 0.0;
  for (int r = 0; r < 3; r++)
    for (int k = 0; k < NV; k++) e = std::fmax(e, std::fabs(J(r, k) - mj[r * NV + k] * sign(k)));
  return e;
}

static Eigen::Vector3d mulJac(const mjtNum *mj, const mjtNum *qvel) {
  Eigen::Vector3d y = Eigen::Vector3d::Zero();
  for (int r = 0; r < 3; r++)
    for (int k = 0; k < NV; k++) y(r) += mj[r * NV + k] * qvel[k];
  return y;
}

static Eigen::Vector3d vec(const mjtNum *x) { return Eigen::Vector3d(x[0], x[1], x[2]); }

struct worst_t {
  double M = 0, h = 0, pos = 0, J_foot = 0, J_hip = 0, J_contact = 0;
  double Jdv_foot = 0, Jdv_hip = 0, Jdv_contact = 0, com = 0, inverse = 0;
};

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s <scene.xml>\n", argv[0]);
    return 2;
  }
  char error[1024] = "";
  mjModel *m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) {
    fprintf(stderr, "could not load model: %s\n", error);
    return 2;
  }
  mjData *d = mj_makeData(m);
  T_CHECK(m->nq == 7 + 12 && m->nv == NV);
  // The inverse check runs in the air: no contacts, no limit or friction-loss
  // constraint forces, so qfrc_inverse = M qacc + qfrc_bias - qfrc_passive
  // exactly. None of this changes qM, qfrc_bias or the Jacobians.
  m->opt.disableflags |= mjDSBL_CONSTRAINT;

  const int base = mj_name2id(m, mjOBJ_BODY, "base");
  int calf[4];
  for (int leg = 0; leg < 4; leg++) {
    calf[leg] = mj_name2id(m, mjOBJ_BODY, CALF[leg]);
    T_CHECK(calf[leg] >= 0);
  }
  T_CHECK(base >= 0);

  const QD::params_t prm = createGo2WholeBodyConfig();
  const QD dyn(prm);
  T_CHECK(dyn.isValid());
  const double rf = prm.foot_radius;
  // go2.xml puts the foot geom at (-0.002, 0, -0.213) in the calf frame; the
  // controller's point is (0, 0, -0.213). Known and deliberate (not this
  // model's to fix): the Jacobians are compared at the controller's point.
  const Eigen::Vector3d foot_calf = prm.legs.leg[0].foot;
  // Everything below takes rf and the foot point from prm itself, so pin both
  // to the simulated geom: the radius exactly, the centre to within the known
  // 2 mm (a fixed config gives 0, anything larger is a config error).
  double foot_offset = 0.0;
  for (int leg = 0; leg < 4; leg++) {
    const int g = mj_name2id(m, mjOBJ_GEOM, FOOT[leg]);
    T_CHECK(g >= 0 && m->geom_bodyid[g] == calf[leg]);
    T_CHECK(std::fabs(rf - m->geom_size[3 * g]) < 1e-12);
    foot_offset =
        std::fmax(foot_offset, (vec(m->geom_pos + 3 * g) - prm.legs.leg[leg].foot).norm());
  }
  T_CHECK(foot_offset < 2e-3 + 1e-12);

  std::mt19937 rng(20261125);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  std::normal_distribution<double> gauss;
  worst_t w;
  mjtNum Mmj[NV * NV], jacp[3 * NV], jacr[3 * NV], jdp[3 * NV], jdr[3 * NV];

  for (int sample = 0; sample < SAMPLES; sample++) {
    mj_resetData(m, d);
    d->qpos[2] = 1.0;  // in the air
    Eigen::Quaterniond quat(gauss(rng), gauss(rng), gauss(rng), gauss(rng));
    quat.normalize();
    d->qpos[3] = quat.w();
    d->qpos[4] = quat.x();
    d->qpos[5] = quat.y();
    d->qpos[6] = quat.z();
    for (int k = 0; k < 3; k++) d->qvel[k] = 2.0 * uni(rng);       // [m/s]
    for (int k = 3; k < 6; k++) d->qvel[k] = 4.0 * uni(rng);       // [rad/s]
    for (int i = 0; i < 12; i++) {
      const mjtNum *range = m->jnt_range + 2 * (1 + i);  // joint 0 is the free joint
      d->qpos[7 + i] = range[0] + 0.5 * (1.0 + uni(rng)) * (range[1] - range[0]);
      d->qvel[6 + i] = 10.0 * uni(rng);                              // [rad/s]
    }
    mj_forward(m, d);

    QD::state_t st;
    st.R = quat.toRotationMatrix();
    st.v_base = vec(d->qvel);
    st.w_body = vec(d->qvel + 3);
    for (int leg = 0; leg < 4; leg++)
      for (int j = 0; j < 3; j++) {
        st.q[leg](j) = POLARITY[3 * leg + j] * d->qpos[7 + 3 * leg + j];
        st.qd[leg](j) = POLARITY[3 * leg + j] * d->qvel[6 + 3 * leg + j];
      }
    QD::terms_t t;
    T_CHECK(dyn.compute(st, t));

    // Dense M one column at a time. mj_fullM's signature and mjData's inertia
    // fields differ between MuJoCo releases; mj_mulM is the same in all of them.
    for (int k = 0; k < NV; k++) {
      mjtNum e[NV] = {0};
      e[k] = 1;
      mj_mulM(m, d, Mmj + k * NV, e);  // M is symmetric: column k fills row k
    }
    for (int r = 0; r < NV; r++) {
      for (int k = 0; k < NV; k++)
        w.M = std::fmax(w.M, std::fabs(t.M(r, k) - sign(r) * sign(k) * Mmj[r * NV + k]));
      w.h = std::fmax(w.h, std::fabs(t.h(r) - sign(r) * d->qfrc_bias[r]));
    }

    const Eigen::Vector3d x0 = vec(d->xpos + 3 * base);
    for (int leg = 0; leg < 4; leg++) {
      // Foot centre: the controller's point in the calf frame, from MuJoCo's
      // own pose, must be where the model puts it.
      const Eigen::Vector3d foot_mj =
          vec(d->xpos + 3 * calf[leg]) +
          Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(d->xmat + 9 * calf[leg]) *
              foot_calf;
      w.pos = std::fmax(w.pos, (t.foot[leg].p + x0 - foot_mj).cwiseAbs().maxCoeff());
      Eigen::Vector3d pt = x0 + t.foot[leg].p;
      mj_jac(m, d, jacp, jacr, pt.data(), calf[leg]);
      mj_jacDot(m, d, jdp, jdr, pt.data(), calf[leg]);
      w.J_foot = std::fmax(w.J_foot, jacError(t.foot[leg].J, jacp));
      w.Jdv_foot =
          std::fmax(w.Jdv_foot, (t.foot[leg].Jdv - mulJac(jdp, d->qvel)).cwiseAbs().maxCoeff());

      // Contact constraint: d/dt (v_centre + r e_z x omega_calf), from the
      // centre's jacDot and the calf's rotational jacDot (jacr does not
      // depend on the point).
      const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d jdv_contact =
          mulJac(jdp, d->qvel) + rf * ez.cross(mulJac(jdr, d->qvel));
      w.Jdv_contact =
          std::fmax(w.Jdv_contact, (t.contact[leg].Jdv - jdv_contact).cwiseAbs().maxCoeff());
      w.pos = std::fmax(w.pos, (t.contact[leg].p - (t.foot[leg].p - rf * ez)).cwiseAbs().maxCoeff());
      pt = x0 + t.contact[leg].p;
      mj_jac(m, d, jacp, jacr, pt.data(), calf[leg]);
      w.J_contact = std::fmax(w.J_contact, jacError(t.contact[leg].J, jacp));

      // Abduction anchor, a point of the trunk.
      const int hip_jnt = 1 + 3 * leg;
      w.pos = std::fmax(w.pos,
                        (t.hip[leg].p + x0 - vec(d->xanchor + 3 * hip_jnt)).cwiseAbs().maxCoeff());
      pt = x0 + t.hip[leg].p;
      mj_jac(m, d, jacp, jacr, pt.data(), base);
      mj_jacDot(m, d, jdp, jdr, pt.data(), base);
      w.J_hip = std::fmax(w.J_hip, jacError(t.hip[leg].J, jacp));
      w.Jdv_hip =
          std::fmax(w.Jdv_hip, (t.hip[leg].Jdv - mulJac(jdp, d->qvel)).cwiseAbs().maxCoeff());
    }

    w.com = std::fmax(w.com, (t.com + x0 - vec(d->subtree_com + 3 * base)).cwiseAbs().maxCoeff());
    T_CHECK(std::fabs(t.mass - m->body_subtreemass[base]) < 1e-9);

    // Inverse dynamics with a random vdot: M vdot + h = qfrc_inverse +
    // qfrc_passive (the passive joint damping is not part of h). Beyond the
    // direct M and h checks this only confirms the sign convention of (6.1).
    QD::VectorV vdot;
    for (int k = 0; k < NV; k++) {
      vdot(k) = 20.0 * uni(rng);
      d->qacc[k] = sign(k) * vdot(k);
    }
    mj_inverse(m, d);
    T_CHECK(d->nefc == 0);
    const QD::VectorV lhs = t.M * vdot + t.h;
    for (int k = 0; k < NV; k++)
      w.inverse = std::fmax(
          w.inverse, std::fabs(lhs(k) - sign(k) * (d->qfrc_inverse[k] + d->qfrc_passive[k])));
  }

  // Per-call cost, Release build: compute() runs in every control tick.
  QD::state_t st;
  for (int leg = 0; leg < 4; leg++) {
    st.q[leg] = Eigen::Vector3d(0.1, 0.9, -1.8);
    st.qd[leg] = Eigen::Vector3d(0.5, -1.0, 2.0);
  }
  st.w_body = Eigen::Vector3d(0.3, -0.2, 0.1);
  QD::terms_t t;
  const int calls = 20000;
  double sink = 0.0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < calls; i++) {
    st.q[0](0) = 1e-6 * i;  // a different state each call
    dyn.compute(st, t);
    sink += t.h(0);
  }
  const double us = std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - t0).count() / calls;

  printf("total mass: model %.6f kg, MuJoCo %.6f kg\n", t.mass, m->body_subtreemass[base]);
  printf("worst over %d random states (tolerance in brackets):\n", SAMPLES);
  printf("  M  vs mj_fullM           %.3e  [1e-6]\n", w.M);
  printf("  h  vs qfrc_bias          %.3e  [1e-5]\n", w.h);
  printf("  foot/contact/hip points  %.3e  [1e-9]\n", w.pos);
  printf("  J foot    vs mj_jac      %.3e  [1e-8]\n", w.J_foot);
  printf("  J contact vs mj_jac      %.3e  [1e-8]\n", w.J_contact);
  printf("  J hip     vs mj_jac      %.3e  [1e-8]\n", w.J_hip);
  printf("  Jdv foot    vs jacDot    %.3e  [1e-8]\n", w.Jdv_foot);
  printf("  Jdv contact vs jacDot    %.3e  [1e-8]\n", w.Jdv_contact);
  printf("  Jdv hip     vs jacDot    %.3e  [1e-8]\n", w.Jdv_hip);
  printf("  com vs subtree_com       %.3e  [1e-9]\n", w.com);
  printf("  M vdot + h vs inverse    %.3e  [1e-5]\n", w.inverse);
  printf("foot radius %.4f m = geom size; foot centre vs geom pos %.3e m [2e-3, known]\n", rf,
         foot_offset);
  printf("compute(): %.2f us per call (%d calls, checksum %.3g)\n", us, calls, sink);

  mj_deleteData(d);
  mj_deleteModel(m);
  T_CHECK(w.M < 1e-6);
  // Analytic h: the measured 5e-7 residual is the unnormalized trunk inertia
  // quaternion in createGo2WholeBodyConfig (1e-13 once normalized), so 1e-5
  // still leaves 20x margin while catching spurious terms the plan's 1e-3 hid.
  T_CHECK(w.h < 1e-5);
  T_CHECK(w.pos < 1e-9);
  T_CHECK(w.J_foot < 1e-8 && w.J_contact < 1e-8 && w.J_hip < 1e-8);
  // mj_jacDot is analytic, not a finite difference, so both sides differ only
  // by rounding on biases of up to ~1e2 m/s^2 at these rates. 1e-8 is the
  // Jacobian bound; a missing or wrongly signed velocity-product term (e.g.
  // the r omega x (omega x e_z) a material-point formula would add to the
  // contact, measured 3e-3..1e1 m/s^2 over these states) is far above it.
  T_CHECK(w.Jdv_foot < 1e-8 && w.Jdv_contact < 1e-8 && w.Jdv_hip < 1e-8);
  T_CHECK(w.com < 1e-9);
  T_CHECK(w.inverse < 1e-5);
  printf("PASS test_quadruped_dynamics\n");
  return 0;
}
