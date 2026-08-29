#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedLegDynamics.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

namespace {

int failures = 0;
const double kDirection[4][3] = {{1, 1, 1}, {-1, 1, 1}, {1, 1, 1}, {-1, 1, 1}};

#define CHECK(condition)                                                        \
  do {                                                                          \
    if (!(condition)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
      ++failures;                                                               \
    }                                                                           \
  } while (0)

void setState(mjModel* model, mjData* data, int leg, const Eigen::Vector3d& q,
              const Eigen::Vector3d& qdot, const Eigen::Vector3d& qddot) {
  mj_resetData(model, data);
  data->qpos[2] = 1.0;
  data->qpos[3] = 1.0;
  for (int j = 0; j < 3; ++j) {
    const int qi = 7 + 3 * leg + j;
    const int vi = 6 + 3 * leg + j;
    data->qpos[qi] = kDirection[leg][j] * q[j];
    data->qvel[vi] = kDirection[leg][j] * qdot[j];
    data->qacc[vi] = kDirection[leg][j] * qddot[j];
  }
  mj_forward(model, data);
  // mj_forward computes accelerations; restore the acceleration posed to the
  // inverse problem after it has populated transforms and velocities.
  mju_zero(data->qacc, model->nv);
  for (int j = 0; j < 3; ++j)
    data->qacc[6 + 3 * leg + j] = kDirection[leg][j] * qddot[j];
  mj_inverse(model, data);
}

Eigen::Vector3d inverseTorque(const mjData* data, int leg) {
  Eigen::Vector3d tau;
  for (int j = 0; j < 3; ++j)
    tau[j] = kDirection[leg][j] * data->qfrc_inverse[6 + 3 * leg + j];
  return tau;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <scene.xml>\n", argv[0]);
    return 2;
  }
  char error[1024] = "";
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "could not load model: %s\n", error);
    return 2;
  }
  mjData* data = mj_makeData(model);
  CHECK(data != nullptr);

  // Passive joint losses are commanded separately by MdlLegControl and are not
  // part of C*qdot+G in paper equation (2).
  for (int i = 0; i < model->nv; ++i) {
    model->dof_damping[i] = 0.0;
    model->dof_frictionloss[i] = 0.0;
  }

  QuadrupedLegDynamics dynamics(createGo2DynamicsConfig());
  const Eigen::Vector3d poses[] = {{0.0, 0.9, -1.8},
                                   {0.18, 1.15, -2.1},
                                   {-0.22, 0.72, -1.45}};
  const Eigen::Vector3d velocities[] = {{0.0, 0.0, 0.0},
                                        {0.7, -1.1, 1.4},
                                        {-0.4, 0.8, -1.2}};

  for (int leg = 0; leg < 4; ++leg) {
    for (int sample = 0; sample < 3; ++sample) {
      QuadrupedLegDynamics::terms_t terms;
      CHECK(dynamics.compute(leg, poses[sample], velocities[sample],
                             Eigen::Vector3d(0.0, 0.0, -9.81), terms));

      setState(model, data, leg, poses[sample], velocities[sample],
               Eigen::Vector3d::Zero());
      const Eigen::Vector3d mujoco_bias = inverseTorque(data, leg);
      const double bias_error = (terms.bias - mujoco_bias).cwiseAbs().maxCoeff();
      if (bias_error > 2e-5) {
        std::fprintf(stderr,
                     "leg %d sample %d bias mismatch %.9g\n"
                     "  ours %.9g %.9g %.9g\n  mj   %.9g %.9g %.9g\n",
                     leg, sample, bias_error, terms.bias[0], terms.bias[1],
                     terms.bias[2], mujoco_bias[0], mujoco_bias[1], mujoco_bias[2]);
        ++failures;
      }

      Eigen::Matrix3d mujoco_mass;
      for (int column = 0; column < 3; ++column) {
        Eigen::Vector3d qddot = Eigen::Vector3d::Zero();
        qddot[column] = 1.0;
        setState(model, data, leg, poses[sample], Eigen::Vector3d::Zero(), qddot);
        const Eigen::Vector3d with_acceleration = inverseTorque(data, leg);
        setState(model, data, leg, poses[sample], Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero());
        mujoco_mass.col(column) = with_acceleration - inverseTorque(data, leg);
      }
      const double mass_error =
          (terms.mass_matrix - mujoco_mass).cwiseAbs().maxCoeff();
      if (mass_error > 2e-6) {
        std::fprintf(stderr, "leg %d sample %d mass mismatch %.9g\n", leg, sample,
                     mass_error);
        ++failures;
      }
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  if (failures != 0) {
    std::fprintf(stderr, "%d MuJoCo dynamics comparisons failed\n", failures);
    return 1;
  }
  std::printf("Go2 leg dynamics agree with MuJoCo\n");
  return 0;
}
