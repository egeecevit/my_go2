#include <cmath>
#include <cstdio>

#include <Eigen/Dense>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

namespace {

int failures = 0;

#define CHECK(condition)                                                        \
  do {                                                                          \
    if (!(condition)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
      ++failures;                                                               \
    }                                                                           \
  } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                  \
  do {                                                                           \
    const double a_ = (actual);                                                   \
    const double e_ = (expected);                                                 \
    if (std::fabs(a_ - e_) > (tolerance)) {                                       \
      std::fprintf(stderr, "FAIL %s:%d: %.12g != %.12g (tol %.3g)\n", __FILE__, \
                   __LINE__, a_, e_, (double)(tolerance));                        \
      ++failures;                                                                \
    }                                                                            \
  } while (0)

void test_go2_parameters_and_terms() {
  const auto params = createGo2DynamicsConfig();
  CHECK(params.validate());

  QuadrupedLegDynamics dynamics(params);
  const Eigen::Vector3d q(0.12, 0.85, -1.72);
  const Eigen::Vector3d qdot(0.4, -0.7, 1.1);
  const Eigen::Vector3d gravity_body(0.0, 0.0, -9.81);

  for (int leg = 0; leg < QuadrupedLegDynamics::NUM_LEGS; ++leg) {
    QuadrupedLegDynamics::terms_t terms;
    CHECK(dynamics.compute(leg, q, qdot, gravity_body, terms));
    CHECK(terms.mass_matrix.allFinite());
    CHECK(terms.bias.allFinite());
    CHECK(terms.jdot_qdot.allFinite());
    CHECK(terms.operational_inertia.allFinite());
    CHECK((terms.mass_matrix - terms.mass_matrix.transpose()).norm() < 1e-10);
    CHECK((terms.operational_inertia - terms.operational_inertia.transpose()).norm() <
          1e-9);
    CHECK(terms.mass_matrix.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff() >
          0.0);
    CHECK(terms.operational_inertia.selfadjointView<Eigen::Lower>()
              .eigenvalues()
              .minCoeff() > 0.0);
  }
}

void test_jdot_matches_directional_finite_difference() {
  const auto params = createGo2DynamicsConfig();
  QuadrupedLegDynamics dynamics(params);
  const Eigen::Vector3d q(-0.18, 1.05, -1.95);
  const Eigen::Vector3d qdot(0.6, -1.2, 1.7);
  const double dt = 1e-6;

  QuadrupedLegDynamics::terms_t terms;
  CHECK(dynamics.compute(0, q, qdot, Eigen::Vector3d(0.0, 0.0, -9.81), terms));

  Eigen::Matrix3d j_plus, j_minus;
  CHECK(dynamics.footJacobian(0, q + dt * qdot, j_plus));
  CHECK(dynamics.footJacobian(0, q - dt * qdot, j_minus));
  const Eigen::Vector3d numerical = ((j_plus - j_minus) / (2.0 * dt)) * qdot;
  CHECK_NEAR((terms.jdot_qdot - numerical).norm(), 0.0, 2e-7);
}

void test_mirrored_legs_have_consistent_inertia() {
  const auto params = createGo2DynamicsConfig();
  QuadrupedLegDynamics dynamics(params);
  const Eigen::Vector3d q(0.0, 0.9, -1.8);
  const Eigen::Vector3d qdot = Eigen::Vector3d::Zero();
  QuadrupedLegDynamics::terms_t left, right;
  CHECK(dynamics.compute(0, q, qdot, Eigen::Vector3d(0.0, 0.0, -9.81), left));
  CHECK(dynamics.compute(1, q, qdot, Eigen::Vector3d(0.0, 0.0, -9.81), right));

  // Controller coordinates make positive abduction point outward on both sides.
  // At the symmetric q1=0 pose, kinetic energy and gravity magnitudes therefore
  // agree even though the first generalized torque changes sign.
  CHECK_NEAR((left.mass_matrix - right.mass_matrix).norm(), 0.0, 2e-9);
  CHECK_NEAR(std::fabs(left.bias[0]), std::fabs(right.bias[0]), 2e-9);
  CHECK_NEAR((left.bias.tail<2>() - right.bias.tail<2>()).norm(), 0.0, 2e-9);
}

void test_dynamics_and_controller_kinematics_share_coordinates() {
  QuadrupedLegDynamics dynamics(createGo2DynamicsConfig());
  QuadrupedKinematics kinematics(createGo2Config());
  const Eigen::Vector3d poses[] = {{0.12, 0.85, -1.72},
                                   {-0.31, 1.21, -2.08},
                                   {0.44, 0.54, -1.27}};
  for (int leg = 0; leg < QuadrupedLegDynamics::NUM_LEGS; ++leg) {
    for (const Eigen::Vector3d& q : poses) {
      Eigen::Vector3d pd, pk;
      Eigen::Matrix3d jd, jk;
      CHECK(dynamics.footPosition(leg, q, pd));
      CHECK(dynamics.footJacobian(leg, q, jd));
      CHECK(kinematics.forwardKinematicsUnchecked(leg, q, pk));
      CHECK(kinematics.jacobian(leg, q, jk));
      CHECK_NEAR((pd - pk).norm(), 0.0, 2e-12);
      CHECK_NEAR((jd - jk).norm(), 0.0, 2e-12);
    }
  }
}

}  // namespace

int main() {
  test_go2_parameters_and_terms();
  test_jdot_matches_directional_finite_difference();
  test_mirrored_legs_have_consistent_inertia();
  test_dynamics_and_controller_kinematics_share_coordinates();
  if (failures != 0) {
    std::fprintf(stderr, "%d leg dynamics checks failed\n", failures);
    return 1;
  }
  std::printf("leg dynamics checks passed\n");
  return 0;
}
