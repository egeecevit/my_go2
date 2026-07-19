#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"

const double TOL = 1e-4;

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                     \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << ": " #cond   \
                << std::endl;                                             \
      std::exit(1);                                                       \
    }                                                                     \
  } while (0)

void test_fk_ik_roundtrip() {
  std::cout << "test_fk_ik_roundtrip..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());
  Eigen::Vector3d angles(0.0, 0.8, -1.6);
  for (int leg = 0; leg < 4; leg++) {
    Eigen::Vector3d footpos, angles_out;
    T_CHECK(kin.forwardKinematics(leg, angles, footpos));
    T_CHECK(kin.inverseKinematics(leg, footpos, angles_out));
    for (int j = 0; j < 3; j++) {
      double err = std::abs(angles[j] - angles_out[j]);
      if (err > TOL) {
        std::cerr << "  FAIL leg " << leg << " joint " << j
                  << ": expected " << angles[j] << " got " << angles_out[j] << std::endl;
        std::exit(1);
      }
    }
  }
  std::cout << "  PASS" << std::endl;
}

void test_jacobian_analytical_vs_numerical() {
  std::cout << "test_jacobian_analytical_vs_numerical..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());
  const double eps = 1e-7;
  const Eigen::Vector3d poses[] = {
      {0.0, 0.8, -1.6}, {0.1, -0.3, -1.2}, {-0.2, 1.2, -2.0}};

  for (int leg = 0; leg < 4; leg++) {
    for (const auto &angles : poses) {
      Eigen::Matrix3d J;
      T_CHECK(kin.jacobian(leg, angles, J));

      Eigen::Vector3d base;
      T_CHECK(kin.forwardKinematicsUnchecked(leg, angles, base));

      Eigen::Matrix3d Jnum;
      for (int i = 0; i < 3; i++) {
        Eigen::Vector3d perturbed = angles;
        perturbed(i) += eps;
        Eigen::Vector3d pp;
        T_CHECK(kin.forwardKinematicsUnchecked(leg, perturbed, pp));
        Jnum.col(i) = (pp - base) / eps;
      }

      double rel = (J - Jnum).norm() / (Jnum.norm() + 1e-12);
      if (rel >= TOL) {
        std::cerr << "  FAIL leg " << leg << ": jacobian relative error "
                  << rel << std::endl;
        std::exit(1);
      }
    }
  }
  std::cout << "  PASS" << std::endl;
}

void test_damped_pseudoinverse_near_singular() {
  std::cout << "test_damped_pseudoinverse_near_singular..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());
  Eigen::Vector3d angles(0.0, 0.1, -0.85);
  Eigen::Matrix3d J;
  T_CHECK(kin.jacobian(0, angles, J));

  double lambda = 0.01;
  double lam2 = lambda * lambda;
  Eigen::Matrix3d JJT = J * J.transpose();
  JJT.diagonal().array() += lam2;
  Eigen::Vector3d adot = J.transpose() * JJT.inverse() * Eigen::Vector3d(0.01, 0.0, -0.05);

  for (int i = 0; i < 3; i++) {
    T_CHECK(std::isfinite(adot[i]));
    T_CHECK(std::abs(adot[i]) < 100.0);
  }
  std::cout << "  PASS" << std::endl;
}

void test_damped_matches_raw() {
  std::cout << "test_damped_matches_raw..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());
  Eigen::Vector3d angles(0.0, 0.8, -1.6);
  Eigen::Matrix3d J;
  T_CHECK(kin.jacobian(0, angles, J));

  Eigen::Vector3d pdot(0.01, 0.02, -0.03);
  Eigen::Vector3d adot_raw = J.inverse() * pdot;

  double lam2 = 0.01 * 0.01;
  Eigen::Matrix3d JJT = J * J.transpose();
  JJT.diagonal().array() += lam2;
  Eigen::Vector3d adot_damp = J.transpose() * JJT.inverse() * pdot;

  for (int i = 0; i < 3; i++) {
    double err = std::abs(adot_raw[i] - adot_damp[i]);
    if (err > 0.01) {
      std::cerr << "  FAIL joint " << i << ": raw=" << adot_raw[i]
                << " damped=" << adot_damp[i] << std::endl;
      std::exit(1);
    }
  }
  std::cout << "  PASS" << std::endl;
}

void test_inverse_pose_transform() {
  std::cout << "test_inverse_pose_transform..." << std::endl;
  Eigen::Vector3d p_contact(0.19, 0.13, -0.25);

  // Raise body by 0.05m — feet go more negative in z
  {
    Eigen::Vector3d t(0.0, 0.0, 0.05);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_body = R.transpose() * (p_contact - t);
    T_CHECK(std::abs(p_body[2] - (p_contact[2] - 0.05)) < 1e-10);
  }

  // Lower body by 0.03m — feet go less negative in z
  {
    Eigen::Vector3d t(0.0, 0.0, -0.03);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_body = R.transpose() * (p_contact - t);
    T_CHECK(std::abs(p_body[2] - (p_contact[2] + 0.03)) < 1e-10);
  }

  // Pitch forward 0.1 rad: p_body = Ry^T * p_contact, so body-frame z
  // picks up the +sp*x term from the frame rotation
  {
    double pitch = 0.1;
    double cp = std::cos(pitch), sp = std::sin(pitch);
    Eigen::Matrix3d Ry;
    Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
    Eigen::Vector3d p_body = Ry.transpose() * p_contact;
    double expected_z = sp * p_contact[0] + cp * p_contact[2];
    T_CHECK(std::abs(p_body[2] - expected_z) < 1e-10);
  }

  std::cout << "  PASS" << std::endl;
}

void test_joint_limit_check() {
  std::cout << "test_joint_limit_check..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());
  T_CHECK(kin.checkJointLimits(0, Eigen::Vector3d(0.0, 0.8, -1.6)));
  T_CHECK(!kin.checkJointLimits(0, Eigen::Vector3d(2.0, 0.8, -1.6)));
  T_CHECK(!kin.checkJointLimits(0, Eigen::Vector3d(0.0, 0.8, 0.0)));
  std::cout << "  PASS" << std::endl;
}

void test_ik_rejects_limit_violation() {
  std::cout << "test_ik_rejects_limit_violation..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());

  // Start from a valid pose, then push foot into the clamp window.
  // Empirically validated on current code (2026-04-13):
  //   dz=0.10 -> IK returns true, FK roundtrip error 0.020 (silent clamp)
  //   dz=0.11 -> IK returns true, FK roundtrip error 0.050 (silent clamp)
  //   dz=0.13 -> IK returns false (NaN path, not the clamp bug)
  // So dz=0.11 is in the clamp window and will NOT hit the NaN path.
  Eigen::Vector3d angles_in(0.0, 0.8, -1.6);
  Eigen::Vector3d footpos;
  T_CHECK(kin.forwardKinematics(0, angles_in, footpos));

  Eigen::Vector3d target = footpos;
  target(2) -= 0.11;

  // If IK claims success, the FK roundtrip must be exact.
  Eigen::Vector3d angles_out;
  bool result = kin.inverseKinematics(0, target, angles_out);
  if (result) {
    Eigen::Vector3d verify;
    T_CHECK(kin.forwardKinematicsUnchecked(0, angles_out, verify));
    double err = (verify - target).norm();
    if (err >= 1e-4) {
      std::cerr << "  FAIL: IK returned true but FK(IK(target)) != target"
                << " (err=" << err << ") — silent clamping" << std::endl;
      std::exit(1);
    }
  }

  std::cout << "  PASS" << std::endl;
}

void test_ik_rejects_unreachable() {
  std::cout << "test_ik_rejects_unreachable..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());

  // Target 1m below the hip — well beyond max reach (l1+l2 = 0.426m)
  Eigen::Vector3d too_far(0.1881, 0.04675 + 0.08, -1.0);
  Eigen::Vector3d angles;
  T_CHECK(!kin.inverseKinematics(0, too_far, angles));

  std::cout << "  PASS" << std::endl;
}

void test_activation_frame_height_trajectory() {
  std::cout << "test_activation_frame_height_trajectory..." << std::endl;
  QuadrupedKinematics kin(createGo1Config());

  // Capture foot position at a known pose (simulates activation)
  Eigen::Vector3d angles0(0.0, 0.8, -1.6);
  Eigen::Vector3d footB0;
  T_CHECK(kin.forwardKinematics(0, angles0, footB0));

  // Body moves up 0.03m -> feet go down in body frame
  double delta_h = 0.03;
  Eigen::Vector3d target = footB0 - Eigen::Vector3d(0.0, 0.0, delta_h);

  // IK should succeed and FK roundtrip should match
  Eigen::Vector3d angles_new;
  T_CHECK(kin.inverseKinematics(0, target, angles_new));

  Eigen::Vector3d verify;
  T_CHECK(kin.forwardKinematics(0, angles_new, verify));
  for (int i = 0; i < 3; i++) {
    if (std::abs(verify[i] - target[i]) >= 1e-4) {
      std::cerr << "  FAIL: FK roundtrip mismatch at index " << i
                << ": got " << verify[i] << " want " << target[i] << std::endl;
      std::exit(1);
    }
  }

  std::cout << "  PASS" << std::endl;
}

int main() {
  std::cout << "=== Posture Math Tests ===" << std::endl;
  test_fk_ik_roundtrip();
  test_jacobian_analytical_vs_numerical();
  test_damped_pseudoinverse_near_singular();
  test_damped_matches_raw();
  test_inverse_pose_transform();
  test_joint_limit_check();
  test_ik_rejects_limit_violation();
  test_ik_rejects_unreachable();
  test_activation_frame_height_trajectory();
  std::cout << "=== All tests passed ===" << std::endl;
  return 0;
}
