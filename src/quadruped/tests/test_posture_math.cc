#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>

#include "quadruped/QuadrupedKinematics.hh"

const double TOL = 1e-4;

static QuadrupedKinematics::params_t makeGo1Params() {
  QuadrupedKinematics::params_t p;
  p.robot_name = "Go1-test";
  p.hip_positions << 0.1881, 0.04675, 0.0,
      0.1881, -0.04675, 0.0,
      -0.1881, 0.04675, 0.0,
      -0.1881, -0.04675, 0.0;
  p.link_lengths << 0.213, 0.213, 0.213, 0.213, 0.213, 0.213, 0.213, 0.213;
  p.hip_flexion_offset << 0.08, -0.08, 0.08, -0.08;
  p.hip_abduction_limits << -1.047, 1.047, -1.047, 1.047, -1.047, 1.047, -1.047, 1.047;
  p.hip_flexion_limits << -0.663, 2.966, -0.663, 2.966, -0.663, 2.966, -0.663, 2.966;
  p.knee_limits << -2.721, -0.837, -2.721, -0.837, -2.721, -0.837, -2.721, -0.837;
  p.joint_directions << 1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1;
  return p;
}

void test_fk_ik_roundtrip() {
  std::cout << "test_fk_ik_roundtrip..." << std::endl;
  QuadrupedKinematics kin(makeGo1Params());
  Eigen::Vector3d angles(0.0, 0.8, -1.6);
  for (int leg = 0; leg < 4; leg++) {
    Eigen::Vector3d footpos, angles_out;
    assert(kin.forwardKinematics(leg, angles, footpos));
    assert(kin.inverseKinematics(leg, footpos, angles_out));
    for (int j = 0; j < 3; j++) {
      double err = std::abs(angles[j] - angles_out[j]);
      if (err > TOL) {
        std::cerr << "  FAIL leg " << leg << " joint " << j
                  << ": expected " << angles[j] << " got " << angles_out[j] << std::endl;
        assert(false);
      }
    }
  }
  std::cout << "  PASS" << std::endl;
}

void test_damped_pseudoinverse_near_singular() {
  std::cout << "test_damped_pseudoinverse_near_singular..." << std::endl;
  QuadrupedKinematics kin(makeGo1Params());
  Eigen::Vector3d angles(0.0, 0.1, -0.85);
  Eigen::Matrix3d J;
  assert(kin.jacobian(0, angles, J));

  double lambda = 0.01;
  double lam2 = lambda * lambda;
  Eigen::Matrix3d JJT = J * J.transpose();
  JJT.diagonal().array() += lam2;
  Eigen::Vector3d adot = J.transpose() * JJT.inverse() * Eigen::Vector3d(0.01, 0.0, -0.05);

  for (int i = 0; i < 3; i++) {
    assert(std::isfinite(adot[i]));
    assert(std::abs(adot[i]) < 100.0);
  }
  std::cout << "  PASS" << std::endl;
}

void test_damped_matches_raw() {
  std::cout << "test_damped_matches_raw..." << std::endl;
  QuadrupedKinematics kin(makeGo1Params());
  Eigen::Vector3d angles(0.0, 0.8, -1.6);
  Eigen::Matrix3d J;
  assert(kin.jacobian(0, angles, J));

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
      assert(false);
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
    assert(std::abs(p_body[2] - (p_contact[2] - 0.05)) < 1e-10);
  }

  // Lower body by 0.03m — feet go less negative in z
  {
    Eigen::Vector3d t(0.0, 0.0, -0.03);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_body = R.transpose() * (p_contact - t);
    assert(std::abs(p_body[2] - (p_contact[2] + 0.03)) < 1e-10);
  }

  // Pitch forward 0.1 rad
  {
    double pitch = 0.1;
    double cp = std::cos(pitch), sp = std::sin(pitch);
    Eigen::Matrix3d Ry;
    Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
    Eigen::Vector3d p_body = Ry.transpose() * p_contact;
    double expected_z = p_contact[2] - sp * p_contact[0];
    assert(std::abs(p_body[2] - expected_z) < 0.01);
  }

  std::cout << "  PASS" << std::endl;
}

void test_joint_limit_check() {
  std::cout << "test_joint_limit_check..." << std::endl;
  QuadrupedKinematics kin(makeGo1Params());
  assert(kin.checkJointLimits(0, Eigen::Vector3d(0.0, 0.8, -1.6)));
  assert(!kin.checkJointLimits(0, Eigen::Vector3d(2.0, 0.8, -1.6)));
  assert(!kin.checkJointLimits(0, Eigen::Vector3d(0.0, 0.8, 0.0)));
  std::cout << "  PASS" << std::endl;
}

int main() {
  std::cout << "=== Posture Math Tests ===" << std::endl;
  test_fk_ik_roundtrip();
  test_damped_pseudoinverse_near_singular();
  test_damped_matches_raw();
  test_inverse_pose_transform();
  test_joint_limit_check();
  std::cout << "=== All tests passed ===" << std::endl;
  return 0;
}
