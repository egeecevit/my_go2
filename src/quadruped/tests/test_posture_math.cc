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

void test_ik_rejects_limit_violation() {
  std::cout << "test_ik_rejects_limit_violation..." << std::endl;
  QuadrupedKinematics kin(makeGo1Params());

  // Start from a valid pose, then push foot into the clamp window.
  // Empirically validated on current code (2026-04-13):
  //   dz=0.10 -> IK returns true, FK roundtrip error 0.020 (silent clamp)
  //   dz=0.11 -> IK returns true, FK roundtrip error 0.050 (silent clamp)
  //   dz=0.13 -> IK returns false (NaN path, not the clamp bug)
  // So dz=0.11 is in the clamp window and will NOT hit the NaN path.
  Eigen::Vector3d angles_in(0.0, 0.8, -1.6);
  Eigen::Vector3d footpos;
  if (!kin.forwardKinematics(0, angles_in, footpos)) {
    std::cerr << "  FAIL: baseline FK failed" << std::endl;
    std::exit(1);
  }

  Eigen::Vector3d target = footpos;
  target(2) -= 0.11;

  // Current code: returns true (clamped). After fix: must return false.
  Eigen::Vector3d angles_out;
  bool result = kin.inverseKinematics(0, target, angles_out);

  // If IK claims success, the FK roundtrip must be exact.
  // Current code fails this: FK of clamped angles != target.
  if (result) {
    Eigen::Vector3d verify;
    if (!kin.forwardKinematicsUnchecked(0, angles_out, verify)) {
      std::cerr << "  FAIL: FK of IK result failed" << std::endl;
      std::exit(1);
    }
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
  QuadrupedKinematics kin(makeGo1Params());

  // Target 1m below the hip — well beyond max reach (l1+l2 = 0.426m)
  Eigen::Vector3d too_far(0.1881, 0.04675 + 0.08, -1.0);
  Eigen::Vector3d angles;
  if (kin.inverseKinematics(0, too_far, angles)) {
    std::cerr << "  FAIL: IK should reject geometrically unreachable target" << std::endl;
    std::exit(1);
  }

  std::cout << "  PASS" << std::endl;
}

void test_activation_frame_height_trajectory() {
  std::cout << "test_activation_frame_height_trajectory..." << std::endl;
  QuadrupedKinematics kin(makeGo1Params());

  // Capture foot position at a known pose (simulates activation)
  Eigen::Vector3d angles0(0.0, 0.8, -1.6);
  Eigen::Vector3d footB0;
  if (!kin.forwardKinematics(0, angles0, footB0)) {
    std::cerr << "  FAIL: baseline FK failed" << std::endl;
    std::exit(1);
  }

  // Body moves up 0.03m -> feet go down in body frame
  double delta_h = 0.03;
  Eigen::Vector3d target = footB0 - Eigen::Vector3d(0.0, 0.0, delta_h);

  // IK should succeed and FK roundtrip should match
  Eigen::Vector3d angles_new;
  if (!kin.inverseKinematics(0, target, angles_new)) {
    std::cerr << "  FAIL: IK failed for height-shifted target" << std::endl;
    std::exit(1);
  }

  Eigen::Vector3d verify;
  if (!kin.forwardKinematics(0, angles_new, verify)) {
    std::cerr << "  FAIL: FK of IK result failed" << std::endl;
    std::exit(1);
  }
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
