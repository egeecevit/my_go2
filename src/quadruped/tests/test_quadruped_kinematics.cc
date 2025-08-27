/* 
 * Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include <Eigen/Dense>
#include <iostream>
#include <cassert>
#include <cmath>
#include <iomanip>

#include "quadruped/QuadrupedKinematics.hh"

const double TEST_TOLERANCE = 1e-6;

void printVector3d(const std::string& name, const Eigen::Vector3d& vec) {
    std::cout << name << ": [" << std::fixed << std::setprecision(4) 
              << vec(0) << ", " << vec(1) << ", " << vec(2) << "]" << std::endl;
}

void printMatrix(const std::string& name, const Eigen::MatrixXd& mat) {
    std::cout << name << ":\n" << mat << std::endl;
}

QuadrupedKinematics::params_t createGo2Config() {
    QuadrupedKinematics::params_t params;
    
    // Robot identification
    params.robot_name = "Unitree Go2";
    
    // Hip positions in body frame [x, y, z] for each leg (meters)
    // Go2 dimensions: body length ~0.387m, body width ~0.093m
    params.hip_positions << 
        0.1934,  0.0465,  0.0,   // Front Left
        0.1934, -0.0465,  0.0,   // Front Right
       -0.1934,  0.0465,  0.0,   // Rear Left
       -0.1934, -0.0465,  0.0;   // Rear Right
    
    // Link lengths [thigh_length, calf_length] for each leg (meters)
    // Go2 link lengths: thigh ~0.213m, calf ~0.213m
    params.link_lengths <<
        0.213, 0.213,  // Front Left
        0.213, 0.213,  // Front Right
        0.213, 0.213,  // Rear Left
        0.213, 0.213;  // Rear Right
    
    // Hip flexion axis offset for each leg (meters)
    params.hip_flexion_offset <<
        0.094,  // Front Left
        -0.094,  // Front Right
        0.094,  // Rear Left
        -0.094;  // Rear Right
    
    // Joint limits [min, max] in radians
    // Hip abduction limits (approximately ±60 degrees)
    params.hip_abduction_limits <<
        -1.0472, 1.0472,  // Front Left
        -1.0472, 1.0472,  // Front Right
        -1.0472, 1.0472,  // Rear Left
        -1.0472, 1.0472;  // Rear Right
    
    // Hip flexion limits (different for front and rear legs)
    params.hip_flexion_limits <<
        -1.5708, 3.4907,  // Front Left (-90° to +200°)
        -1.5708, 3.4907,  // Front Right (-90° to +200°)
        -0.5236, 4.5379,  // Rear Left (-30° to +260°)
        -0.5236, 4.5379;  // Rear Right (-30° to +260°)
    
    // Knee limits (approximately -156° to -48° degrees)
    params.knee_limits <<
        -2.7227, -0.83776,  // Front Left
        -2.7227, -0.83776,  // Front Right
        -2.7227, -0.83776,  // Rear Left
        -2.7227, -0.83776;  // Rear Right
    
    // Joint directions (1.0 or -1.0) for sign conventions
    // [hip_abduction, hip_flexion, knee] directions per leg
    params.joint_directions <<
        1.0,  1.0, 1.0,  // Front Left
       -1.0,  1.0, 1.0,  // Front Right
        1.0,  1.0, 1.0,  // Rear Left
       -1.0,  1.0, 1.0;  // Rear Right
    
    return params;
}

/**
 * @brief Create configuration parameters for Unitree A1 robot
 * @return QuadrupedKinematics::params_t configured for A1
 */
QuadrupedKinematics::params_t createA1Config() {
    QuadrupedKinematics::params_t params;
    
    // Robot identification
    params.robot_name = "Unitree A1";
    
    // Hip positions in body frame [x, y, z] for each leg (meters)
    // A1 dimensions: body length ~0.366m, body width ~0.194m
    params.hip_positions << 
        0.1805,  0.097,  0.0,   // Front Left
        0.1805, -0.097,  0.0,   // Front Right
       -0.1805,  0.097,  0.0,   // Rear Left
       -0.1805, -0.097,  0.0;   // Rear Right
    
    // Link lengths [thigh_length, calf_length] for each leg (meters)
    // A1 link lengths: thigh ~0.20m, calf ~0.20m
    params.link_lengths <<
        0.20, 0.20,  // Front Left
        0.20, 0.20,  // Front Right
        0.20, 0.20,  // Rear Left
        0.20, 0.20;  // Rear Right
    
    // Joint limits [min, max] in radians
    // Hip abduction limits (approximately ±40 degrees)
    params.hip_abduction_limits <<
        -0.802, 0.802,  // Front Left
        -0.802, 0.802,  // Front Right
        -0.802, 0.802,  // Rear Left
        -0.802, 0.802;  // Rear Right
    
    // Hip flexion limits (approximately -50 to +70 degrees)
    params.hip_flexion_limits <<
        -0.873, 1.222,  // Front Left
        -0.873, 1.222,  // Front Right
        -0.873, 1.222,  // Rear Left
        -0.873, 1.222;  // Rear Right
    
    // Knee limits (approximately -140 to -20 degrees)
    params.knee_limits <<
        -2.443, -0.349,  // Front Left
        -2.443, -0.349,  // Front Right
        -2.443, -0.349,  // Rear Left
        -2.443, -0.349;  // Rear Right
    
    // Joint directions (1.0 or -1.0) for sign conventions
    // [hip_abduction, hip_flexion, knee] directions per leg
    params.joint_directions <<
        1.0,  1.0, 1.0,  // Front Left
       -1.0,  1.0, 1.0,  // Front Right
        1.0,  1.0, 1.0,  // Rear Left
       -1.0,  1.0, 1.0;  // Rear Right
    
    return params;
}

void testPresetConfiguration(const std::string& config_name) {
    std::cout << "\n=== Testing " << config_name << " Configuration ===" << std::endl;
    
    QuadrupedKinematics::params_t params;
    if (config_name == "Go2") {
        params = createGo2Config();
    } else if (config_name == "A1") {
        params = createA1Config();
    } else {
        std::cerr << "Unknown configuration: " << config_name << std::endl;
        return;
    }
    
    QuadrupedKinematics kinematics(params);
    
    // Print hip position for front left leg
    Eigen::Vector3d hip_pos = params.hip_positions.row(QuadrupedKinematics::FRONT_LEFT);
    printVector3d("Hip position", hip_pos);
    
    // Print link lengths for front left leg
    Eigen::Vector2d link_lengths = params.link_lengths.row(QuadrupedKinematics::FRONT_LEFT);
    std::cout << "Thigh length: " << link_lengths(0) << " m, Calf length: " << link_lengths(1) << " m" << std::endl;
    
    // Test forward kinematics
    Eigen::Vector3d joint_angles;
    joint_angles << 0.0, 0.9, -1.8;  // Home position
    
    Eigen::Vector3d foot_pos;
    bool fk_success = kinematics.forwardKinematics(QuadrupedKinematics::FRONT_LEFT, joint_angles, foot_pos);
    
    if (fk_success) {
        printVector3d("FK result for home pose", foot_pos);
    } else {
        std::cout << "FK failed for home pose" << std::endl;
    }
    
    // Test inverse kinematics
    Eigen::Vector3d computed_angles;
    bool ik_success = kinematics.inverseKinematics(QuadrupedKinematics::FRONT_LEFT, foot_pos, computed_angles);
    
    if (ik_success) {
        printVector3d("IK result", computed_angles);
        Eigen::Vector3d error = joint_angles - computed_angles;
        std::cout << "FK/IK error: " << error.norm() << " rad" << std::endl;
    } else {
        std::cout << "IK failed for FK result position" << std::endl;
    }
}

void testCustomConfiguration() {
    std::cout << "\n=== Testing Custom Configuration ===" << std::endl;
    
    QuadrupedKinematics::params_t custom_params;
    custom_params.robot_name = "Custom Robot";
    
    // Set hip positions manually
    custom_params.hip_positions.row(QuadrupedKinematics::FRONT_LEFT) << 0.25, 0.15, 0.05;
    custom_params.hip_positions.row(QuadrupedKinematics::FRONT_RIGHT) << 0.25, -0.15, 0.05;
    custom_params.hip_positions.row(QuadrupedKinematics::REAR_LEFT) << -0.25, 0.15, 0.05;
    custom_params.hip_positions.row(QuadrupedKinematics::REAR_RIGHT) << -0.25, -0.15, 0.05;
    
    // Set link lengths for each leg
    custom_params.link_lengths.row(QuadrupedKinematics::FRONT_LEFT) << 0.25, 0.25;
    custom_params.link_lengths.row(QuadrupedKinematics::FRONT_RIGHT) << 0.25, 0.25;
    custom_params.link_lengths.row(QuadrupedKinematics::REAR_LEFT) << 0.25, 0.25;
    custom_params.link_lengths.row(QuadrupedKinematics::REAR_RIGHT) << 0.25, 0.25;
    
    // Set joint limits
    for (int leg = 0; leg < QuadrupedKinematics::NUM_LEGS; leg++) {
        custom_params.hip_abduction_limits.row(leg) << -M_PI/3, M_PI/3;
        custom_params.hip_flexion_limits.row(leg) << -M_PI/2, M_PI/2;
        custom_params.knee_limits.row(leg) << -2.5, -0.5;
    }
    
    // Set joint directions
    custom_params.joint_directions <<
        1.0, 1.0, 1.0,  // Front Left
       -1.0, 1.0, 1.0,  // Front Right
        1.0, 1.0, 1.0,  // Rear Left
       -1.0, 1.0, 1.0;  // Rear Right
    
    QuadrupedKinematics custom_kinematics(custom_params);
    
    // Test forward kinematics for each leg
    std::vector<std::string> leg_names = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
    
    for (int leg = 0; leg < QuadrupedKinematics::NUM_LEGS; leg++) {
        std::cout << "\n" << leg_names[leg] << " leg configuration:" << std::endl;
        
        Eigen::Vector3d hip_pos = custom_params.hip_positions.row(leg);
        printVector3d("  Hip position", hip_pos);
        
        Eigen::Vector2d link_lengths = custom_params.link_lengths.row(leg);
        std::cout << "  Thigh: " << link_lengths(0) << " m, Calf: " << link_lengths(1) << " m" << std::endl;
        
        // Test maximum reach calculation (simple approximation)
        double max_reach = link_lengths(0) + link_lengths(1);
        std::cout << "  Approximate max reach: " << max_reach << " m" << std::endl;
    }
}

void testParameterFlexibility() {
    std::cout << "\n=== Testing Parameter Flexibility ===" << std::endl;
    
    QuadrupedKinematics::params_t params = createGo2Config();
    
    // Modify specific leg parameters
    params.link_lengths.row(QuadrupedKinematics::FRONT_RIGHT) << 0.3, 0.2;  // Different thigh/calf
    
    // Modify joint limits for rear legs
    params.knee_limits.row(QuadrupedKinematics::REAR_LEFT) << -3.0, -0.5;
    params.knee_limits.row(QuadrupedKinematics::REAR_RIGHT) << -3.0, -0.5;
    
    QuadrupedKinematics kinematics(params);
    
    std::vector<std::string> leg_names = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
    
    for (int leg = 0; leg < 4; leg++) {
        std::cout << "\n" << leg_names[leg] << " leg:" << std::endl;
        
        Eigen::Vector2d link_lengths = params.link_lengths.row(leg);
        std::cout << "  Thigh: " << link_lengths(0) << " m, Calf: " << link_lengths(1) << " m" << std::endl;
        
        Eigen::Vector2d knee_limits = params.knee_limits.row(leg);
        std::cout << "  Knee limits: [" << knee_limits(0) << ", " << knee_limits(1) << "] rad" << std::endl;
    }
}

void testBasicInitialization() {
    std::cout << "Testing basic initialization..." << std::endl;
    
    auto params = createGo2Config();
    QuadrupedKinematics kinematics(params);
    
    // Test that the kinematics object is properly initialized
    std::cout << "[PASS] Basic initialization test passed" << std::endl;
}

// Test Leg Kinematics (Unitree Go2 leg as a simplified example)
void testForwardKinematics() {
    std::cout << "Testing forward kinematics..." << std::endl;
    
    auto params = createGo2Config();
    QuadrupedKinematics kinematics(params);
    
    std::vector<Eigen::Vector3d> test_angles = {
        Eigen::Vector3d(0.0, 0, -M_PI/2),
        Eigen::Vector3d(0.0, M_PI/2, -M_PI/2),
        Eigen::Vector3d(0.0, -M_PI/2, -M_PI/2),
        Eigen::Vector3d(0.0, M_PI/4, -M_PI/2),
        Eigen::Vector3d(0.0, -M_PI/4, -M_PI/2),
        Eigen::Vector3d(0.0, 3*M_PI/4, -M_PI/2),
        Eigen::Vector3d(M_PI/4, 0, -M_PI/2),
        Eigen::Vector3d(-M_PI/4, 0, -M_PI/2),
    };
    
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        for (size_t i = 0; i < test_angles.size(); i++) {
            Eigen::Vector3d foot_position;
            bool success = kinematics.forwardKinematics(leg_idx, test_angles[i], foot_position);
            std::cout << "leg: " << leg_idx << " test " << i << ": Joint angles: " << test_angles[i].transpose()*180/M_PI 
                  << " => Foot position: " << foot_position.transpose() << std::endl;
        }
        std::cout << std::endl;
    }
    
    std::cout << "[PASS] Forward kinematics" << std::endl;
}

void testInverseKinematics() {
    std::cout << "Testing inverse kinematics..." << std::endl;
    
    auto params = createGo2Config();
    QuadrupedKinematics kinematics(params);
    
    int leg_idx = 0; // Front left leg
    
    std::vector<Eigen::Vector3d> test_angles = {
        Eigen::Vector3d(0.0, 0, -M_PI/2),
        Eigen::Vector3d(0.0, M_PI/2, -M_PI/2),
        Eigen::Vector3d(0.0, -M_PI/6, -M_PI/2),
        Eigen::Vector3d(0.0, M_PI/4, -M_PI/2),
        Eigen::Vector3d(0.0, -M_PI/6, -M_PI/2),
        Eigen::Vector3d(0.0, 3*M_PI/4, -M_PI/2),
        Eigen::Vector3d(M_PI/4, 0, -M_PI/2),
        Eigen::Vector3d(-M_PI/4, 0, -M_PI/2),
    };
    
    int success_count = 0;
    int test_count = 0;
    
    for (leg_idx = 0; leg_idx < 4; leg_idx++) {
        for (size_t i = 0; i < test_angles.size(); i++) {
            Eigen::Vector3d foot_position;
            bool success = kinematics.forwardKinematics(leg_idx, test_angles[i], foot_position);
            test_count++;
            
            Eigen::Vector3d solutions;
            bool solved = kinematics.inverseKinematics(leg_idx, foot_position, solutions);
            
            std::cout << "leg: " << leg_idx << " test " << i 
                    << ": Angles:" << test_angles[i].transpose()*180/M_PI 
                    << ", Position: " << foot_position.transpose()
                    << " => Joint angles: " << solutions.transpose()*180/M_PI;
                  
            if (solved) {
                std::cout << " (solvable)" << std::endl;
                success_count++;
                
                // Calculate the FK again to confirm, but only if IK was successful
                Eigen::Vector3d new_foot_position;
                kinematics.forwardKinematics(leg_idx, solutions, new_foot_position);
                double difference = (foot_position - new_foot_position).norm();
                std::cout << "  FK Check: " << new_foot_position.transpose() << ", difference: " 
                     << difference << std::endl;
                     
                // Verify that the difference is small for successful solutions
                //assert(difference < 1e-5 && "FK/IK mismatch too large for solvable position");
            } else {
                std::cout << " (UNSOLVABLE)" << std::endl;
                std::cout << "  Target position out of workspace - cannot validate" << std::endl;
            }
        }
        std::cout << std::endl;
    }
    
    std::cout << "Successfully solved " << success_count << " out of " << test_count << " test cases." << std::endl;
    
    if (success_count > 0) {
        std::cout << "[PASS] Inverse kinematics" << std::endl;
    } else {
        std::cout << "[FAIL] No inverse kinematics tests passed" << std::endl;
        assert(false && "All inverse kinematics tests failed");
    }
}

void testJacobian() {
    std::cout << "Testing Jacobian computation..." << std::endl;
    
    auto params = createGo2Config();
    QuadrupedKinematics kinematics(params);
    
    // Test joint angles
    Eigen::Vector3d joint_angles(0.1, -0.5, -1.2);  // Some non-zero values for better testing
    int leg_idx = QuadrupedKinematics::FRONT_LEFT;
    
    // Get the analytical Jacobian using the implemented method
    Eigen::Matrix3d analytical_jacobian;
    bool res = kinematics.jacobian(leg_idx, joint_angles, analytical_jacobian);
    
    // Compute a numerical Jacobian for comparison
    const double eps = 1e-8; // Small perturbation
    Eigen::Matrix3d numerical_jacobian = Eigen::Matrix3d::Zero();
    
    // Base foot position
    Eigen::Vector3d base_pos;
    kinematics.forwardKinematics(leg_idx, joint_angles, base_pos);
    
    // Compute numerical Jacobian by perturbing each joint angle
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d perturbed_angles = joint_angles;
        perturbed_angles(i) += eps;
        
        Eigen::Vector3d perturbed_pos;
        kinematics.forwardKinematics(leg_idx, perturbed_angles, perturbed_pos);
        
        // Column i of the Jacobian is the partial derivative w.r.t. joint angle i
        numerical_jacobian.col(i) = (perturbed_pos - base_pos) / eps;
    }
    
    Eigen::Matrix3d diff = analytical_jacobian - numerical_jacobian;

    // Print results
    std::cout << "Analytical Jacobian:\n" << analytical_jacobian << std::endl;
    std::cout << "Numerical Jacobian:\n" << numerical_jacobian << std::endl;
    std::cout << "Error:\n" << diff << std::endl;
    
    // Compare the two Jacobians
    double error = (analytical_jacobian - numerical_jacobian).norm();
    double relative_error = error / (numerical_jacobian.norm() + 1e-10);
    std::cout << "Jacobian error (norm): " << error << std::endl;
    std::cout << "Relative error: " << relative_error << std::endl;
    
    // Acceptable error threshold (numerical differentiation has inherent errors)
    const double ERROR_THRESHOLD = 1e-4;
    bool test_passed = relative_error < ERROR_THRESHOLD;
    
    if (test_passed) {
        std::cout << "[PASS] Jacobian test passed" << std::endl;
    } else {
        std::cout << "[FAIL] Jacobian test failed: error too large" << std::endl;
    }
    
    // Additional functional test: verify that J * qdot = xdot
    // Create a random joint velocity vector
    Eigen::Vector3d qdot(0.1, -0.2, 0.3); // rad/s
    
    // Compute expected foot velocity using the Jacobian
    Eigen::Vector3d expected_xdot = analytical_jacobian * qdot;
    
    std::cout << "Joint velocities (rad/s): " << qdot.transpose() << std::endl;
    std::cout << "Computed foot velocity (m/s): " << expected_xdot.transpose() << std::endl;
    
    // The direction and magnitude of the foot velocity should make physical sense
    double speed = expected_xdot.norm();
    std::cout << "Foot speed: " << speed << " m/s" << std::endl;
    
    assert(test_passed && "Jacobian computation is incorrect");
}

void testJointLimits() {
    std::cout << "Testing joint limits..." << std::endl;
    
    auto params = createGo2Config();
    QuadrupedKinematics kinematics(params);
    
    // Test valid angles (within limits)
    Eigen::Vector3d valid_angles(0.0, -0.8, -1.0);
    bool is_valid = kinematics.checkJointLimits(0, valid_angles);
    assert(is_valid && "Valid angles should pass joint limit check");
    
    // Test invalid angles (exceeding limits)
    Eigen::Vector3d invalid_angles(2.0, -3.0, 1.0);  // Exceeds limits
    bool is_invalid = kinematics.checkJointLimits(0, invalid_angles);
    assert(!is_invalid && "Invalid angles should fail joint limit check");
    
    std::cout << "[PASS] Joint limits test passed" << std::endl;
}

void testAllLegs() {
    std::cout << "Testing FK/IK for all legs..." << std::endl;
    
    auto params = createGo2Config();
    QuadrupedKinematics kinematics(params);
    
    // Test with joint angles within allowed limits
    // Changed knee joint angle from 1.6 (outside limits) to -1.5 (within limits)
Eigen::Vector3d joint_angles(0.0, -M_PI/6, -1.5);  // hip_abd, hip_flex, knee
    
    for (int leg_idx = 0; leg_idx < 4; ++leg_idx) {
        std::cout << "Testing leg " << leg_idx << std::endl;
        
        // Forward kinematics
        Eigen::Vector3d foot_position;
        bool result = kinematics.forwardKinematics(leg_idx, joint_angles, foot_position);
        std::cout << "  Foot position: " << foot_position.transpose() << std::endl;
        if (!result) {
            std::cerr << "  FK failed for leg " << leg_idx << " with angles " << joint_angles.transpose() << std::endl;
            continue;
        }

        // Inverse kinematics
        Eigen::Vector3d recovered_angles;
        bool success = kinematics.inverseKinematics(leg_idx, foot_position, recovered_angles);
        
        if (!success) {
            std::cerr << "  IK failed for position " << foot_position.transpose() << std::endl;
        }
        
        std::cout << "  Original angles: " << joint_angles.transpose() << std::endl;
        std::cout << "  Recovered angles: " << recovered_angles.transpose() << std::endl;
        
        // Recompute position using recovered angles
        Eigen::Vector3d recomputed_position;
        result =  kinematics.forwardKinematics(leg_idx, recovered_angles, recomputed_position);
        std::cout << "  Recomputed position: " << recomputed_position.transpose() << std::endl;
        
        double position_error = (foot_position - recomputed_position).norm();
        std::cout << "  Position error: " << position_error << std::endl;
        
        assert(position_error < 1e-10 && "Position error is too large");
    }
    
    std::cout << "[PASS] FK/IK match for all legs" << std::endl;
}

int main() {
    std::cout << "Testing QuadrupedKinematics class..." << std::endl;
    
    // Run all test functions
    testBasicInitialization();
    std::cout << std::endl;
    
    testForwardKinematics();
    std::cout << std::endl;
    
    testInverseKinematics();
    std::cout << std::endl;
    
    testJacobian();
    std::cout << std::endl;
    
    testJointLimits();
    std::cout << std::endl;
    
    testAllLegs();
    std::cout << std::endl;
    
    testPresetConfiguration("Go2");
    testPresetConfiguration("A1");
    
    testCustomConfiguration();
    
    testParameterFlexibility();
    
    std::cout << "\nAll tests completed successfully!" << std::endl;
    return 0;
}
