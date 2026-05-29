#include "quadruped/QuadrupedConfigs.hh"

QuadrupedKinematics::params_t createGo2Config() {
  QuadrupedKinematics::params_t params;

  params.robot_name = "Unitree Go2";

  params.hip_positions << 0.1934, 0.0465, 0.0,
      0.1934, -0.0465, 0.0,
      -0.1934, 0.0465, 0.0,
      -0.1934, -0.0465, 0.0;

  params.link_lengths << 0.213, 0.213, 0.213, 0.213, 0.213, 0.213, 0.213, 0.213;

  params.hip_flexion_offset << 0.0955, -0.0955, 0.0955, -0.0955;

  params.hip_abduction_limits << -1.0472, 1.0472, -1.0472, 1.0472,
      -1.0472, 1.0472, -1.0472, 1.0472;

  params.hip_flexion_limits << -1.5708, 3.4907, -1.5708, 3.4907,
      -0.5236, 4.5379, -0.5236, 4.5379;

  params.knee_limits << -2.7227, -0.83776, -2.7227, -0.83776,
      -2.7227, -0.83776, -2.7227, -0.83776;

  params.joint_directions << 1.0, 1.0, 1.0, -1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, -1.0, 1.0, 1.0;

  return params;
}

QuadrupedKinematics::params_t createGo1Config() {
  QuadrupedKinematics::params_t params;

  params.robot_name = "Unitree Go1";

  params.hip_positions << 0.1881, 0.04675, 0.0,
      0.1881, -0.04675, 0.0,
      -0.1881, 0.04675, 0.0,
      -0.1881, -0.04675, 0.0;

  params.link_lengths << 0.213, 0.213, 0.213, 0.213, 0.213, 0.213, 0.213, 0.213;

  params.hip_flexion_offset << 0.08, -0.08, 0.08, -0.08;

  params.hip_abduction_limits << -1.047, 1.047, -1.047, 1.047,
      -1.047, 1.047, -1.047, 1.047;

  params.hip_flexion_limits << -0.663, 2.966, -0.663, 2.966,
      -0.663, 2.966, -0.663, 2.966;

  params.knee_limits << -2.721, -0.837, -2.721, -0.837,
      -2.721, -0.837, -2.721, -0.837;

  params.joint_directions << 1.0, 1.0, 1.0, -1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, -1.0, 1.0, 1.0;

  return params;
}
