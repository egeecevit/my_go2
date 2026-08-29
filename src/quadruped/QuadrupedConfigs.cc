#include "quadruped/QuadrupedConfigs.hh"

#include <Eigen/Geometry>

namespace {

Eigen::Matrix3d inertiaFromPrincipal(double i0, double i1, double i2, double qw,
                                     double qx, double qy, double qz) {
  const Eigen::Matrix3d rotation =
      Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  return rotation * Eigen::Vector3d(i0, i1, i2).asDiagonal() * rotation.transpose();
}

void setLink(QuadrupedLegDynamics::link_t& link, double mass,
             const Eigen::Vector3d& com, const Eigen::Vector3d& diagonal_inertia,
             const Eigen::Vector4d& quaternion_wxyz) {
  link.mass = mass;
  link.com = com;
  link.inertia_com =
      inertiaFromPrincipal(diagonal_inertia[0], diagonal_inertia[1], diagonal_inertia[2],
                           quaternion_wxyz[0], quaternion_wxyz[1], quaternion_wxyz[2],
                           quaternion_wxyz[3]);
}

}  // namespace

QuadrupedLegDynamics::params_t createGo2DynamicsConfig() {
  QuadrupedLegDynamics::params_t params;
  params.robot_name = "Unitree Go2";
  params.armature = Eigen::Vector3d::Constant(0.01);
  params.torque_limit = Eigen::Vector3d(23.7, 23.7, 45.43);

  static const double hip_x[4] = {0.1934, 0.1934, -0.1934, -0.1934};
  static const double side[4] = {1.0, -1.0, 1.0, -1.0};
  static const Eigen::Vector4d hip_quat[4] = {
      {0.497014, 0.499245, 0.505462, 0.498237},
      {0.498237, 0.505462, 0.499245, 0.497014},
      {0.505462, 0.498237, 0.497014, 0.499245},
      {0.499245, 0.497014, 0.498237, 0.505462}};

  for (int leg = 0; leg < QuadrupedLegDynamics::NUM_LEGS; ++leg) {
    auto& lp = params.leg[leg];
    lp.joint_direction = Eigen::Vector3d(side[leg], 1.0, 1.0);
    lp.joint[0].parent_translation = Eigen::Vector3d(hip_x[leg], 0.0465 * side[leg], 0.0);
    lp.joint[0].axis = Eigen::Vector3d::UnitX();
    lp.joint[1].parent_translation = Eigen::Vector3d(0.0, 0.0955 * side[leg], 0.0);
    lp.joint[1].axis = Eigen::Vector3d::UnitY();
    lp.joint[2].parent_translation = Eigen::Vector3d(0.0, 0.0, -0.213);
    lp.joint[2].axis = Eigen::Vector3d::UnitY();
    lp.foot = Eigen::Vector3d(0.0, 0.0, -0.213);

    setLink(lp.joint[0].link, 0.678,
            Eigen::Vector3d((leg < 2 ? -1.0 : 1.0) * 0.0054,
                            side[leg] * 0.00194, -0.000105),
            Eigen::Vector3d(0.00088403, 0.000596003, 0.000479967), hip_quat[leg]);

    const Eigen::Vector4d thigh_quat =
        side[leg] > 0.0 ? Eigen::Vector4d(0.829533, 0.0847635, -0.0200632, 0.551623)
                        : Eigen::Vector4d(0.551623, -0.0200632, 0.0847635, 0.829533);
    setLink(lp.joint[1].link, 1.152,
            Eigen::Vector3d(-0.00374, -side[leg] * 0.0223, -0.0327),
            Eigen::Vector3d(0.00594973, 0.00584149, 0.000878787), thigh_quat);

    const Eigen::Vector4d calf_quat =
        side[leg] > 0.0 ? Eigen::Vector4d(0.710672, 0.00154099, -0.00450087, 0.703508)
                        : Eigen::Vector4d(0.703508, -0.00450087, 0.00154099, 0.710672);
    setLink(lp.joint[2].link, 0.241352,
            Eigen::Vector3d(0.00629595, -side[leg] * 0.000622121, -0.141417),
            Eigen::Vector3d(0.0014901, 0.00146356, 5.31397e-05), calf_quat);
  }
  return params;
}

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
