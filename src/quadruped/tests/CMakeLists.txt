add_executable(test_posture_math test_posture_math.cc)
target_link_libraries(test_posture_math quadruped)
add_test(NAME test_posture_math COMMAND test_posture_math)

add_executable(test_axis_calibration test_axis_calibration.cc)
add_test(NAME test_axis_calibration COMMAND test_axis_calibration)

# Links rtcore/rtclient because it pulls in MdlStateEstimator.o, which carries
# the rtcore Module base class alongside the filter math being tested.
add_executable(test_state_estimator test_state_estimator.cc)
target_link_libraries(test_state_estimator quadruped rtcore rtclient)
add_test(NAME test_state_estimator COMMAND test_state_estimator)

# Same reason as test_state_estimator: pulls in MdlOrientationEstimator.o, which
# carries the rtcore Module base class alongside the attitude filter.
add_executable(test_mahony_filter test_mahony_filter.cc)
target_link_libraries(test_mahony_filter quadruped rtcore rtclient)
add_test(NAME test_mahony_filter COMMAND test_mahony_filter)

# Same reason as test_state_estimator: pulls in MdlTrot.o, which carries the
# rtcore Module base class alongside the gait schedule being tested.
add_executable(test_trot_gait test_trot_gait.cc)
target_link_libraries(test_trot_gait quadruped rtcore rtclient)
add_test(NAME test_trot_gait COMMAND test_trot_gait)

# Same reason as test_state_estimator: pulls in MdlConvexMPC.o, which carries
# the rtcore Module base class alongside the QP being tested.
add_executable(test_convex_mpc test_convex_mpc.cc)
target_link_libraries(test_convex_mpc quadruped rtcore rtclient)
add_test(NAME test_convex_mpc COMMAND test_convex_mpc)

# Same reason as test_state_estimator: pulls in MdlLegControl.o, which carries
# the rtcore Module base class alongside the Cartesian force law being tested.
add_executable(test_leg_cartesian_force test_leg_cartesian_force.cc)
target_link_libraries(test_leg_cartesian_force quadruped rtcore rtclient)
add_test(NAME test_leg_cartesian_force COMMAND test_leg_cartesian_force)

add_executable(test_leg_dynamics test_leg_dynamics.cc)
target_link_libraries(test_leg_dynamics quadruped rtcore rtclient)
add_test(NAME test_leg_dynamics COMMAND test_leg_dynamics)

add_executable(test_config_layering test_config_layering.cc)
target_link_libraries(test_config_layering rtcore rtclient)
foreach(chain robotv1 sim go1)
  add_test(NAME test_config_layering_${chain}
           COMMAND test_config_layering ${CMAKE_SOURCE_DIR}/config ${chain})
endforeach()
