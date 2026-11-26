add_executable(test_simmotorhw_bounds test_simmotorhw_bounds.cc)
target_link_libraries(test_simmotorhw_bounds mujocohw rtcore rtclient)
add_test(NAME test_simmotorhw_bounds COMMAND test_simmotorhw_bounds)

# Links MuJoCo directly rather than through mujocohw: the sign under test runs
# through MdlLegControl's Cartesian force helper and the physics, and pulling in
# MdlSimDriver would drag a GLFW window along with it for nothing. The model
# path is passed in rather than discovered, since this test has no
# configuration search path of its own.
add_executable(test_stance_force_sign test_stance_force_sign.cc)
target_link_libraries(test_stance_force_sign quadruped mujoco::mujoco rtcore rtclient)
add_test(NAME test_stance_force_sign
         COMMAND test_stance_force_sign
                 ${CMAKE_SOURCE_DIR}/models/unitree_go2/scene.xml)

add_executable(test_go2_leg_dynamics test_go2_leg_dynamics.cc)
target_link_libraries(test_go2_leg_dynamics quadruped mujoco::mujoco rtcore rtclient)
add_test(NAME test_go2_leg_dynamics
         COMMAND test_go2_leg_dynamics
                 ${CMAKE_SOURCE_DIR}/models/unitree_go2/scene.xml)

add_executable(test_quadruped_dynamics test_quadruped_dynamics.cc)
target_link_libraries(test_quadruped_dynamics quadruped mujoco::mujoco rtcore rtclient)
add_test(NAME test_quadruped_dynamics
         COMMAND test_quadruped_dynamics
                 ${CMAKE_SOURCE_DIR}/models/unitree_go2/scene.xml)

# Links mujocohw (for MdlSimDriver::getFootGRF()/setModelForTesting()) as well
# as quadruped (for MdlLegControl/QuadrupedKinematics, used to build a real
# standing feedforward torque, same as test_stance_force_sign.cc). mujocohw
# already carries SimHW.o's HARDWARE_IMPL(ClockHW/MotorHW/IMUHW), so unlike
# the mujoco-only tests above, this one must not declare them again.
add_executable(test_sim_grf test_sim_grf.cc)
target_link_libraries(test_sim_grf mujocohw quadruped mujoco::mujoco rtcore rtclient)
add_test(NAME test_sim_grf
         COMMAND test_sim_grf
                 ${CMAKE_SOURCE_DIR}/models/unitree_go2/scene.xml)

# Same link set as test_sim_grf, but quadruped first: nothing in this test
# names MdlSimDriver, so mujocohw's objects are only pulled in by the hardware
# singletons that MdlAMB.o/MdlLegControl.o reference, and they have to be
# known before the linker reaches mujocohw's GLFW dependency.
add_executable(test_amb_sim test_amb_sim.cc)
target_link_libraries(test_amb_sim quadruped mujocohw mujoco::mujoco rtcore rtclient)
add_test(NAME test_amb_sim
         COMMAND test_amb_sim
                 ${CMAKE_SOURCE_DIR}/models/unitree_go2/scene.xml)

# The whole-body realization of MdlAMB, with a bare MdlConvexMPC; link set and
# order of test_amb_sim.
add_executable(test_amb_wbc_sim test_amb_wbc_sim.cc)
target_link_libraries(test_amb_wbc_sim quadruped mujocohw mujoco::mujoco rtcore rtclient)
add_test(NAME test_amb_wbc_sim
         COMMAND test_amb_wbc_sim
                 ${CMAKE_SOURCE_DIR}/models/unitree_go2/scene.xml)
