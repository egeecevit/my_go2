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
