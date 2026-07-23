add_executable(test_posture_math test_posture_math.cc)
target_link_libraries(test_posture_math quadruped)
add_test(NAME test_posture_math COMMAND test_posture_math)

add_executable(test_axis_calibration test_axis_calibration.cc)
add_test(NAME test_axis_calibration COMMAND test_axis_calibration)

add_executable(test_config_layering test_config_layering.cc)
target_link_libraries(test_config_layering rtcore rtclient)
foreach(chain robotv1 sim go1)
  add_test(NAME test_config_layering_${chain}
           COMMAND test_config_layering ${CMAKE_SOURCE_DIR}/config ${chain})
endforeach()
