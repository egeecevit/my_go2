set(SOURCES main.cc Supervisor.cc app_common.cc)

if(HARDWARE_TARGET STREQUAL "robot")
  set(ROBOTEXE robot)
  add_executable(${ROBOTEXE} ${SOURCES})
  target_link_libraries(${ROBOTEXE} robothw quadruped rtcore rtclient)
  install(TARGETS ${ROBOTEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/robot.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()

if(HARDWARE_TARGET STREQUAL "simulation")
  set(SIMEXE simulation)
  add_executable(${SIMEXE} ${SOURCES})
  target_link_libraries(${SIMEXE} mujocohw quadruped rtcore rtclient)

  install(TARGETS ${SIMEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/sim.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()

if(HARDWARE_TARGET STREQUAL "go1")
  # Hardware test binary
  set(GO1TEST go1test)
  add_executable(${GO1TEST} test_main.cc app_common.cc)
  target_link_libraries(${GO1TEST} go1hw quadruped rtcore rtclient)
  target_link_options(${GO1TEST} PRIVATE -no-pie)

  install(TARGETS ${GO1TEST})
  file(GLOB go1testscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/go1test.sh)
  install(FILES ${go1testscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()
