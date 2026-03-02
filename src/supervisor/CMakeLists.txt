set(SOURCES main.cc Supervisor.cc app_common.cc)

if(${COMPILE_ROBOT})
  set(ROBOTEXE robot)
  add_executable(${ROBOTEXE} ${SOURCES})
  target_link_libraries(${ROBOTEXE} robothw quadruped rtcore rtclient)
  install(TARGETS ${ROBOTEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/robot.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()

if(${COMPILE_SIMULATION})
  set(SIMEXE simulation)
  add_executable(${SIMEXE} ${SOURCES})
  target_link_libraries(${SIMEXE} mujocohw quadruped rtcore rtclient)

  install(TARGETS ${SIMEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/sim.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()

if(${COMPILE_GO1})
  set(SIMEXE go1simulation)
  add_executable(${SIMEXE} ${SOURCES})
  target_link_libraries(${SIMEXE} go1hw quadruped rtcore rtclient)
  target_link_options(${SIMEXE} PRIVATE -no-pie) # Required since unitree_legged_sdk.a is a precompiled library built without -fPIC

  install(TARGETS ${SIMEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/go1sim.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()
