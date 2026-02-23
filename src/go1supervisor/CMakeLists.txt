set(SOURCES main.cc Go1Supervisor.cc)

if(${COMPILE_ROBOT})
  set(ROBOTEXE robot)
  add_executable(${ROBOTEXE} ${SOURCES})
  target_link_libraries(${ROBOTEXE} robothw quadruped rtcore rtclient)
  install(TARGETS ${ROBOTEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/robot.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()

if(${COMPILE_GO1})
  set(SIMEXE go1simulation)
  add_executable(${SIMEXE} ${SOURCES})
  target_link_libraries(${SIMEXE} mujocohw go1hw quadruped rtcore rtclient)

  install(TARGETS ${SIMEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/go1sim.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

endif()
