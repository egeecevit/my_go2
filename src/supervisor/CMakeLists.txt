set(SOURCES main.cc Supervisor.cc)

if(${COMPILE_ROBOT})
  set(ROBOTEXE robot)
  add_executable(${ROBOTEXE} ${SOURCES})
  target_link_libraries(${ROBOTEXE} robothw quadruped rtcore rtclient)
  install(TARGETS ${ROBOTEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/robot.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

  #set_target_properties(${ROBOTEXE}
  #  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${${CMAKE_PROJECT_NAME}_SOURCE_DIR}/bin")
endif()

if(${COMPILE_SIMULATION})
  set(SIMEXE simulation)
  add_executable(${SIMEXE} ${SOURCES})
  target_link_libraries(${SIMEXE} simhw quadruped rtcore rtclient)

  install(TARGETS ${SIMEXE})
  file(GLOB binscript LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/sim.sh)
  install(FILES ${binscript} DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ)

  #set_target_properties(${SIMEXE}
  #  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${${CMAKE_PROJECT_NAME}_SOURCE_DIR}/bin")
endif()
