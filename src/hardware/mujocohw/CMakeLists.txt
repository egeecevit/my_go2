set(SOURCES SimHW.cc MdlSimDriver.cc SimMotorHW.cc SimIMUHW.cc SimClockHW.cc )

add_library(mujocohw STATIC ${SOURCES})
install(TARGETS mujocohw)

# Mujoco related section ==========================================================

message("Enabling MUJOCO simulation support")
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW) # INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW) # Default to GLVND if available.

enable_language(C)
enable_language(CXX)
set(SAMPLE_STANDALONE ON)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(SampleOptions)
include(SampleDependencies)

# no-unused is to prevent Eigen's compilation errors due to the no-unused warning combined with AVX
set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}" -Wno-unused)
set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

target_compile_options(mujocohw PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS}) 
target_link_libraries(mujocohw mujoco::mujoco glfw Threads::Threads )
target_link_options(mujocohw PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})

