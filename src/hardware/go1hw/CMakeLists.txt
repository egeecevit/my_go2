cmake_minimum_required(VERSION 3.5)

set (SOURCES MdlGo1.cc)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(UNITREE_ARCH amd64)
elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(UNITREE_ARCH arm64)
else()
  message(FATAL_ERROR "go1hw: unsupported architecture ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(UNITREE_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk)

# Wrap the precompiled SDK as an IMPORTED target
add_library(unitree_legged_sdk STATIC IMPORTED GLOBAL)
set_target_properties(unitree_legged_sdk PROPERTIES
    IMPORTED_LOCATION             ${UNITREE_SDK_DIR}/lib/cpp/${UNITREE_ARCH}/libunitree_legged_sdk.a
    INTERFACE_INCLUDE_DIRECTORIES ${UNITREE_SDK_DIR}/include
    INTERFACE_LINK_LIBRARIES      "-pthread"
    INTERFACE_LINK_OPTIONS        "-no-pie"
)

add_library(go1hw STATIC ${SOURCES})
install(TARGETS go1hw)
target_include_directories(go1hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${UNITREE_SDK_DIR}/include)
# target_link_libraries(go1hw PUBLIC unitree_legged_sdk)
target_link_libraries(go1hw PRIVATE unitree_legged_sdk)
