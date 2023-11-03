cmake_minimum_required(VERSION 3.0.2)
project(agihawk)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()


# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM arm64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    if (NOT DEFINED CATKIN_DEVEL_PREFIX)
        set(CMAKE_CXX_ARCH_FLAGS " -march=native")
    endif ()
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")

# Define Library
cs_add_library(px4_pilot
        src/px4_pilot.cpp
        src/ros_traj_visualizer.cpp
        src/bridge/px4_bridge.cpp
        src/bridge/ros_bridge.cpp
        src/bridge/rotors_bridge.cpp
        src/ctrl_feedback_publisher.cpp
        )

target_link_libraries(px4_pilot
  ${catkin_LIBRARIES}
  $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  link_directories("${PROJECT_SOURCE_DIR}/../agilicious_internal/agilib/externals/acados-src/lib") # Hack, necessary for now until we fix linking with gcc
endif()


target_link_libraries(px4_pilot "-Wl,--disable-new-dtags")

target_compile_options(px4_pilot PRIVATE
  -fno-finite-math-only
  -Wall
  -Werror
  -Wpedantic
  -Wunused
  -Wno-unused-parameter
  -Wundef
  -Wcast-align
  -Wredundant-decls
  -Wodr
  -Wunreachable-code
  -Wno-non-virtual-dtor
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(px4_pilot PRIVATE
    -Wmissing-include-dirs
    -Wmissing-declarations
    # To keep the compiler calm about Eigen
    -Wno-sign-conversion
    -Wno-implicit-int-float-conversion
    -Wno-c99-extensions
    -Wno-implicit-int-conversion
    -Wno-non-virtual-dtor
  )
endif()

# Define Executable
cs_add_executable(${PROJECT_NAME}
        src/node.cpp)
target_link_libraries(${PROJECT_NAME}
        px4_pilot)

# Finish
cs_install()
cs_export()