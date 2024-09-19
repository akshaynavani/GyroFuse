cmake_minimum_required(VERSION 3.16)
project(GyroFuse VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GYROFUSE_BUILD_TESTS "Build unit tests" ON)
option(GYROFUSE_BUILD_CLI "Build the gyrofuse command line tool" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(gyrofuse
  src/csv_io.cpp
  src/tilt.cpp
  src/complementary_filter.cpp
  src/kalman_axis.cpp
  src/kalman_filter.cpp
  src/calibration.cpp
)
target_include_directories(gyrofuse PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(gyrofuse PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>
)
