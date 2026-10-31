cmake_minimum_required(VERSION 3.25)
project(fusion_slam VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

# System/toolchain-provided Eigen only: core builds never download dependencies.
find_package(Eigen3 3.3 QUIET CONFIG)
if(NOT TARGET Eigen3::Eigen)
  find_path(SLAM_EIGEN_INCLUDE_DIR NAMES Eigen/Core PATH_SUFFIXES eigen3)
  if(NOT SLAM_EIGEN_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 >= 3.3 is required. Install Eigen or set SLAM_EIGEN_INCLUDE_DIR.")
  endif()
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${SLAM_EIGEN_INCLUDE_DIR}")
endif()

add_library(slam_core STATIC src/core.cpp)
add_library(slam::core ALIAS slam_core)
target_compile_features(slam_core PUBLIC cxx_std_20)
target_include_directories(slam_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_core PUBLIC Eigen3::Eigen)
set_target_properties(slam_core PROPERTIES CXX_EXTENSIONS OFF)

if(BUILD_TESTING)
  add_executable(slam_core_smoke tests/core_smoke.cpp)
  target_link_libraries(slam_core_smoke PRIVATE slam::core)
  add_test(NAME core_smoke COMMAND slam_core_smoke)

  # Required Catch2 v3 suite is pinned and opt-out only for offline bootstrap
  # environments that cannot fetch dependencies (smoke test remains runnable).
  option(SLAM_ENABLE_CATCH2 "Fetch pinned Catch2 v3 for detailed tests" ON)
  if(SLAM_ENABLE_CATCH2)
    include(FetchContent)
    FetchContent_Declare(Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG 2b60af89e23d28eefc081bc930831ee9d45ea58b
      GIT_SHALLOW FALSE)
    FetchContent_MakeAvailable(Catch2)
    add_executable(slam_core_catch2 tests/core_catch2.cpp)
    target_link_libraries(slam_core_catch2 PRIVATE slam::core Catch2::Catch2WithMain)
    add_test(NAME core_catch2 COMMAND slam_core_catch2)
  endif()
endif()
