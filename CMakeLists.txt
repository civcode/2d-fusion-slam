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

add_library(slam_fixtures STATIC src/synthetic.cpp)
target_compile_features(slam_fixtures PUBLIC cxx_std_20)
target_include_directories(slam_fixtures PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_fixtures PUBLIC slam::core)
set_target_properties(slam_fixtures PROPERTIES CXX_EXTENSIONS OFF)

add_library(slam_sensors STATIC src/scan.cpp)
add_library(slam::sensors ALIAS slam_sensors)
target_compile_features(slam_sensors PUBLIC cxx_std_20)
target_include_directories(slam_sensors PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_sensors PUBLIC slam::core)

add_library(slam_ld19 STATIC src/ld19.cpp)
add_library(slam::ld19 ALIAS slam_ld19)
target_compile_features(slam_ld19 PUBLIC cxx_std_20)
target_include_directories(slam_ld19 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_ld19 PUBLIC slam::sensors)

add_library(slam_motion STATIC src/trajectory.cpp)
add_library(slam::motion ALIAS slam_motion)
target_compile_features(slam_motion PUBLIC cxx_std_20)
target_include_directories(slam_motion PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_motion PUBLIC slam::sensors slam::core)

add_library(slam_mapping STATIC src/occupancy.cpp)
add_library(slam::mapping ALIAS slam_mapping)
target_compile_features(slam_mapping PUBLIC cxx_std_20)
target_include_directories(slam_mapping PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_mapping PUBLIC slam::core slam::sensors)

add_library(slam_distance_field STATIC src/distance_field.cpp)
add_library(slam::distance_field ALIAS slam_distance_field)
target_compile_features(slam_distance_field PUBLIC cxx_std_20)
target_include_directories(slam_distance_field PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_distance_field PUBLIC slam::mapping)

if(BUILD_TESTING)
  add_executable(slam_distance_field_test tests/distance_field_test.cpp)
  target_link_libraries(slam_distance_field_test PRIVATE slam::distance_field)
  add_test(NAME distance_field COMMAND slam_distance_field_test)
  add_executable(slam_ld19_motion_test tests/ld19_motion_test.cpp)
  target_link_libraries(slam_ld19_motion_test PRIVATE slam_ld19 slam_motion slam_fixtures)
  add_test(NAME ld19_motion COMMAND slam_ld19_motion_test)
  add_executable(slam_scan_mapping_test tests/scan_mapping_test.cpp)
  target_link_libraries(slam_scan_mapping_test PRIVATE slam::mapping slam_fixtures)
  add_test(NAME scan_mapping COMMAND slam_scan_mapping_test)
  add_executable(slam_fixtures_smoke tests/fixtures_smoke.cpp)
  target_link_libraries(slam_fixtures_smoke PRIVATE slam_fixtures)
  add_test(NAME fixtures_smoke COMMAND slam_fixtures_smoke)

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
