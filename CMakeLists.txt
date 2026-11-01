cmake_minimum_required(VERSION 3.25)
project(fusion_slam VERSION 0.1.0 LANGUAGES CXX)

# Ubuntu 24.04's stable APT packages contain GTSAM 4.2 and Catch2 3.4.
# Keep the existing GTSAM 4.3 / Catch2 3.16 development baseline unless
# the distro-profile is explicitly selected (for example, by Docker).
option(SLAM_APT_DEPENDENCIES
  "Use Ubuntu 24.04 APT-compatible minimum GTSAM/Catch2 versions" OFF)
if(SLAM_APT_DEPENDENCIES)
  set(_slam_gtsam_min_version "4.2")
  set(_slam_catch2_min_version "3.4")
else()
  set(_slam_gtsam_min_version "4.3")
  set(_slam_catch2_min_version "3.16")
endif()

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

add_library(slam_match_geometry STATIC src/match_geometry.cpp)
add_library(slam::match_geometry ALIAS slam_match_geometry)
target_compile_features(slam_match_geometry PUBLIC cxx_std_20)
target_include_directories(slam_match_geometry PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_match_geometry PUBLIC slam::distance_field)

# Ceres is an optional private solver: core tests and map construction never
# fetch solver dependencies or link GTSAM. Enable when the toolchain has Ceres.
option(SLAM_ENABLE_CERES "Build the Ceres scan-to-map matcher if available" ON)
option(SLAM_REQUIRE_CERES "Fail configuration if Ceres is requested but unavailable" OFF)
if(SLAM_ENABLE_CERES)
  find_package(Ceres QUIET CONFIG)
  if(Ceres_FOUND)
    add_library(slam_match_ceres STATIC src/match_ceres.cpp)
    add_library(slam::match_ceres ALIAS slam_match_ceres)
    target_compile_features(slam_match_ceres PUBLIC cxx_std_20)
    target_compile_definitions(slam_match_ceres PUBLIC SLAM_HAS_CERES=1)
    target_include_directories(slam_match_ceres PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    target_link_libraries(slam_match_ceres PUBLIC slam::match_geometry PRIVATE Ceres::ceres)
  else()
    if(SLAM_REQUIRE_CERES)
      message(FATAL_ERROR "Ceres required: set Ceres_DIR or CMAKE_PREFIX_PATH to the offline installation")
    endif()
    message(STATUS "Ceres not found: geometry reference matcher and EDT tests remain available")
  endif()
endif()

# CARMEN log parser is dependency-free; gzip file inspection needs zlib only.
add_library(slam_carmen STATIC src/carmen.cpp)
add_library(slam::carmen ALIAS slam_carmen)
target_compile_features(slam_carmen PUBLIC cxx_std_20)
target_include_directories(slam_carmen PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
add_library(slam_carmen_geometry STATIC src/carmen_geometry.cpp)
add_library(slam::carmen_geometry ALIAS slam_carmen_geometry)
target_compile_features(slam_carmen_geometry PUBLIC cxx_std_20)
target_include_directories(slam_carmen_geometry PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_carmen_geometry PUBLIC slam::carmen slam::core)

find_package(ZLIB QUIET)
if(ZLIB_FOUND)
  add_executable(slam_carmen_inspect src/carmen_inspect.cpp)
  target_compile_features(slam_carmen_inspect PRIVATE cxx_std_20)
  target_link_libraries(slam_carmen_inspect PRIVATE slam::carmen ZLIB::ZLIB)
else()
  message(STATUS "zlib not found: CARMEN text reader builds; gzip inspection disabled")
endif()
add_library(slam_carmen_map STATIC src/carmen_map.cpp)
add_library(slam::carmen_map ALIAS slam_carmen_map)
target_compile_features(slam_carmen_map PUBLIC cxx_std_20)
target_include_directories(slam_carmen_map PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
if(ZLIB_FOUND)
  add_executable(slam_carmen_map_inspect src/carmen_map_inspect.cpp)
  target_compile_features(slam_carmen_map_inspect PRIVATE cxx_std_20)
  target_link_libraries(slam_carmen_map_inspect PRIVATE slam::carmen_map ZLIB::ZLIB)
endif()
add_library(slam_fr079_alignment STATIC src/fr079_alignment.cpp)
add_library(slam::fr079_alignment ALIAS slam_fr079_alignment)
target_compile_features(slam_fr079_alignment PUBLIC cxx_std_20)
target_include_directories(slam_fr079_alignment PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_fr079_alignment PUBLIC slam::carmen slam::carmen_map)
if(ZLIB_FOUND)
  add_executable(slam_fr079_alignment_inspect src/fr079_alignment_inspect.cpp)
  target_compile_features(slam_fr079_alignment_inspect PRIVATE cxx_std_20)
  target_link_libraries(slam_fr079_alignment_inspect PRIVATE slam::fr079_alignment ZLIB::ZLIB)
endif()

set(SLAM_FR079_MAP "" CACHE FILEPATH "Locally supplied 5cm CARMEN binary occupancy map (opt-in)")

set(SLAM_FR079_LOG "" CACHE FILEPATH "Locally provided fr079.log.gz (opt-in)")
set(SLAM_FR079_SM_LOG "" CACHE FILEPATH "Locally provided fr079-sm.log.gz (opt-in)")

# Build a frozen scan-to-map benchmark only from training observations.
add_library(slam_fr079_train_only STATIC src/fr079_train_only.cpp)
add_library(slam::fr079_train_only ALIAS slam_fr079_train_only)
target_compile_features(slam_fr079_train_only PUBLIC cxx_std_20)
target_include_directories(slam_fr079_train_only PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_fr079_train_only PUBLIC slam::carmen slam::match_geometry)
if(TARGET slam_match_ceres)
  target_link_libraries(slam_fr079_train_only PUBLIC slam::match_ceres)
endif()
if(ZLIB_FOUND)
  add_executable(slam_fr079_train_only_benchmark src/fr079_train_only_cli.cpp)
  target_compile_features(slam_fr079_train_only_benchmark PRIVATE cxx_std_20)
  target_link_libraries(slam_fr079_train_only_benchmark PRIVATE
    slam::fr079_train_only ZLIB::ZLIB)
endif()

# Authoritative versioned map bundle and visualization-only PGM/YAML export.
add_library(slam_map_bundle STATIC src/map_bundle.cpp)
add_library(slam::map_bundle ALIAS slam_map_bundle)
target_compile_features(slam_map_bundle PUBLIC cxx_std_20)
target_include_directories(slam_map_bundle PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_map_bundle PUBLIC slam::distance_field)

add_executable(slam_map_inspect src/map_inspect.cpp)
target_compile_features(slam_map_inspect PRIVATE cxx_std_20)
target_link_libraries(slam_map_inspect PRIVATE slam::map_bundle)
if(ZLIB_FOUND)
  add_executable(slam_fr079_map_export src/fr079_map_export.cpp)
  target_compile_features(slam_fr079_map_export PRIVATE cxx_std_20)
  target_link_libraries(slam_fr079_map_export PRIVATE
    slam::map_bundle slam::fr079_train_only ZLIB::ZLIB)
endif()

# Workstation-only GTSAM 4.2+ Pose2 batch reference, never linked to Pi runtime.
# A locally installed GTSAM package is preferred; the uploaded full source
# can also be supplied explicitly without any download or vendoring.
option(SLAM_ENABLE_GTSAM "Enable offline GTSAM Pose2 batch graph when available" ON)
option(SLAM_REQUIRE_GTSAM "Fail if the GTSAM offline optimizer is unavailable" OFF)
set(SLAM_GTSAM_SOURCE_DIR "" CACHE PATH "Extracted GTSAM 4.3.0 source root (optional)")
if(SLAM_REQUIRE_GTSAM AND NOT SLAM_ENABLE_GTSAM)
  message(FATAL_ERROR "SLAM_REQUIRE_GTSAM requires SLAM_ENABLE_GTSAM=ON")
endif()
if(SLAM_ENABLE_GTSAM)
  find_package(GTSAM ${_slam_gtsam_min_version} QUIET CONFIG)
  if(NOT GTSAM_FOUND AND SLAM_GTSAM_SOURCE_DIR)
    if(NOT EXISTS "${SLAM_GTSAM_SOURCE_DIR}/CMakeLists.txt")
      message(FATAL_ERROR "SLAM_GTSAM_SOURCE_DIR is not an extracted GTSAM source root")
    endif()
    set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
    set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
    set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
    set(GTSAM_USE_BOOST_FEATURES OFF CACHE BOOL "" FORCE)
    set(GTSAM_ENABLE_BOOST_SERIALIZATION OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_WITH_PRECOMPILED_HEADERS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${SLAM_GTSAM_SOURCE_DIR}"
      "${CMAKE_CURRENT_BINARY_DIR}/_deps/gtsam" EXCLUDE_FROM_ALL)
  endif()
  if(TARGET gtsam)
    add_library(slam_offline_graph STATIC src/pose_graph_gtsam.cpp)
    add_library(slam::offline_graph ALIAS slam_offline_graph)
    target_compile_features(slam_offline_graph PUBLIC cxx_std_20)
    target_include_directories(slam_offline_graph PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    target_link_libraries(slam_offline_graph PUBLIC slam::core PRIVATE gtsam)
    add_executable(slam_pose_graph_demo src/pose_graph_demo.cpp)
    target_link_libraries(slam_pose_graph_demo PRIVATE slam::offline_graph)
  elseif(SLAM_REQUIRE_GTSAM)
    message(FATAL_ERROR "GTSAM >= ${_slam_gtsam_min_version} required: set GTSAM_DIR or SLAM_GTSAM_SOURCE_DIR")
  else()
    message(STATUS "GTSAM >= ${_slam_gtsam_min_version} unavailable: optional workstation graph target skipped")
  endif()
endif()

# Deliberately not linked to Raspberry Pi known-map localization. A verified
# offline graph is necessary for candidate generation and held-out promotion.
if(TARGET slam_offline_graph)
  add_library(slam_offline_generation STATIC src/map_generation.cpp)
  add_library(slam::offline_generation ALIAS slam_offline_generation)
  target_compile_features(slam_offline_generation PUBLIC cxx_std_20)
  target_include_directories(slam_offline_generation PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(slam_offline_generation PUBLIC
    slam::offline_graph slam::map_bundle slam::match_geometry)
endif()

if(BUILD_TESTING)
  if(TARGET slam_offline_generation)
    add_executable(slam_map_generation_test tests/map_generation_test.cpp)
    target_link_libraries(slam_map_generation_test PRIVATE slam::offline_generation)
    add_test(NAME offline_map_promotion COMMAND slam_map_generation_test)
  endif()
  if(TARGET slam_offline_graph)
    add_executable(slam_pose_graph_test tests/pose_graph_test.cpp)
    target_link_libraries(slam_pose_graph_test PRIVATE slam::offline_graph)
    add_test(NAME pose_graph COMMAND slam_pose_graph_test)
    add_test(NAME pose_graph_demo COMMAND slam_pose_graph_demo)
  endif()
  add_executable(slam_map_bundle_test tests/map_bundle_test.cpp)
  target_link_libraries(slam_map_bundle_test PRIVATE slam::map_bundle)
  add_test(NAME map_bundle COMMAND slam_map_bundle_test)
  if(TARGET slam_fr079_map_export AND SLAM_FR079_LOG)
    add_test(NAME fr079_map_export
      COMMAND slam_fr079_map_export "${SLAM_FR079_LOG}"
      "${CMAKE_CURRENT_BINARY_DIR}/fr079-training")
    set_tests_properties(fr079_map_export PROPERTIES FIXTURES_SETUP fr079_exported_map)
    add_test(NAME fr079_map_import
      COMMAND slam_map_inspect "${CMAKE_CURRENT_BINARY_DIR}/fr079-training.slammap")
    set_tests_properties(fr079_map_import PROPERTIES FIXTURES_REQUIRED fr079_exported_map)
  endif()
  add_executable(slam_fr079_train_only_test tests/fr079_train_only_test.cpp)
  target_link_libraries(slam_fr079_train_only_test PRIVATE slam::fr079_train_only)
  add_test(NAME fr079_train_only_unit COMMAND slam_fr079_train_only_test)
  if(TARGET slam_fr079_train_only_benchmark AND SLAM_FR079_LOG)
    if(TARGET slam_match_ceres)
      add_test(NAME fr079_train_only_ceres
        COMMAND slam_fr079_train_only_benchmark "${SLAM_FR079_LOG}")
    else()
      add_test(NAME fr079_train_only_reference
        COMMAND slam_fr079_train_only_benchmark "${SLAM_FR079_LOG}" --reference)
    endif()
  endif()
  if(TARGET slam_fr079_alignment_inspect AND SLAM_FR079_MAP AND SLAM_FR079_SM_LOG)
    add_test(NAME fr079_alignment_diagnostic
      COMMAND slam_fr079_alignment_inspect "${SLAM_FR079_SM_LOG}" "${SLAM_FR079_MAP}")
  endif()
  if(SLAM_FR079_MAP)
    if(NOT EXISTS "${SLAM_FR079_MAP}")
      message(FATAL_ERROR "SLAM_FR079_MAP does not exist")
    endif()
    if(NOT TARGET slam_carmen_map_inspect)
      message(FATAL_ERROR "FR079 map tests require zlib")
    endif()
    add_test(NAME fr079_reference_grid COMMAND slam_carmen_map_inspect "${SLAM_FR079_MAP}" 910 323)
  endif()
  if(TARGET slam_carmen_inspect)
    if(SLAM_FR079_LOG)
      if(NOT EXISTS "${SLAM_FR079_LOG}")
        message(FATAL_ERROR "SLAM_FR079_LOG does not exist")
      endif()
      add_test(NAME fr079_full_log COMMAND slam_carmen_inspect "${SLAM_FR079_LOG}" 3118)
    endif()
    if(SLAM_FR079_SM_LOG)
      if(NOT EXISTS "${SLAM_FR079_SM_LOG}")
        message(FATAL_ERROR "SLAM_FR079_SM_LOG does not exist")
      endif()
      add_test(NAME fr079_sm_full_log COMMAND slam_carmen_inspect "${SLAM_FR079_SM_LOG}" 3118)
    endif()
  endif()
  add_executable(slam_match_geometry_test tests/matcher_test.cpp)
  target_link_libraries(slam_match_geometry_test PRIVATE slam::match_geometry)
  add_test(NAME match_geometry COMMAND slam_match_geometry_test)
  if(TARGET slam_match_ceres)
    add_executable(slam_match_ceres_test tests/matcher_test.cpp)
    target_link_libraries(slam_match_ceres_test PRIVATE slam::match_ceres)
    add_test(NAME match_ceres COMMAND slam_match_ceres_test)
  endif()
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
  option(SLAM_ENABLE_CATCH2 "Enable Catch2 v3 detailed tests" ON)
  option(SLAM_REQUIRE_SYSTEM_CATCH2
    "Fail when installed Catch2 is missing instead of fetching source" OFF)
  if(SLAM_REQUIRE_SYSTEM_CATCH2 AND NOT SLAM_ENABLE_CATCH2)
    message(FATAL_ERROR "SLAM_REQUIRE_SYSTEM_CATCH2 requires SLAM_ENABLE_CATCH2=ON")
  endif()
  if(SLAM_ENABLE_CATCH2)
    # Prefer restored CMake package to avoid unnecessary fetch/build work.
    find_package(Catch2 ${_slam_catch2_min_version} QUIET CONFIG)
    if(NOT TARGET Catch2::Catch2WithMain)
      if(SLAM_REQUIRE_SYSTEM_CATCH2)
        message(FATAL_ERROR
          "Catch2 >= ${_slam_catch2_min_version} required as an installed CMake package; refusing source download")
      endif()
      include(FetchContent)
    # Prefer the user-supplied complete release for air-gapped builds. Never
    # commit/download a vendored copy simply to run tests.
    set(SLAM_CATCH2_SOURCE_DIR "" CACHE PATH "Local Catch2 v3.16.1 source root")
    if(SLAM_CATCH2_SOURCE_DIR)
      if(NOT EXISTS "${SLAM_CATCH2_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "SLAM_CATCH2_SOURCE_DIR must point to the extracted Catch2 source root")
      endif()
      FetchContent_Declare(Catch2 SOURCE_DIR "${SLAM_CATCH2_SOURCE_DIR}")
    else()
      FetchContent_Declare(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG 08092139210881f01a34e60c5f2fd6afa0ea4024
        GIT_SHALLOW FALSE)
    endif()
      FetchContent_MakeAvailable(Catch2)
    endif()
    add_executable(slam_fr079_alignment_catch2 tests/fr079_alignment_test.cpp)
    target_link_libraries(slam_fr079_alignment_catch2 PRIVATE slam::fr079_alignment Catch2::Catch2WithMain)
    add_test(NAME fr079_alignment_unit COMMAND slam_fr079_alignment_catch2)
    add_executable(slam_carmen_map_catch2 tests/carmen_map_test.cpp)
    target_link_libraries(slam_carmen_map_catch2 PRIVATE slam::carmen_map Catch2::Catch2WithMain)
    add_test(NAME carmen_map_catch2 COMMAND slam_carmen_map_catch2)
    add_executable(slam_carmen_geometry_catch2 tests/carmen_geometry_test.cpp)
    target_link_libraries(slam_carmen_geometry_catch2 PRIVATE slam::carmen_geometry Catch2::Catch2WithMain)
    add_test(NAME carmen_geometry_catch2 COMMAND slam_carmen_geometry_catch2)
    add_executable(slam_carmen_catch2 tests/carmen_test.cpp)
    target_link_libraries(slam_carmen_catch2 PRIVATE slam::carmen Catch2::Catch2WithMain)
    add_test(NAME carmen_catch2 COMMAND slam_carmen_catch2)
    add_executable(slam_core_catch2 tests/core_catch2.cpp)
    target_link_libraries(slam_core_catch2 PRIVATE slam::core Catch2::Catch2WithMain)
    add_test(NAME core_catch2 COMMAND slam_core_catch2)
  endif()
endif()
