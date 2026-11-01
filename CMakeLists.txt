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

# The offline viewer has no C++ runtime dependency; run its standalone
# Python standard-library tests whenever Python >= 3.10 is installed.
if(BUILD_TESTING)
  find_package(Python3 3.10 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME web_ui_unit
      COMMAND "${Python3_EXECUTABLE}" -m unittest discover
        -s "${CMAKE_CURRENT_SOURCE_DIR}/tests/web"
        -p "test_*.py" -v)
    set_tests_properties(web_ui_unit PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    # Cross-language FUSMAP2D fixture parity: the bundled synthetic demo
    # must parse identically in the real C++ inspector and Python dashboard.
    add_test(NAME demo_fixture_native
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/web/check_native_fixture.py"
        "$<TARGET_FILE:slam_map_inspect>")
    set_tests_properties(demo_fixture_native PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
endif()

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

# Trust-gated timestamped gyro/wheel integration. No workstation solver
# dependency: the same sensor frontend can support a later Pi recorder.
add_library(slam_fusion_frontend STATIC src/gyro_wheel.cpp)
add_library(slam::fusion_frontend ALIAS slam_fusion_frontend)
target_compile_features(slam_fusion_frontend PUBLIC cxx_std_20)
target_include_directories(slam_fusion_frontend PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_fusion_frontend PUBLIC slam::motion)


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

# Portable, read-only known-map trust/validity layer. Works without GTSAM or
# Ceres and accepts an explicit Ceres matcher callback when linked by a consumer.
add_library(slam_known_map STATIC src/known_map.cpp)
add_library(slam::known_map ALIAS slam_known_map)
target_compile_features(slam_known_map PUBLIC cxx_std_20)
target_include_directories(slam_known_map PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_known_map PUBLIC slam::match_geometry)

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

find_package(OpenSSL QUIET COMPONENTS Crypto)
# FR079 sequential LiDAR + embedded scan-time odometry SLAM.
# No known-map reference or logged laser trajectory enters this estimator.
add_library(slam_fr079_slam_frontend STATIC src/fr079_offline.cpp)
add_library(slam::fr079_offline ALIAS slam_fr079_slam_frontend)
target_compile_features(slam_fr079_slam_frontend PUBLIC cxx_std_20)
target_include_directories(slam_fr079_slam_frontend PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_fr079_slam_frontend PUBLIC
  slam::carmen slam::map_bundle slam::match_geometry)
if(TARGET slam_match_ceres)
  target_link_libraries(slam_fr079_slam_frontend PUBLIC slam::match_ceres)
endif()
if(ZLIB_FOUND AND OpenSSL_FOUND)
  add_executable(slam_fr079_offline src/fr079_offline_cli.cpp)
  target_compile_features(slam_fr079_offline PRIVATE cxx_std_20)
  target_link_libraries(slam_fr079_offline PRIVATE
    slam::fr079_offline ZLIB::ZLIB OpenSSL::Crypto)
  if(BUILD_TESTING AND Python3_Interpreter_FOUND)
    add_test(NAME fr079_offline_slam
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fr079_offline_test.py"
        "$<TARGET_FILE:slam_fr079_offline>")
    set_tests_properties(fr079_offline_slam PROPERTIES TIMEOUT 240)
    if(SLAM_FR079_LOG)
      if(TARGET slam_match_ceres)
        add_test(NAME fr079_offline_real_data
          COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/fr079_offline_test.py"
            "$<TARGET_FILE:slam_fr079_offline>" "${SLAM_FR079_LOG}")
      else()
        add_test(NAME fr079_offline_real_data
          COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/fr079_offline_test.py"
            "$<TARGET_FILE:slam_fr079_offline>" "${SLAM_FR079_LOG}" --reference)
      endif()
      set_tests_properties(fr079_offline_real_data PROPERTIES TIMEOUT 600)
    endif()
  endif()
endif()

# Offline Ceres-to-replay diagnostics: writes JSON to stdout, never changes maps.
if(TARGET slam_match_ceres AND ZLIB_FOUND)
  add_executable(slam_ceres_replay_export src/ceres_replay_export.cpp)
  target_compile_features(slam_ceres_replay_export PRIVATE cxx_std_20)
  target_link_libraries(slam_ceres_replay_export PRIVATE
    slam::carmen slam::map_bundle slam::match_ceres slam::fr079_train_only ZLIB::ZLIB)
  if(BUILD_TESTING AND Python3_Interpreter_FOUND)
    add_test(NAME ceres_replay_native
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/web/test_native_ceres_export.py"
        "$<TARGET_FILE:slam_ceres_replay_export>")
    set_tests_properties(ceres_replay_native PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    # This regression rebuilds the actual train-only map, rejects a
    # nontraining map and checks the exact excluded scan index sequence.
    add_test(NAME ceres_heldout_native
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/web/test_native_heldout.py"
        "$<TARGET_FILE:slam_fr079_map_export>"
        "$<TARGET_FILE:slam_ceres_replay_export>")
    set_tests_properties(ceres_heldout_native PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
endif()

add_executable(slam_map_inspect src/map_inspect.cpp)
target_compile_features(slam_map_inspect PRIVATE cxx_std_20)
target_link_libraries(slam_map_inspect PRIVATE slam::map_bundle)

# Native read-only browser UI. No Python runtime and no Ceres/GTSAM. The
# browser JS/CSS assets remain unchanged and are served from a fixed allowlist.
option(SLAM_BUILD_NATIVE_DEMO "Build the native C++20 offline web dashboard" ON)
option(SLAM_REQUIRE_NATIVE_DEMO "Fail configure if native viewer dependencies are missing" OFF)
if(SLAM_BUILD_NATIVE_DEMO)
  find_package(OpenSSL QUIET COMPONENTS Crypto)
  if(OpenSSL_FOUND AND ZLIB_FOUND AND UNIX)
    add_executable(fusion_slam_demo
      apps/demo/native/main.cpp
      apps/demo/native/data.cpp
      apps/demo/native/fusion.cpp)
    target_compile_features(fusion_slam_demo PRIVATE cxx_std_20)
    target_include_directories(fusion_slam_demo PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}")
    target_compile_definitions(fusion_slam_demo PRIVATE
      FUSION_VIEWER_STATIC_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/apps/demo/web/static")
    target_link_libraries(fusion_slam_demo PRIVATE
      slam::map_bundle OpenSSL::Crypto ZLIB::ZLIB)
    set_target_properties(fusion_slam_demo PROPERTIES CXX_EXTENSIONS OFF)
    if(BUILD_TESTING)
      add_executable(slam_native_viewer_json_test tests/native_viewer_json_test.cpp)
      target_compile_features(slam_native_viewer_json_test PRIVATE cxx_std_20)
      target_include_directories(slam_native_viewer_json_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}")
      add_test(NAME native_viewer_json COMMAND slam_native_viewer_json_test)
    endif()
    if(BUILD_TESTING AND Python3_Interpreter_FOUND)
      add_test(NAME native_web_api
        COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/web/test_native_viewer.py"
          "$<TARGET_FILE:fusion_slam_demo>")
      set_tests_properties(native_web_api PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" TIMEOUT 90)
    endif()
  elseif(SLAM_REQUIRE_NATIVE_DEMO)
    message(FATAL_ERROR
      "Native C++ viewer requires POSIX sockets, ZLIB and OpenSSL::Crypto")
  else()
    message(STATUS "Native viewer skipped: POSIX, zlib or OpenSSL Crypto unavailable")
  endif()
endif()


# Ground-truth occupancy evaluation is independent of Ceres, GTSAM, zlib
# and network downloads. Inputs are immutable FUSMAP2D synthetic snapshots.
add_library(slam_map_accuracy STATIC src/map_accuracy.cpp)
add_library(slam::map_accuracy ALIAS slam_map_accuracy)
target_compile_features(slam_map_accuracy PUBLIC cxx_std_20)
target_include_directories(slam_map_accuracy PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(slam_map_accuracy PUBLIC slam_fixtures slam::map_bundle)
add_executable(slam_map_accuracy_cli src/map_accuracy_cli.cpp)
target_compile_features(slam_map_accuracy_cli PRIVATE cxx_std_20)
target_link_libraries(slam_map_accuracy_cli PRIVATE slam::map_accuracy)
add_executable(slam_synthetic_eval_fixture src/synthetic_eval_fixture.cpp)
target_compile_features(slam_synthetic_eval_fixture PRIVATE cxx_std_20)
target_link_libraries(slam_synthetic_eval_fixture PRIVATE
  slam_fixtures slam::map_bundle slam::mapping)
# Actual synthetic scan-to-map baseline: poses after the first anchor are
# inferred from biased relative odometry and independently held-out scans.
add_executable(slam_synthetic_mapping_benchmark
  src/synthetic_mapping_benchmark.cpp)
target_compile_features(slam_synthetic_mapping_benchmark PRIVATE cxx_std_20)
target_link_libraries(slam_synthetic_mapping_benchmark PRIVATE
  slam_fixtures slam::mapping slam::map_bundle
  slam::match_geometry slam::map_accuracy)
if(ZLIB_FOUND)
  add_executable(slam_fr079_map_export src/fr079_map_export.cpp)
  target_compile_features(slam_fr079_map_export PRIVATE cxx_std_20)
  target_link_libraries(slam_fr079_map_export PRIVATE
    slam::map_bundle slam::fr079_train_only ZLIB::ZLIB)
endif()

# Linux-only durable map generation registry. Does not require GTSAM or hardware.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_library(slam_map_registry STATIC src/generation_store.cpp)
  add_library(slam::map_registry ALIAS slam_map_registry)
  target_compile_features(slam_map_registry PUBLIC cxx_std_20)
  target_include_directories(slam_map_registry PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(slam_map_registry PUBLIC slam::map_bundle)
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

# Full deterministic *moving-scan* sensor-fusion experiment.
# Uses the exact same simulator for all baselines, without feeding reference
# poses to the estimator or rebuilding an oracle-pose map.
option(SLAM_PROFILE_FUSION "Collect steady-clock native fusion profiling samples" ON)
add_executable(slam_offline_fusion src/offline_fusion.cpp)
if(SLAM_PROFILE_FUSION)
  target_compile_definitions(slam_offline_fusion PRIVATE SLAM_PROFILE_FUSION=1)
endif()
target_compile_features(slam_offline_fusion PRIVATE cxx_std_20)
target_link_libraries(slam_offline_fusion PRIVATE
  slam::fusion_frontend slam::mapping slam::map_bundle
  slam::match_geometry slam::map_accuracy slam_fixtures)
if(TARGET slam_match_ceres)
  target_link_libraries(slam_offline_fusion PRIVATE slam::match_ceres)
  target_compile_definitions(slam_offline_fusion PRIVATE SLAM_FUSION_WITH_CERES=1)
endif()
if(TARGET slam_offline_graph)
  target_link_libraries(slam_offline_fusion PRIVATE slam::offline_graph)
  target_compile_definitions(slam_offline_fusion PRIVATE SLAM_FUSION_WITH_GTSAM=1)
endif()
if(BUILD_TESTING)
  add_executable(slam_fusion_profiler_test tests/fusion_profiler_test.cpp)
  target_compile_features(slam_fusion_profiler_test PRIVATE cxx_std_20)
  target_include_directories(slam_fusion_profiler_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  if(SLAM_PROFILE_FUSION)
    target_compile_definitions(slam_fusion_profiler_test PRIVATE SLAM_PROFILE_FUSION=1)
  endif()
  add_test(NAME fusion_profiler COMMAND slam_fusion_profiler_test)
  add_executable(slam_gyro_wheel_test tests/gyro_wheel_test.cpp)
  target_compile_features(slam_gyro_wheel_test PRIVATE cxx_std_20)
  target_link_libraries(slam_gyro_wheel_test PRIVATE slam::fusion_frontend)
  add_test(NAME gyro_wheel_fusion COMMAND slam_gyro_wheel_test)
  if(Python3_Interpreter_FOUND)
    add_test(NAME offline_fusion
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/offline_fusion_test.py"
        "$<TARGET_FILE:slam_offline_fusion>"
        "$<TARGET_FILE:slam_map_accuracy_cli>")
    set_tests_properties(offline_fusion PROPERTIES TIMEOUT 240)
    if(TARGET fusion_slam_demo)
      add_test(NAME fusion_replay_http
        COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/web/test_fusion_http.py"
          "$<TARGET_FILE:fusion_slam_demo>"
          "$<TARGET_FILE:slam_offline_fusion>")
      set_tests_properties(fusion_replay_http PROPERTIES TIMEOUT 240)
    endif()
  endif()
endif()

# Deliberately not linked to Raspberry Pi known-map localization. A verified
# offline graph is necessary for candidate generation and held-out promotion.
if(TARGET slam_offline_graph AND TARGET slam_map_registry)
  add_library(slam_offline_generation STATIC src/map_generation.cpp)
  add_library(slam::offline_generation ALIAS slam_offline_generation)
  target_compile_features(slam_offline_generation PUBLIC cxx_std_20)
  target_include_directories(slam_offline_generation PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(slam_offline_generation PUBLIC
    slam::offline_graph slam::map_bundle slam::map_registry slam::match_geometry)
endif()

if(BUILD_TESTING)
  if(TARGET slam_map_registry)
    add_executable(slam_map_registry_test tests/generation_store_test.cpp)
    target_link_libraries(slam_map_registry_test PRIVATE slam::map_registry)
    add_test(NAME map_registry COMMAND slam_map_registry_test)
  endif()
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
  add_executable(slam_map_accuracy_test tests/map_accuracy_test.cpp)
  target_link_libraries(slam_map_accuracy_test PRIVATE slam::map_accuracy)
  add_test(NAME map_accuracy COMMAND slam_map_accuracy_test)
  if(Python3_Interpreter_FOUND)
    add_test(NAME synthetic_mapping_benchmark
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/synthetic_mapping_benchmark_test.py"
        "$<TARGET_FILE:slam_synthetic_mapping_benchmark>"
        "$<TARGET_FILE:slam_map_accuracy_cli>"
        "${CMAKE_CURRENT_BINARY_DIR}")
    set_tests_properties(synthetic_mapping_benchmark PROPERTIES
      TIMEOUT 180)
  endif()
  if(Python3_Interpreter_FOUND)
    add_test(NAME map_accuracy_fixture_export
      COMMAND slam_synthetic_eval_fixture apartment
        "${CMAKE_CURRENT_BINARY_DIR}/accuracy-fixture.slammap")
    set_tests_properties(map_accuracy_fixture_export PROPERTIES
      FIXTURES_SETUP accuracy_exported_map)
    add_test(NAME map_accuracy_cli_smoke
      COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/map_accuracy_cli_test.py"
        "$<TARGET_FILE:slam_map_accuracy_cli>"
        "${CMAKE_CURRENT_BINARY_DIR}/accuracy-fixture.slammap")
    set_tests_properties(map_accuracy_cli_smoke PROPERTIES
      FIXTURES_REQUIRED accuracy_exported_map)
  endif()
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
  add_executable(slam_known_map_test tests/known_map_test.cpp)
  target_link_libraries(slam_known_map_test PRIVATE slam::known_map)
  add_test(NAME known_map_tracker COMMAND slam_known_map_test)
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
