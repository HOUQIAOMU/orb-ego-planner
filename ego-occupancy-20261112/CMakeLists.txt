cmake_minimum_required(VERSION 3.16)
project(ego_occupancy VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EGO_BUILD_RGBD "Build optional OpenCV RGB-D adapters" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

# Pose bridge shared with the patched ORB core.
add_library(ego_pose_bridge SHARED bridge/pose_bridge.cpp)
target_include_directories(ego_pose_bridge PUBLIC bridge)
target_link_libraries(ego_pose_bridge PUBLIC Threads::Threads)
target_compile_options(ego_pose_bridge PRIVATE -Wall -Wextra -Wpedantic)

# Core occupancy grid and snapshot IO.
add_library(ego_occupancy src/map.cpp src/io.cpp)
target_include_directories(ego_occupancy PUBLIC include)
target_link_libraries(ego_occupancy PUBLIC Eigen3::Eigen Threads::Threads)
target_compile_options(ego_occupancy PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)

# Display math stays independent of image, camera and OpenGL dependencies.
add_library(ego_display_color src/display_color.cpp)
target_include_directories(ego_display_color PUBLIC include)
target_link_libraries(ego_display_color PUBLIC Eigen3::Eigen)
target_compile_options(ego_display_color PRIVATE -Wall -Wextra -Wpedantic)
if(BUILD_TESTING)
  add_executable(occupancy-display-color-test tests/display_color_test.cpp)
  target_link_libraries(occupancy-display-color-test PRIVATE ego_display_color)
  add_test(NAME display_color_contract COMMAND occupancy-display-color-test)
  add_executable(occupancy-display-color-benchmark tests/display_color_benchmark.cpp)
  target_link_libraries(occupancy-display-color-benchmark PRIVATE ego_display_color ego_occupancy)
endif()

add_executable(ego-map-demo apps/map_demo.cpp)
target_link_libraries(ego-map-demo PRIVATE ego_occupancy)

add_executable(ego-map-inspect apps/map_inspect.cpp)
target_link_libraries(ego-map-inspect PRIVATE ego_occupancy)

if(BUILD_TESTING)
  add_executable(occupancy-map-test tests/map_test.cpp)
  target_link_libraries(occupancy-map-test PRIVATE ego_occupancy)
  add_test(NAME voxel_contract COMMAND occupancy-map-test)

  add_executable(occupancy-io-test tests/io_test.cpp)
  target_link_libraries(occupancy-io-test PRIVATE ego_occupancy)
  add_test(NAME snapshot_io_contract COMMAND occupancy-io-test)

  add_executable(occupancy-bridge-test tests/bridge_test.cpp)
  target_link_libraries(occupancy-bridge-test PRIVATE ego_pose_bridge)
  add_test(NAME pose_bridge_contract COMMAND occupancy-bridge-test)
endif()

option(EGO_BUILD_DISPLAY "Build Pangolin occupancy display" OFF)
if(EGO_BUILD_DISPLAY)
  if(NOT EGO_BUILD_RGBD)
    message(FATAL_ERROR "Display requires RGB-D/OpenCV adapters")
  endif()

  find_package(Pangolin REQUIRED)
  add_library(ego_occupancy_display src/display.cpp)
  target_include_directories(ego_occupancy_display PRIVATE ${Pangolin_INCLUDE_DIRS})
  target_link_libraries(ego_occupancy_display
    PUBLIC ego_occupancy_rgbd ego_display_color ${Pangolin_LIBRARIES})
  target_compile_options(ego_occupancy_display PRIVATE -Wall -Wextra -Wpedantic)

  if(BUILD_TESTING AND UNIX)
    add_executable(occupancy-display-startup-test tests/display_startup_test.cpp)
    target_link_libraries(occupancy-display-startup-test
      PRIVATE ego_occupancy_display ${CMAKE_DL_LIBS})
    target_link_options(occupancy-display-startup-test
      PRIVATE -Wl,--export-dynamic)
    add_test(NAME display_startup_failure_contract
      COMMAND occupancy-display-startup-test)
    set_tests_properties(display_startup_failure_contract
      PROPERTIES TIMEOUT 15)

    add_executable(occupancy-display-depth-render-test tests/display_depth_render_test.cpp)
    target_link_libraries(occupancy-display-depth-render-test PRIVATE ego_occupancy_display)
    add_test(NAME display_depth_render_contract COMMAND occupancy-display-depth-render-test)
    set_tests_properties(display_depth_render_contract PROPERTIES
      SKIP_RETURN_CODE 77 TIMEOUT 15 RUN_SERIAL TRUE)

    add_executable(occupancy-display-render-test tests/display_render_test.cpp)
    target_link_libraries(occupancy-display-render-test
      PRIVATE ego_occupancy_display)
    add_test(NAME display_rgb_color_contract
      COMMAND occupancy-display-render-test color)
    add_test(NAME display_failure_cleanup_contract
      COMMAND occupancy-display-render-test cleanup)
    set_tests_properties(display_rgb_color_contract display_failure_cleanup_contract
      PROPERTIES
        SKIP_RETURN_CODE 77
        TIMEOUT 15
        RUN_SERIAL TRUE)
  endif()
endif()

option(EGO_BUILD_ORB "Build experimental ORB RGB-D replay/live mapping entry" OFF)
set(ORB_SLAM3_ROOT "" CACHE PATH "Existing built ORB checkout")
set(EGO_ORB_CORE "" CACHE FILEPATH "Independent ORB library built with pose_bridge")

if(EGO_BUILD_ORB)
  if(NOT EGO_BUILD_RGBD
     OR NOT EXISTS "${ORB_SLAM3_ROOT}/include/System.h"
     OR NOT EXISTS "${EGO_ORB_CORE}")
    message(FATAL_ERROR
      "ORB entry requires RGB-D, existing ORB_SLAM3_ROOT and built EGO_ORB_CORE")
  endif()

  find_package(realsense2 CONFIG REQUIRED)
  find_package(Pangolin REQUIRED)

  add_executable(ego-rgbd-map apps/rgbd_map.cpp apps/live_source.cpp)
  target_compile_definitions(ego-rgbd-map
    PRIVATE COMPILEDWITHC11 ELPP_NO_DEFAULT_LOG_FILE)
  target_include_directories(ego-rgbd-map PRIVATE
    "${ORB_SLAM3_ROOT}"
    "${ORB_SLAM3_ROOT}/include"
    "${ORB_SLAM3_ROOT}/include/CameraModels"
    "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
    ${Pangolin_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  target_link_libraries(ego-rgbd-map PRIVATE
    ego_occupancy_rgbd
    ego_mapping_options
    "${EGO_ORB_CORE}"
    realsense2::realsense2
    ${Pangolin_LIBRARIES})

  get_filename_component(EGO_CORE_DIR "${EGO_ORB_CORE}" DIRECTORY)
  set_target_properties(ego-rgbd-map PROPERTIES
    BUILD_RPATH
      "${EGO_CORE_DIR};${CMAKE_BINARY_DIR};${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")

  if(EGO_BUILD_DISPLAY)
    target_compile_definitions(ego-rgbd-map PRIVATE EGO_HAS_DISPLAY)
    target_link_libraries(ego-rgbd-map PRIVATE ego_occupancy_display)
  endif()

  if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME app_preflight_contract
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/tests/app_contract.py"
              "$<TARGET_FILE:ego-rgbd-map>")
  endif()
endif()

if(EGO_BUILD_RGBD)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs)

  add_library(ego_occupancy_rgbd
    src/depth.cpp
    src/worker.cpp
    src/current_frame.cpp)
  target_include_directories(ego_occupancy_rgbd PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ego_occupancy_rgbd
    PUBLIC ego_occupancy ego_pose_bridge ${OpenCV_LIBS})
  target_compile_options(ego_occupancy_rgbd PRIVATE -Wall -Wextra -Wpedantic)

  if(BUILD_TESTING)
    add_executable(occupancy-depth-test tests/depth_test.cpp)
    target_link_libraries(occupancy-depth-test PRIVATE ego_occupancy_rgbd)
    add_test(NAME depth_contract COMMAND occupancy-depth-test)

    add_executable(occupancy-worker-test tests/worker_test.cpp)
    target_link_libraries(occupancy-worker-test PRIVATE ego_occupancy_rgbd)
    add_test(NAME worker_contract COMMAND occupancy-worker-test)

    add_executable(occupancy-current-frame-test tests/current_frame_test.cpp)
    target_link_libraries(occupancy-current-frame-test
      PRIVATE ego_occupancy_rgbd)
    add_test(NAME current_frame_contract COMMAND occupancy-current-frame-test)
  endif()
endif()

add_library(ego_mapping_options src/options.cpp)
target_include_directories(ego_mapping_options PUBLIC include)

if(BUILD_TESTING)
  add_executable(occupancy-options-test tests/options_test.cpp)
  target_link_libraries(occupancy-options-test PRIVATE ego_mapping_options)
  add_test(NAME mapping_cli_contract COMMAND occupancy-options-test)
endif()

if(BUILD_TESTING AND EGO_BUILD_ORB)
  add_test(NAME bridge_relocation_contract
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_relink.py")
endif()
