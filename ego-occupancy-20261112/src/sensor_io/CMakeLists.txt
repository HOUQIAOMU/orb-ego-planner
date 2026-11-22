# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_occupancy_rgbd STATIC src/depth.cpp src/worker.cpp src/current_frame.cpp)
target_include_directories(ego_occupancy_rgbd PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_occupancy_rgbd PUBLIC ego_occupancy ego_pose_bridge ${OpenCV_LIBS})
ego_warnings(ego_occupancy_rgbd )

target_include_directories(ego_occupancy_rgbd PUBLIC ${OpenCV_INCLUDE_DIRS})
if(EGO_BUILD_ORB)
  add_library(ego_camera_live src/live_source.cpp)
  target_link_libraries(ego_camera_live PUBLIC ego_occupancy_rgbd ego_mapping_options realsense2::realsense2)
  ego_warnings(ego_camera_live)
endif()
