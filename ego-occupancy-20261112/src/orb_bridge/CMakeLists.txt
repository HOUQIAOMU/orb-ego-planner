# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_pose_bridge SHARED src/pose_bridge.cpp)
target_include_directories(ego_pose_bridge PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_pose_bridge PUBLIC Threads::Threads)
ego_warnings(ego_pose_bridge )
