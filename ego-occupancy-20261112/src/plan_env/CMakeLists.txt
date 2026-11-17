# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_occupancy STATIC src/map.cpp src/io.cpp)
target_include_directories(ego_occupancy PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_occupancy PUBLIC Eigen3::Eigen Threads::Threads)
ego_warnings(ego_occupancy )
