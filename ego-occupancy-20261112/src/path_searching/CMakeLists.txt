# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_path_searching STATIC src/planning_map_view.cpp src/astar.cpp)
target_include_directories(ego_path_searching PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_path_searching PUBLIC ego_planner_math ego_occupancy)
ego_warnings(ego_path_searching STRICT)
