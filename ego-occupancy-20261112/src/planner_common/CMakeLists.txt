# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_planner_math STATIC src/planning_config.cpp src/seed_builder.cpp)
target_include_directories(ego_planner_math PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_planner_math PUBLIC ego_bspline ego_occupancy)
ego_warnings(ego_planner_math STRICT)
