# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_planner_core STATIC src/trajectory_validator.cpp src/planner_core.cpp src/planning_fsm.cpp)
target_include_directories(ego_planner_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_planner_core PUBLIC ego_bspline_opt ego_path_searching ego_planner_math)
ego_warnings(ego_planner_core STRICT)

add_library(ego_planner_runtime src/planning_runtime.cpp)
target_link_libraries(ego_planner_runtime PUBLIC ego_planner_core Threads::Threads)
ego_warnings(ego_planner_runtime STRICT)
