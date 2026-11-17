# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_bspline_opt STATIC src/collision_constraints.cpp src/rebound_optimizer.cpp)
target_include_directories(ego_bspline_opt PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_bspline_opt PUBLIC ego_path_searching ego_planner_math)
ego_warnings(ego_bspline_opt STRICT)

target_include_directories(ego_bspline_opt SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/third_party/lbfgs")
