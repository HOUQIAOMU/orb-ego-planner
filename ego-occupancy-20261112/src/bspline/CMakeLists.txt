# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_bspline STATIC src/bspline.cpp src/polynomial_traj.cpp)
target_include_directories(ego_bspline PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_bspline PUBLIC Eigen3::Eigen)
ego_warnings(ego_bspline STRICT)
