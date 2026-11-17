# 模块仅声明自身源码和依赖；公共头文件命名空间保持兼容。
add_library(ego_display_color STATIC src/display_color.cpp)
target_include_directories(ego_display_color PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_display_color PUBLIC Eigen3::Eigen ego_occupancy)
ego_warnings(ego_display_color )

# 显示数据接口为独立头文件目标；无显示的规划应用也能使用，不引入Pangolin。
add_library(ego_display_contract INTERFACE)
target_include_directories(ego_display_contract INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ego_display_contract INTERFACE ego_occupancy)
if(EGO_BUILD_DISPLAY)
  add_library(ego_occupancy_display src/display.cpp)
  target_include_directories(ego_occupancy_display PRIVATE ${Pangolin_INCLUDE_DIRS})
  target_link_libraries(ego_occupancy_display PUBLIC ego_occupancy_rgbd ego_display_color ${Pangolin_LIBRARIES})
  ego_warnings(ego_occupancy_display)
endif()
