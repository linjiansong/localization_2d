cmake_minimum_required(VERSION 3.8)
project(localization_2d)

cmake_policy(SET CMP0074 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type." FORCE)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED)
find_package(solex_msgs REQUIRED)

# 包含路径（自动引入 ament_target_dependencies 无法处理的第三方库头文件）
include_directories(
  .
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(localization_2d_node
  src/node_main.cpp
  src/node.cpp
  src/localization.cpp
  include/pose_estimator/imu_tracker.cpp
  include/pose_estimator/pose_extrapolator.cpp
  include/map_builder/active_map.cpp
  include/map_builder/icp_aligner.cpp
  include/map_builder/local_map_builder.cpp
  include/map_builder/motion_filter.cpp
  include/map_builder/ndt_aligner.cpp
  include/map_builder/submap.cpp
  include/scan_match/ceres_scan_matcher_2d.cpp
  include/scan_match/fast_correlative_scan_matcher_2d.cpp
  include/scan_match/laser_data_inserter.cpp
  include/scan_match/precomputation_grid_2d.cpp
  include/scan_match/probability_grid.cpp
  include/scan_match/real_time_correlative_scan_matcher_2d.cpp
  include/pose_graph/pose_graph.cpp
)

# 使用 ament 处理 ROS 依赖，自动解决 nav_msgs 找不到头文件的问题
ament_target_dependencies(localization_2d_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  solex_msgs
)

target_link_libraries(localization_2d_node
  ${PCL_LIBRARIES}
  yaml-cpp
  Eigen3::Eigen
  ${CERES_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS localization_2d_node DESTINATION lib/${PROJECT_NAME})

# 【新增内容】：安装 launch 文件夹和其他可能需要的配置文件目录
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()