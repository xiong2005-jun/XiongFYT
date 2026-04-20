cmake_minimum_required(VERSION 3.8)
project(virtual_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rm_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含头文件
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 编译组件
add_library(virtual_serial_node SHARED
  src/virtual_serial_node.cpp
)
target_compile_features(virtual_serial_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  virtual_serial_node
  rclcpp
  rclcpp_components
  rm_interfaces
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  rm_utils
  Eigen3
)

# 注册组件
rclcpp_components_register_nodes(virtual_serial_node "fyt::serial_driver::VirtualSerialNode")
set(node_plugins "${node_plugins}fyt::serial_driver::VirtualSerialNode;$<TARGET_FILE:virtual_serial_node>\n")

# 安装库
install(TARGETS
  virtual_serial_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装launch文件到ROS2标准share目录
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
