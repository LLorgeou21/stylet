cmake_minimum_required(VERSION 3.8)
project(stylet_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(stylet_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(procedure_planner src/procedure_planner.cpp)
target_include_directories(procedure_planner PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(procedure_planner
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  stylet_msgs
  geometric_shapes
  shape_msgs
  ament_index_cpp
  control_msgs
)
install(TARGETS procedure_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp)
  target_include_directories(test_geometry_utils PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_geometry_utils moveit_msgs)
endif()

ament_package()
