cmake_minimum_required(VERSION 3.8)
project(stylet_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

add_executable(cloud_preprocessor src/cloud_preprocessor.cpp)
target_include_directories(cloud_preprocessor PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_preprocessor ${PCL_LIBRARIES})
ament_target_dependencies(cloud_preprocessor rclcpp sensor_msgs pcl_conversions)
install(TARGETS cloud_preprocessor DESTINATION lib/${PROJECT_NAME})

add_executable(point_cloud_merger src/point_cloud_merger.cpp)
ament_target_dependencies(point_cloud_merger rclcpp sensor_msgs tf2_ros tf2_sensor_msgs message_filters)
install(TARGETS point_cloud_merger DESTINATION lib/${PROJECT_NAME})

ament_package()
