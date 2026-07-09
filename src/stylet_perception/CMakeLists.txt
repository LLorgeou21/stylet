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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(cloud_preprocessor src/cloud_preprocessor.cpp)
target_include_directories(cloud_preprocessor PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_preprocessor ${PCL_LIBRARIES})
ament_target_dependencies(cloud_preprocessor rclcpp sensor_msgs pcl_conversions)
install(TARGETS cloud_preprocessor DESTINATION lib/${PROJECT_NAME})

add_executable(point_cloud_merger src/point_cloud_merger.cpp)
ament_target_dependencies(point_cloud_merger rclcpp sensor_msgs tf2_ros tf2_sensor_msgs message_filters)
install(TARGETS point_cloud_merger DESTINATION lib/${PROJECT_NAME})

add_executable(surface_registration src/surface_registration.cpp)
target_include_directories(surface_registration PUBLIC
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(surface_registration ${PCL_LIBRARIES})
ament_target_dependencies(surface_registration rclcpp sensor_msgs geometry_msgs std_msgs tf2_ros pcl_conversions ament_index_cpp)
install(TARGETS surface_registration DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registration_utils test/test_registration_utils.cpp)
  target_include_directories(test_registration_utils PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
