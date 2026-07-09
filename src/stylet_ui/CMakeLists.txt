cmake_minimum_required(VERSION 3.8)
project(stylet_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(stylet_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_library(stylet_ui SHARED
  src/target_definition_panel.cpp
  src/entry_point_tool.cpp
  include/stylet_ui/target_definition_panel.hpp
  include/stylet_ui/entry_point_tool.hpp
)

target_include_directories(stylet_ui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(stylet_ui
  rclcpp
  rclcpp_action
  stylet_msgs
  rviz_common
  rviz_rendering
  pluginlib
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(stylet_ui Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS stylet_ui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(stylet_ui)

ament_package()
