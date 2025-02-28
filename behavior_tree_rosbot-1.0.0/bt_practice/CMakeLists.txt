cmake_minimum_required(VERSION 3.8)
project(bt_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

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

# Install directories
install(DIRECTORY
  config
  bt_xml
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts for test separately things
install(PROGRAMS
  scripts/test_vision.py
  scripts/test_nav2.py
  scripts/autonomy_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install python scripts for the BT
ament_python_install_package(
  behaviors
  PACKAGE_DIR python/behaviors)

# Install c++ nodes
set(AUTONOMY_SOURCES
  src/vision_behavior.cpp
  src/nav2_behavior.cpp
)
set(TARGET_DEPENDS
  rclcpp rclcpp_action
  nav2_msgs sensor_msgs cv_bridge image_transport
  behaviortree_cpp tf2 tf2_ros tf2_geometry_msgs yaml-cpp
)
include_directories(include/${PROJECT_NAME})
add_executable(autonomy_node_cpp src/autonomy_node.cpp ${AUTONOMY_SOURCES})
install(TARGETS
  autonomy_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(autonomy_node_cpp ${TARGET_DEPENDS})
target_link_libraries(autonomy_node_cpp ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

ament_package()
