cmake_minimum_required(VERSION 3.8)
project(bt_beach_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
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
  bt_xml
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install c++ codes
# set(AUTONOMY_SOURCES
#   src/bn_pub_msg.cpp
# )
# Set the target link libraries
set(TARGET_DEPENDS
  rclcpp rclcpp_action
  std_msgs std_srvs
  nav2_msgs sensor_msgs cv_bridge image_transport
  tf2 tf2_ros tf2_geometry_msgs yaml-cpp
  behaviortree_cpp behaviortree_ros2
)
  # # Include directory for header files
  # include_directories(include/${PROJECT_NAME}) 
  # # Add the executable
  # add_executable(bt_autonomy src/bt_autonomy.cpp ${AUTONOMY_SOURCES})
  # install(TARGETS 
  #   bt_autonomy 
  #   DESTINATION lib/${PROJECT_NAME}
  # )
  # ament_target_dependencies(bt_autonomy ${TARGET_DEPENDS}) # Add the dependencies
  # target_link_libraries(bt_autonomy ${YAML_CPP_LIBRARIES}) # Link the libraries

# add_executable(main_bt src/main_bt.cpp ${AUTONOMY_SOURCES})
# ament_target_dependencies(main_bt ${TARGET_DEPENDS})
# install(TARGETS main_bt DESTINATION lib/${PROJECT_NAME})

# EXAMPLES of using the wrapper library
# - subscriber
# add_executable(ex_subscriber src/ex_subscriber.cpp src/bn_sub_info.cpp)
# ament_target_dependencies(ex_subscriber ${TARGET_DEPENDS})
# install(TARGETS ex_subscriber DESTINATION lib/${PROJECT_NAME})
# - publisher
# add_executable(ex_publisher src/ex_publisher.cpp src/bn_pub_info.cpp)
# ament_target_dependencies(ex_publisher ${TARGET_DEPENDS})
# install(TARGETS ex_publisher DESTINATION lib/${PROJECT_NAME})
# - action server: sleep (but it is not necessary to run the action server from BT-samples)
# add_executable(ex_action_sleep src/ex_action_sleep.cpp src/bn_action_sleep.cpp)
# ament_target_dependencies(ex_action_sleep ${TARGET_DEPENDS})
# install(TARGETS ex_action_sleep DESTINATION lib/${PROJECT_NAME})
# - action server: nav2
# add_executable(ex_action_nav2 src/ex_action_nav2.cpp src/bn_action_nav2.cpp)
# ament_target_dependencies(ex_action_nav2 ${TARGET_DEPENDS})
# install(TARGETS ex_action_nav2 DESTINATION lib/${PROJECT_NAME})
# - action server: spin
# add_executable(ex_action_spin src/ex_action_spin.cpp src/bn_action_spin.cpp)
# ament_target_dependencies(ex_action_spin ${TARGET_DEPENDS})
# install(TARGETS ex_action_spin DESTINATION lib/${PROJECT_NAME})
# - action server: wait
# add_executable(ex_action_wait src/ex_action_wait.cpp src/bn_action_wait.cpp)
# ament_target_dependencies(ex_action_wait ${TARGET_DEPENDS})
# install(TARGETS ex_action_wait DESTINATION lib/${PROJECT_NAME})
# - service server: reset localization
# add_executable(ex_service_reset_localization src/ex_service_reset_localization.cpp src/bn_service_reset_localization.cpp)
# ament_target_dependencies(ex_service_reset_localization ${TARGET_DEPENDS})
# install(TARGETS ex_service_reset_localization DESTINATION lib/${PROJECT_NAME})

# - test server
add_executable(my_test src/my_test.cpp src/my_test.cpp)
ament_target_dependencies(my_test ${TARGET_DEPENDS})
install(TARGETS my_test DESTINATION lib/${PROJECT_NAME})

# MAIN BEHAVIOR TREE
add_executable(main_bt src/main_bt.cpp src/main_bt.cpp)
ament_target_dependencies(main_bt ${TARGET_DEPENDS})
install(TARGETS main_bt DESTINATION lib/${PROJECT_NAME})

ament_package()
