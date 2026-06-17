cmake_minimum_required(VERSION 3.8)
project(visual_servoing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedGoal.msg"
  "msg/DetectedGoalArray.msg"
  "srv/SaveCurrentTagGoal.srv"
  "msg/SharedControlGoalDebug.msg"
  "msg/SharedControlDebug.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(visual_servoing_node
  src/visual_servoing_node.cpp
  src/visual_servoing_display.cpp
  src/camera_config.cpp
  src/apriltag_pose_estimator.cpp
  src/pose_transformer.cpp
  src/detected_goal_builder.cpp
  src/pose_save_client.cpp
)

add_executable(handeye_tf_publisher
  src/handeye_tf_publisher.cpp
)

add_executable(shared_control_visualization_node
  src/shared_control_visualization_node.cpp
)

add_executable(save_goal_manager
  src/save_goal_manager.cpp
)

target_include_directories(visual_servoing_node PUBLIC
  include
  third_party
  third_party/apriltag
  third_party/apriltag/common
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(save_goal_manager PUBLIC
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(visual_servoing_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

ament_target_dependencies(handeye_tf_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(shared_control_visualization_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(save_goal_manager
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

target_link_libraries(visual_servoing_node
  apriltag
  ${OpenCV_LIBRARIES}
  ${cpp_typesupport_target}
)

target_link_libraries(handeye_tf_publisher
  ${OpenCV_LIBRARIES}
)

target_include_directories(shared_control_visualization_node PUBLIC
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_link_libraries(shared_control_visualization_node
  ${cpp_typesupport_target}
)

target_link_libraries(save_goal_manager
  ${cpp_typesupport_target}
)

add_dependencies(shared_control_visualization_node
  ${PROJECT_NAME}__rosidl_generator_cpp
)

add_dependencies(visual_servoing_node
  ${PROJECT_NAME}__rosidl_generator_cpp
)

add_dependencies(save_goal_manager
  ${PROJECT_NAME}__rosidl_generator_cpp
)

install(TARGETS
  visual_servoing_node
  handeye_tf_publisher
  shared_control_visualization_node
  save_goal_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
