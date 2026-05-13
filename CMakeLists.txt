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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedGoal.msg"
  "msg/DetectedGoalArray.msg"
  "srv/SaveCurrentTagGoal.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(visual_servoing_node
  src/visual_servoing_node.cpp
  src/visual_servoing_display.cpp
  src/camera_config.cpp
  src/apriltag_pose_estimator.cpp
  src/pose_transformer.cpp
  src/detected_goal_builder.cpp
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

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

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

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

target_link_libraries(visual_servoing_node
  apriltag
  ${OpenCV_LIBRARIES}
  ${cpp_typesupport_target}
)
add_dependencies(visual_servoing_node
  ${PROJECT_NAME}__rosidl_generator_cpp
)

install(TARGETS
  visual_servoing_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
