cmake_minimum_required(VERSION 3.8)
project(limo_visions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport)



add_executable(move_to_ar src/move_to_ar.cpp)
ament_target_dependencies(move_to_ar
  "cv_bridge"
  "geometry_msgs"
  "std_msgs"
  "rclcpp"
  "sensor_msgs"
  )
  install(
  TARGETS move_to_ar
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(object_detect src/object_detect.cpp)
ament_target_dependencies(object_detect 
"cv_bridge"
"geometry_msgs"
"std_msgs"
"rclcpp"
"sensor_msgs"
"image_transport")
install(TARGETS object_detect RUNTIME DESTINATION lib/${PROJECT_NAME})


add_executable(detect_line src/detect_line.cpp)
ament_target_dependencies(detect_line 
"cv_bridge"
"geometry_msgs"
"std_msgs"
"rclcpp"
"sensor_msgs"
"image_transport")
install(TARGETS detect_line RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(traffic_light src/traffic_light.cpp)
ament_target_dependencies(traffic_light 
"cv_bridge"
"geometry_msgs"
"std_msgs"
"rclcpp"
"sensor_msgs"
"image_transport")
install(TARGETS traffic_light RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport pluginlib rosidl_default_runtime rclcpp sensor_msgs)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the detecting line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the detecting line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
