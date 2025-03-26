cmake_minimum_required(VERSION 3.8)
project(boson_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

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

add_executable(boson_camera_node src/boson_camera_node.cpp)
add_executable(boson_viewer_node src/boson_viewer_node.cpp)
ament_target_dependencies(boson_camera_node rclcpp sensor_msgs OpenCV)
ament_target_dependencies(boson_viewer_node rclcpp sensor_msgs OpenCV)

add_executable(bayer_to_rgb_node src/bayer_to_rgb_node.cpp)
ament_target_dependencies(bayer_to_rgb_node rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS boson_camera_node boson_viewer_node bayer_to_rgb_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
