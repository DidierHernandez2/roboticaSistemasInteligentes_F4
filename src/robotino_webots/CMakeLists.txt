cmake_minimum_required(VERSION 3.8)
project(robotino_webots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(rclcpp REQUIRED)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files so `ros2 launch robotino_webots ...` works
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# (Optional) install assets so package:// resolves (uncomment if you have them)
# install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY rviz/  DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY protos/ DESTINATION share/${PROJECT_NAME})

# Install your Python ROS node (NOT the Webots controller)
install(PROGRAMS
  src/robotino_webots_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)


install(
  DIRECTORY controllers/
  DESTINATION share/${PROJECT_NAME}/controllers
)


ament_package()
