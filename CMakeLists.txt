cmake_minimum_required(VERSION 3.8)
project(omni_bot2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Install Python executables
install(DIRECTORY config scripts launch maps mesh urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller.py
  scripts/move_robot.py
  scripts/old_controller.py
  scripts/new_controller.py
  scripts/odom.py
  scripts/odom_50hz.py
  scripts/teensy_imu.py
  scripts/odom_v2.py
  scripts/dock_pose_pub.py
  scripts/docking_controller.py
  scripts/arm_driver.py
  scripts/voice_serv.py
  
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
