cmake_minimum_required(VERSION 3.5)
project(motor_drive)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(motor_drive src/motor_drive.cpp)
add_executable(speed_filtering src/speed_filtering.cpp)
add_executable(joy_cmd src/joy_cmd.cpp)


ament_target_dependencies(
  motor_drive
  rclcpp
  std_msgs
  serial
  geometry_msgs
)
ament_target_dependencies(
  speed_filtering
  rclcpp
  std_msgs
  serial
  geometry_msgs
)
ament_target_dependencies(
  joy_cmd
  rclcpp
  std_msgs
  serial
  sensor_msgs 
  geometry_msgs
)
install(TARGETS 
  motor_drive
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS 
  speed_filtering
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS 
  joy_cmd
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
