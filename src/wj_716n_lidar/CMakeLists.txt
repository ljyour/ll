cmake_minimum_required(VERSION 3.8)
project(wj_716n_lidar)

## Compile as C++17, supported in ROS 2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# 这里移动 find_package(ament_lint_auto REQUIRED) 到 ament_package() 之前
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # Skip copyright check (if necessary)
  set(ament_cmake_cpplint_FOUND TRUE) # Skip cpplint (if necessary)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(
  include   # Include the directory containing header files
)

# Declare ROS messages, services, and actions (optional if needed)
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
# )

# add_action_files(
#   FILES
#   Action1.action
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# Declare the package (no need for catkin_package here)
ament_package()

###########
## Build ##
###########

# Specify additional locations of header files
add_executable(wj_716n_lidar
   src/wj_716n_lidar_01.cpp src/async_client.cpp src/wj_716n_lidar_protocol.cpp )

# Include directories for target
target_include_directories(wj_716n_lidar PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

# Set target compile features
target_compile_features(wj_716n_lidar PUBLIC cxx_std_17)  # Require C++17

# Specify the dependencies for your target
ament_target_dependencies(wj_716n_lidar
  "rclcpp"
  "std_msgs"
  "rcl_interfaces"
  "sensor_msgs"
  "Boost"
)

# Install executable
install(TARGETS wj_716n_lidar
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
