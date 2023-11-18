cmake_minimum_required(VERSION 3.5)
project(sub_pub_tester)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # 

# include
INCLUDE_DIRECTORIES(
    include
)

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryLevels.msg"
  "msg/Orientation.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# executables
add_executable(battery_pub src/battery_pub.cpp)
ament_target_dependencies(battery_pub
  rclcpp
  std_msgs
)

add_executable(orientation_pub src/orientation_pub.cpp)
ament_target_dependencies(orientation_pub
  rclcpp
  std_msgs
)

add_executable(speed_pub src/speed_pub.cpp)
ament_target_dependencies(speed_pub
  rclcpp
  std_msgs
)

rosidl_target_interfaces(battery_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(orientation_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
    battery_pub
    orientation_pub
    speed_pub
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
