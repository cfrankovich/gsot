cmake_minimum_required(VERSION 3.5)
project(sub_data_transmission)

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
find_package(sub_pub_tester REQUIRED)

# executables
add_executable(metric_data_transmitter src/metric_data_transmitter.cpp src/tcp_client.cpp)
ament_target_dependencies(metric_data_transmitter rclcpp std_msgs sub_pub_tester)

install(TARGETS
  metric_data_transmitter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
