cmake_minimum_required(VERSION 3.5)
project(gsot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # # --- # 

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pub_tester REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# include
INCLUDE_DIRECTORIES(
    include
    ${OpenCV_INCLUDE_DIRS}
)

# executables
add_executable(metric_data_transmitter src/metric_data_transmitter.cpp src/tcp_client.cpp)
ament_target_dependencies(metric_data_transmitter rclcpp std_msgs pub_tester)

add_executable(camera_data_transmitter src/camera_data_transmitter.cpp src/udp_client.cpp)
ament_target_dependencies(camera_data_transmitter 
    rclcpp 
    pub_tester
    image_transport
    cv_bridge
    sensor_msgs
)
target_link_libraries(camera_data_transmitter
  ${OpenCV_LIBRARIES}
)

add_executable(config_data_transmitter src/config_data_transmitter.cpp src/tcp_client.cpp)
ament_target_dependencies(config_data_transmitter rclcpp)

install(TARGETS
  metric_data_transmitter
  camera_data_transmitter
  config_data_transmitter
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
