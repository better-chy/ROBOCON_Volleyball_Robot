cmake_minimum_required(VERSION 3.8)
project(hit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_library(GPIOD_LIBRARY gpiod REQUIRED)

add_executable(infrared_trigger_node src/infrared_trigger_node.cpp)
target_compile_features(infrared_trigger_node PUBLIC cxx_std_17)
target_link_libraries(infrared_trigger_node ${GPIOD_LIBRARY})
ament_target_dependencies(
  infrared_trigger_node
  rclcpp
  std_msgs
)

add_executable(joy_hit_trigger_node src/joy_hit_trigger_node.cpp)
target_compile_features(joy_hit_trigger_node PUBLIC cxx_std_17)
ament_target_dependencies(
  joy_hit_trigger_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS infrared_trigger_node joy_hit_trigger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
