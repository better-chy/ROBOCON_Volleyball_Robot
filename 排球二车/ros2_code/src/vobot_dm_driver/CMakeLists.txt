cmake_minimum_required(VERSION 3.8)
project(vobot_dm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dm_msgs REQUIRED)

add_executable(dm_motor_node src/dm_motor_node.cpp)
target_include_directories(dm_motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dm_motor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dm_motor_node
  "rclcpp"
  "std_msgs"
  "dm_msgs"
)

add_executable(dm_pitch_motor_node src/dm_pitch_motor_node.cpp)
target_include_directories(dm_pitch_motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dm_pitch_motor_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  dm_pitch_motor_node
  "rclcpp"
  "std_msgs"
)

add_executable(dm_hit_motor_node src/dm_hit_motor_node.cpp)
target_include_directories(dm_hit_motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dm_hit_motor_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  dm_hit_motor_node
  "rclcpp"
  "std_msgs"
)

install(TARGETS dm_motor_node dm_pitch_motor_node dm_hit_motor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

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
