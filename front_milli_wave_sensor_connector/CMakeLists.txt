cmake_minimum_required(VERSION 3.8)
project(front_milli_wave_sensor_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(front_milli_wave_sensor_msg REQUIRED)

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

add_executable(front_milli_wave_sensor_connector src/front_milli_wave_sensor_connector.cpp src/main.cpp)
ament_target_dependencies(front_milli_wave_sensor_connector rclcpp std_msgs front_milli_wave_sensor_msg visualization_msgs)
target_include_directories(front_milli_wave_sensor_connector
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
install(
  TARGETS front_milli_wave_sensor_connector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
 launch
 config
 DESTINATION share/${PROJECT_NAME}
)

ament_package()
