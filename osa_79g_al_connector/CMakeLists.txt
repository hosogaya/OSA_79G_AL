cmake_minimum_required(VERSION 3.8)
project(osa_79g_al_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(front_mill_wave_sensor_msg REQUIRED)

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

add_executable(osa_79g_al_connector src/osa_79g_al_connector.cpp src/main.cpp)
ament_target_dependencies(osa_79g_al_connector rclcpp std_msgs front_mill_wave_sensor_msg visualization_msgs)
target_include_directories(osa_79g_al_connector
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
install(
  TARGETS osa_79g_al_connector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
 launch
 config
 DESTINATION share/${PROJECT_NAME}
)

ament_package()
