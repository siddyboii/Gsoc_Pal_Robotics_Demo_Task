cmake_minimum_required(VERSION 3.8)
project(dummy_sensor_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(dummy_sensor_plugin SHARED
  src/dummy_sensor_plugin.cpp
)

target_include_directories(dummy_sensor_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dummy_sensor_plugin
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
)

ament_target_dependencies(dummy_sensor_plugin
  rclcpp
  ros_gz_sim
  std_msgs
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

install(TARGETS dummy_sensor_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/dummy_sensor_plugin.sh.in")

ament_package()
