cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(filter_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
add_executable(kalman_filter_service
  src/kalman_filter_service.cpp
  ../../kalman-filter-no-ros/kalman_filter.cpp
)
ament_target_dependencies(kalman_filter_service rclcpp filter_interface)

add_executable(kalman_filter_client
  src/kalman_filter_client.cpp
)
ament_target_dependencies(kalman_filter_client rclcpp filter_interface)

target_include_directories(kalman_filter_service PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_include_directories(kalman_filter_client PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

install(TARGETS
  kalman_filter_service
  kalman_filter_client
  DESTINATION lib/${PROJECT_NAME})


ament_package()
