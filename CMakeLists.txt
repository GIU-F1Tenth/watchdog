cmake_minimum_required(VERSION 3.8)
project(watchdog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SanityWarning.msg"
  "msg/SensorHealth.msg"
  "msg/SanitySummary.msg"
  DEPENDENCIES builtin_interfaces
)

# Install Python files manually
install(
  DIRECTORY watchdog/
  DESTINATION lib/python3.10/site-packages/watchdog
  FILES_MATCHING
  PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  watchdog/watchdog_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()