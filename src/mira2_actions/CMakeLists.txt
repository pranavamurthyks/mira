cmake_minimum_required(VERSION 3.8)
project(mira2_actions)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(custom_msgs REQUIRED)

# Build bt_runner executable
add_executable(bt_runner
  src/bt_runner.cpp
  src/IsFlareVisible.cpp
  src/FlareAlignX.cpp    # Add your new BT node here
)

# Include directories
target_include_directories(bt_runner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ROS dependencies
ament_target_dependencies(bt_runner
  rclcpp
  geometry_msgs
  custom_msgs
)

# BehaviorTree.CPP library
target_link_libraries(bt_runner
  behaviortree_cpp::behaviortree_cpp
)

# Install executable
install(TARGETS
  bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()