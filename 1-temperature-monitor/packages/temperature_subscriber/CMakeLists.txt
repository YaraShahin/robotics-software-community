cmake_minimum_required(VERSION 3.22)
project(temperature_subscriber)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPS
  rclpy
  rclcpp
  std_msgs
  temperature_monitoring_interfaces
)

foreach(dep ${THIS_PACKAGE_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${temperature_monitoring_interfaces_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_lib
  ${rclcpp_INCLUDE_DIRS}
  src/code.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  ${THIS_PACKAGE_DEPS}
)

# Add executables
add_executable(${PROJECT_NAME}_node src/code_node.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Add your tests here
endif()

ament_package()
