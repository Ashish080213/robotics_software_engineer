cmake_minimum_required(VERSION 3.8)
project(module_2_assignment)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(task1 src/task1.cpp)
add_executable(task1b src/task1b.cpp)
add_executable(task2 src/task2.cpp)
add_executable(task3 src/task3.cpp)
add_executable(task4 src/task4.cpp)

ament_target_dependencies(task1 rclcpp std_msgs geometry_msgs)
ament_target_dependencies(task1b rclcpp std_msgs geometry_msgs)
ament_target_dependencies(task2 rclcpp std_msgs geometry_msgs)
ament_target_dependencies(task3 rclcpp std_msgs geometry_msgs)
ament_target_dependencies(task4 rclcpp std_msgs geometry_msgs)

install(TARGETS
  task1
  task1b
  task2
  task3
  task4
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
