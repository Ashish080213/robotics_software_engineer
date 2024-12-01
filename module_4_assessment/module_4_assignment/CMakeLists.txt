cmake_minimum_required(VERSION 3.8)
project(module_4_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(task_1 src/task_1.cpp)
add_executable(task3b src/task3b.cpp)
add_executable(task3c src/task3c.cpp)

ament_target_dependencies(task3b rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(task3c rclcpp sensor_msgs geometry_msgs nav_msgs)
ament_target_dependencies(task_1 rclcpp sensor_msgs cv_bridge)
target_link_libraries(task_1 ${OpenCV_LIBS})

install(TARGETS
  task_1
  task3b
  task3c
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
