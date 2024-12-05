cmake_minimum_required(VERSION 3.8)
project(module_6_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

# Add library for EKF
add_library(ekf_lib src/ekf_lib.cpp)
target_include_directories(ekf_lib PRIVATE ${EIGEN3_INCLUDE_DIR} include)
target_link_libraries(ekf_lib Eigen3::Eigen)

# Add library for EKF
add_library(task3_lib src/task3_lib.cpp)
target_include_directories(task3_lib PRIVATE ${EIGEN3_INCLUDE_DIR} include)
target_link_libraries(task3_lib Eigen3::Eigen)

# Add executable for Task1 node
add_executable(task1 src/task1.cpp)
target_include_directories(task1 PRIVATE include)
target_link_libraries(task1 ekf_lib)
ament_target_dependencies(task1 rclcpp sensor_msgs nav_msgs visualization_msgs)

# Add executable for Task3 node
add_executable(task3 src/task3.cpp)
target_include_directories(task3 PRIVATE include)
target_link_libraries(task3 task3_lib)
ament_target_dependencies(task3 rclcpp sensor_msgs nav_msgs visualization_msgs tf2)

add_executable(task3b src/task3b.cpp)
target_include_directories(task3b PRIVATE include)
target_link_libraries(task3b task3_lib)
ament_target_dependencies(task3b rclcpp sensor_msgs nav_msgs visualization_msgs tf2)

install(TARGETS
  task1 task3 task3b
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
