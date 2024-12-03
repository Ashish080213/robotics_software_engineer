cmake_minimum_required(VERSION 3.8)
project(module_5_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)

add_executable(task1 src/task1.cpp)
add_executable(task2 src/task2.cpp)

add_library(lqr_lib STATIC src/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

add_executable(task4 src/task4.cpp)
target_link_libraries(task4 lqr_lib Eigen3::Eigen)
ament_target_dependencies(task4 rclcpp nav_msgs geometry_msgs tf2 angles)
target_include_directories(task4 PRIVATE ${EIGEN_INCLUDE_DIR} include)

ament_target_dependencies(task1 rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(task1 ${OpenCV_LIBS})
ament_target_dependencies(task2 rclcpp geometry_msgs nav_msgs tf2)

install(TARGETS
  task1 task2 lqr_lib task4
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
