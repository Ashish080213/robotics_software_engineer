cmake_minimum_required(VERSION 3.8)
project(module_8_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(include/module_8_assignment)

# Occupancy Grid
add_executable(occupancy_grid_node src/occupancy_grid.cpp)
ament_target_dependencies(occupancy_grid_node rclcpp nav_msgs tf2_ros)

# Astar Library
add_library(algo_astar src/algo_astar.cpp )
ament_target_dependencies(algo_astar rclcpp geometry_msgs nav_msgs)

# RRT Library
add_library(rrt_node src/algo_rrt_node.cpp)
add_library(algo_rrt src/algo_rrt.cpp)
target_link_libraries(algo_rrt rrt_node)

# RRT Star Library
add_library(rrtstar_node src/algo_rrtstar_node.cpp)
add_library(algo_rrtstar src/algo_rrtstar.cpp)
target_link_libraries(algo_rrtstar rrtstar_node)

# Planner Interface for Algorithms
add_executable(path_planning_node src/path_planning.cpp)
target_link_libraries(path_planning_node algo_astar algo_rrt)
ament_target_dependencies(path_planning_node rclcpp geometry_msgs nav_msgs)

# Planner2 Interface for Algorithms
add_executable(pp_node src/pp.cpp)
target_link_libraries(pp_node algo_astar algo_rrt algo_rrtstar)
ament_target_dependencies(pp_node rclcpp geometry_msgs nav_msgs)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  occupancy_grid_node
  path_planning_node
  pp_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)

  ament_add_gtest(test_rrtstar_node test/test_rrtstar_node.cpp)
  target_link_libraries(test_rrtstar_node rrtstar_node)

  ament_add_gtest(test_rrtstar test/test_rrtstar.cpp)
  target_link_libraries(test_rrtstar rrtstar_node algo_rrtstar)

  # ament_add_gtest(test_rrt_node test/test_rrt_node.cpp)
  # target_link_libraries(test_rrt_node rrt_node)

  # ament_add_gtest(test_rrt test/test_rrt.cpp)
  # target_link_libraries(test_rrt rrt_node algo_rrt)

  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
