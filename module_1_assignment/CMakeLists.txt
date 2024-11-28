cmake_minimum_required(VERSION 3.8)
project(module_1_assignment)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_executable(task1 src/task1.cpp)
add_executable(task2 src/task2.cpp)
add_executable(task3 src/task3_main.cpp src/task3.cpp)
add_executable(task3b src/task3b_main.cpp src/task3b.cpp)

# install(TARGETS
#   task1
#   task2
#   task3
#   task3b
#   DESTINATION /home/ashish/assignment_ws/src/robotics_software_engineer/module_1_assignment/build)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
