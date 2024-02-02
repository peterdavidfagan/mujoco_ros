cmake_minimum_required(VERSION 3.8)
project(mujoco_ros_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(MUJOCO_LIB_FOLDER ${PROJECT_SOURCE_DIR}/mujoco-3.1.1/lib)
find_library(mujoco libmujoco.so.3.1.1 PATHS ${MUJOCO_LIB_FOLDER})

add_executable(franka_block src/franka_block.cpp)
ament_target_dependencies(franka_block rclcpp std_msgs)
target_link_libraries(franka_block mujoco)

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

ament_package()
