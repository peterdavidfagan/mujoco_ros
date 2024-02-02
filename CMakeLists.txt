cmake_minimum_required(VERSION 3.8)
project(mujoco_ros_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# include mujoco headers
include_directories(${PROJECT_SOURCE_DIR}/mujoco-3.1.1/include)

# add executable for franka simulation
add_executable(franka_block src/franka_block.cpp)
target_link_libraries(
	franka_block 
	rclcpp::rclcpp 
	std_msgs::std_msgs__rosidl_generator_c
	sensor_msgs::sensor_msgs__rosidl_generator_c
	geometry_msgs::geometry_msgs__rosidl_generator_c
	rosgraph_msgs::rosgraph_msgs__rosidl_generator_c
	${PROJECT_SOURCE_DIR}/mujoco-3.1.1/lib/libmujoco.so.3.1.1
	)
#ament_target_dependencies(franka_block rclcpp std_msgs mujoco)

install(TARGETS franka_block
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
