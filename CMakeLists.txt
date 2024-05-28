cmake_minimum_required(VERSION 3.16)
project(mujoco_ros_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error -Wno-dev)
endif()

# Add the directory containing your macros
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(FindOrFetch)

FindOrFetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  mujoco
  LIBRARY_NAME
  mujoco
  GIT_REPO
  https://github.com/google-deepmind/mujoco.git
  GIT_TAG
  3.1.1
  TARGETS
  mujoco::mujoco
  mujoco::platform_ui_adapter
  EXCLUDE_FROM_ALL
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# 1. build a static C++ library for python bindings to link against

# include python development headers
include_directories(${Python3_INCLUDE_DIRS})

# add a static library for python bindings to link against
add_library(mujoco_ros_envs STATIC 
	src/mujoco_ros.cpp
  )
target_include_directories(mujoco_ros_envs PUBLIC
	${PROJECT_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}
	)
target_link_libraries(mujoco_ros_envs PUBLIC
  mujoco::mujoco
  mujoco::platform_ui_adapter
	rclcpp::rclcpp 
  std_msgs::std_msgs__rosidl_generator_c
  sensor_msgs::sensor_msgs__rosidl_typesupport_c
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_c
  geometry_msgs::geometry_msgs__rosidl_generator_c
  rosgraph_msgs::rosgraph_msgs__rosidl_generator_c
  glfw
  ${OpenCV_LIBS}
)
set_target_properties(mujoco_ros_envs PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS mujoco_ros_envs
  DESTINATION lib/${PROJECT_NAME}
)

# 2. build python bindings
ament_python_install_package(mujoco_ros)

# Set the build location and install location for a CPython extension
function(configure_build_install_location library_name)
  install(TARGETS ${library_name} DESTINATION "${PYTHON_INSTALL_DIR}/mujoco_ros")
endfunction()

pybind11_add_module(franka_env 
  src/franka_table.cpp
  )
target_include_directories(mujoco_ros_envs PUBLIC
	${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(
  franka_env PRIVATE 
  mujoco_ros_envs 
  )
set_target_properties(franka_env PROPERTIES INSTALL_RPATH "${CMAKE_BINARY_DIR}/lib")
configure_build_install_location(franka_env)

ament_package()
