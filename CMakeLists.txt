cmake_minimum_required(VERSION 3.8)
project(mujoco_ros_py)

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
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# 1. build a static library for python bindings

# configure mujoco linking
set(CMAKE_INSTALL_RPATH ${PROJECT_SOURCE_DIR}/mujoco-3.1.1/lib) # install rpath for mujoco shared libraries
include_directories(${PROJECT_SOURCE_DIR}/mujoco-3.1.1/include)
link_directories(${PROJECT_SOURCE_DIR}/mujoco-3.1.1/lib)

# add a static library for python bindings to link against
add_library(mujoco_ros_envs STATIC 
	src/mujoco_ros.cpp
  src/franka_table.cpp
  )
target_include_directories(mujoco_ros_envs PUBLIC
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/mujoco-3.1.1/include
	${OpenCV_INCLUDE_DIRS}
	)
target_link_libraries(mujoco_ros_envs PUBLIC
	rclcpp::rclcpp 
  std_msgs::std_msgs__rosidl_generator_c
  sensor_msgs::sensor_msgs__rosidl_typesupport_c
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_c
  geometry_msgs::geometry_msgs__rosidl_generator_c
  rosgraph_msgs::rosgraph_msgs__rosidl_generator_c
  glfw
  ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/mujoco-3.1.1/lib/libmujoco.so.3.1.1
)
set_target_properties(mujoco_ros_envs PROPERTIES POSITION_INDEPENDENT_CODE ON)
install(TARGETS mujoco_ros_envs
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

# 2. build python binding of C implementation

ament_python_install_package(mujoco_ros)

# Set the build location and install location for a CPython extension
function(configure_build_install_location library_name)
  install(TARGETS ${library_name} DESTINATION "${PYTHON_INSTALL_DIR}/mujoco_ros")
endfunction()

pybind11_add_module(franka_env 
  src/franka_bind.cpp
  )
target_link_libraries(
  franka_env PRIVATE mujoco_ros_envs
  )
configure_build_install_location(franka_env)

ament_package()
