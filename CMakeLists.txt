cmake_minimum_required(VERSION 3.8)
project(mujoco_ros)

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

# install rpath for shared libraries
set(CMAKE_INSTALL_RPATH ${PROJECT_SOURCE_DIR}/mujoco-3.1.1/lib)

# include mujoco headers
include_directories(${PROJECT_SOURCE_DIR}/mujoco-3.1.1/include)
link_directories(${PROJECT_SOURCE_DIR}/mujoco-3.1.1/lib)

# add mujoco ros as a library
add_library(mujoco_ros STATIC 
	src/mujoco_ros.cpp)
target_include_directories(mujoco_ros PUBLIC
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/mujoco-3.1.1/include
	${OpenCV_INCLUDE_DIRS}
	)
target_link_libraries(mujoco_ros PUBLIC
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

# add executable for basic table top environment with Franka Emika Panda
add_executable(franka src/franka.cpp)
target_link_libraries(franka PRIVATE
	mujoco_ros
	)

# Set the build location and install location for a CPython extension
function(configure_build_install_location library_name)
    set_target_properties(
    ${library_name}
    PROPERTIES # Use generator expression to avoid prepending a build type
               # specific directory on Windows
               LIBRARY_OUTPUT_DIRECTORY
               $<1:${CMAKE_CURRENT_BINARY_DIR}/test_mujoco_ros>
               RUNTIME_OUTPUT_DIRECTORY
               $<1:${CMAKE_CURRENT_BINARY_DIR}/test_mujoco_ros>)
  install(TARGETS ${library_name} DESTINATION "${PYTHON_INSTALL_DIR}/mujoco_ros")
endfunction()

pybind11_add_module(franka_env src/franka_bind.cpp)
configure_build_install_location(franka_env)

install(TARGETS mujoco_ros franka
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
