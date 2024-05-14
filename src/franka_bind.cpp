#include <pybind11/pybind11.h>
#include "mujoco_ros.hpp"
#include "franka_table.hpp"

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(franka_env, m)
{
    m.doc() = R"(
            Python bindings for moveit_ros environments for the purpose of supporting the python interactive viewer.
            )";

    auto utils = py::module::import("mujoco_ros.utils");

    // construct a python class for franka ros simulation instance  
    py::class_<mujoco_ros::FrankaMJROS, std::shared_ptr<mujoco_ros::FrankaMJROS>>(m, "FrankaEnv", R"(
        A class to encapsulate ROS compatible mujoco simulation environment.
        )")

        .def(py::init([](){
            rclcpp::init(0, nullptr);

            std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
                std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            auto custom_deleter = [executor](mujoco_ros::FrankaMJROS* franka_env) {
                executor->cancel();
                rclcpp::shutdown();
                delete franka_env;
            };
            std::shared_ptr<mujoco_ros::FrankaMJROS> node(new mujoco_ros::FrankaMJROS(), custom_deleter);

            auto spin_node = [node, executor]() {
            executor->add_node(node);
            executor->spin();
            };

            std::thread execution_thread(spin_node);
            execution_thread.detach();

            return node;
        }),
        py::return_value_policy::take_ownership,
        R"(Initialize mujoco ros franka simulation instance)")
        .def_property_readonly("model", &mujoco_ros::FrankaMJROS::get_model)
        .def_property_readonly("data", &mujoco_ros::FrankaMJROS::get_data);

    
    // test a basic function
    m.def("add", &add, R"(
            Add two numbers

            Some other explanation about the add function.
        )");
}
