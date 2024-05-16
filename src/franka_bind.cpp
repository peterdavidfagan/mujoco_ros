#include <pybind11/pybind11.h>
#include "mujoco_ros.hpp"
#include "franka_table.hpp"

namespace py = pybind11;

PYBIND11_MODULE(franka_env, m)
{
    m.doc() = R"(
            Python bindings for moveit_ros environments for the purpose of supporting the python interactive viewer.
            )";
            
    // construct a python class for franka ros simulation instance  
    py::class_<mujoco_ros::FrankaMJROS, std::shared_ptr<mujoco_ros::FrankaMJROS>>(m, "FrankaEnv", R"(
        A class to encapsulate ROS compatible mujoco simulation environment.
        )")

        .def(py::init([](py::object model, py::object data){
            rclcpp::init(0, nullptr);

            std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
                std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            auto custom_deleter = [executor](mujoco_ros::FrankaMJROS* franka_env) {
                executor->cancel();
                rclcpp::shutdown();
                delete franka_env;
            };
            std::shared_ptr<mujoco_ros::FrankaMJROS> node(new mujoco_ros::FrankaMJROS(model, data), custom_deleter);

            auto spin_node = [node, executor]() {
            executor->add_node(node);
            executor->spin();
            };

            std::thread execution_thread(spin_node);
            execution_thread.detach();

            return node;
        }),
        py::arg("model"),
        py::arg("data"),
        py::return_value_policy::take_ownership,
        R"(Initialize mujoco ros franka simulation instance)")
        .def_property("is_syncing", &mujoco_ros::FrankaMJROS::getSync, &mujoco_ros::FrankaMJROS::setSync);

}
