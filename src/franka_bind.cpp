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
    py::class_<mujoco_ros::FrankaMJROS, std::shared_ptr<mujoco_ros::FrankaMJROS>>(m, "FrankaMJROS", R"(
        A class to encapsulate ROS compatible mujoco simulation environment.
        )")

        .def(py::init([](const std::string& node_name, 
                         const std::vector<std::string>& launch_params_filepaths, 
                         const py::object& config_dict){
            
            // This section is used to load the appropriate node parameters before spinning a simulation instance
            // Priority is given to parameters supplied directly via a config_dict, followed by launch parameters
            // and finally no supplied parameters.
            std::vector<std::string> launch_arguments;
            if (!config_dict.is(py::none()))
            {
            auto utils = py::module::import("mujoco_ros.utils");
            std::string params_filepath =
                utils.attr("create_params_file_from_dict")(config_dict, node_name).cast<std::string>();
            launch_arguments = { "--ros-args", "--params-file", params_filepath };
            }
            else if (!launch_params_filepaths.empty())
            {
            launch_arguments = { "--ros-args" };
            for (const auto& launch_params_filepath : launch_params_filepaths)
            {
                launch_arguments.push_back("--params-file");
                launch_arguments.push_back(launch_params_filepath);
            }
            }

            // Initialize ROS, pass launch arguments with rclcpp::init()
            if (!rclcpp::ok())
            {
            std::vector<const char*> chars;
            chars.reserve(launch_arguments.size());
            for (const auto& arg : launch_arguments)
            {
                chars.push_back(arg.c_str());
            }

            rclcpp::init(launch_arguments.size(), chars.data());
            // RCLCPP_INFO(getLogger(), "Initialize rclcpp");
            }

            // Build NodeOptions
            // RCLCPP_INFO(getLogger(), "Initialize node parameters");
            rclcpp::NodeOptions node_options;
            node_options.allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)
                .arguments(launch_arguments);

            // RCLCPP_INFO(getLogger(), "Initialize node and executor");
            std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
                std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            auto custom_deleter = [executor](mujoco_ros::FrankaMJROS* franka_env) {
                executor->cancel();
                rclcpp::shutdown();
                delete franka_env;
            };
            std::shared_ptr<mujoco_ros::FrankaMJROS> node(new mujoco_ros::FrankaMJROS(), custom_deleter);

            // RCLCPP_INFO(getLogger(), "Spin separate thread");
            auto spin_node = [node, executor]() {
            executor->add_node(node);
            executor->spin();
            };
            std::thread execution_thread(spin_node);
            execution_thread.detach();

            return node;
        }),
        py::arg("node_name")="franka_mj_env",
        py::arg("launch_params_filepaths") = utils.attr("get_launch_params_filepaths")().cast<std::vector<std::string>>(),
        py::arg("config_dict") = py::none(),
        py::return_value_policy::take_ownership,
        R"(Initialize mujoco ros franka simulation instance)");
        // .def_property_readonly("model", , )
        // .def_property_readonly("data", , );

    
    // test a basic function
    m.def("add", &add, R"(
            Add two numbers

            Some other explanation about the add function.
        )");
}
