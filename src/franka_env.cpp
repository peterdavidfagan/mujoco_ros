#include <chrono>
#include <stdexcept>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <thread>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rmw/types.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <pybind11/pybind11.h>

#include "mujoco_ros.hpp"
#include "franka_env.hpp"


namespace py = pybind11;
using namespace std::chrono_literals;

namespace franka_env
{

FrankaEnvBase::FrankaEnvBase(
  py::object model, 
  py::object data, 
  const std::string command_interface,
  int control_steps,
  double control_timer_freq) : mujoco_ros::MJROS()
{
  // cast Python model and data instances
  std::uintptr_t m_raw = model.attr("_address").cast<std::uintptr_t>();
  std::uintptr_t d_raw = data.attr("_address").cast<std::uintptr_t>();
  m = reinterpret_cast<mjModel*>(m_raw);
  d = reinterpret_cast<mjData*>(d_raw);

  this->control_steps = control_steps;

  /*
  * There is a mismatch between mujoco menagerie joint names and those
  * used in the robotiq ROS package. Here we map the mujoco joint names
  * at initialization to those used by ROS and use this when publishing
  * the robot state. This mapping should be moved to config file.
  * Also note the panda joint mapping aligns with my fork of mujoco_menagerie
  * I plan to update this mapping to be consistent with the upstream version.
  */
  std::map<std::string, std::string> joint_name_map;
  joint_name_map["joint1"] = "panda_joint1";
  joint_name_map["joint2"] = "panda_joint2";
  joint_name_map["joint3"] = "panda_joint3";
  joint_name_map["joint4"] = "panda_joint4";
  joint_name_map["joint5"] = "panda_joint5";
  joint_name_map["joint6"] = "panda_joint6";
  joint_name_map["joint7"] = "panda_joint7";
  joint_name_map["robotiq_2f85_right_driver_joint"] = "robotiq_85_right_knuckle_joint";
  joint_name_map["robotiq_2f85_right_coupler_joint"] = "robotiq_85_right_finger_joint";
  joint_name_map["robotiq_2f85_right_spring_link_joint"] = "robotiq_85_right_inner_knuckle_joint";
  joint_name_map["robotiq_2f85_right_follower_joint"] = "robotiq_85_right_finger_tip_joint";
  joint_name_map["robotiq_2f85_left_driver_joint"] = "robotiq_85_left_knuckle_joint";
  joint_name_map["robotiq_2f85_left_coupler_joint"] = "robotiq_85_left_finger_joint";
  joint_name_map["robotiq_2f85_left_spring_link_joint"] = "robotiq_85_left_inner_knuckle_joint";
  joint_name_map["robotiq_2f85_left_follower_joint"] = "robotiq_85_left_finger_tip_joint";
  for (int i = 0; i < m->nq; i++)
  {
    const char* joint_name = mj_id2name(m, mjOBJ_JOINT, i);
    if (joint_name != NULL)
    {
      std::string joint_name_str(joint_name);
      if (joint_name_str.find("panda") == std::string::npos)
      {
        continue;
      }
      else
      {
        size_t first_slash = joint_name_str.find("/");
        std::string parsed_joint_name = joint_name_str.substr(first_slash + 1);
        std::replace(parsed_joint_name.begin(), parsed_joint_name.end(), '/', '_');
        joint_names.push_back(joint_name_map[parsed_joint_name]);
      }
    }
  }

  // ROS 2 config
  static const rmw_qos_profile_t rmw_qos_profile_reliable = { RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                              10,
                                                              RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                              RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                                              RMW_QOS_DEADLINE_DEFAULT,
                                                              RMW_QOS_LIFESPAN_DEFAULT,
                                                              RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                                              RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                                              false };
  auto qos_reliable =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_reliable), rmw_qos_profile_reliable);
  physics_step_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // joint state and camera publishers
  rclcpp::PublisherOptions physics_publisher_options;
  physics_publisher_options.callback_group = physics_step_callback_group_;
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_states", qos_reliable,
                                                                                physics_publisher_options);

  // TODO: tune the controller update rate based on real-time factor
  // control interfaces and physics timestepping
  if (command_interface == "effort") 
  {
    joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/panda/panda_arm_joint_commands", qos_reliable,
        std::bind(&FrankaEnvBase::effort_joint_command_callback, this, std::placeholders::_1));
  }
  else if (command_interface == "position") 
  {
    joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/panda/panda_arm_joint_commands", qos_reliable,
        std::bind(&FrankaEnvBase::position_joint_command_callback, this, std::placeholders::_1));
  }
  else 
  {
    throw std::runtime_error("Command interface not supported.");
  }

  // for now gripper is position interface
  robotiq_joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/robotiq/robotiq_joint_commands", qos_reliable,
        std::bind(&FrankaEnvBase::robotiq_joint_command_callback, this, std::placeholders::_1));

  // timer to step simulation
  timer_ = this->create_wall_timer(std::chrono::duration<double>(control_timer_freq), std::bind(&FrankaEnvBase::step, this), physics_step_callback_group_);
}

void FrankaEnvBase::setSync(const bool &status) {
  is_syncing = status;

  if (status)
    {
      // sleep to ensure all pre-sync callbacks are completed
      // otherwise a segfault will be encountered
      std::this_thread::sleep_for(4ms);
    }
  }
bool FrankaEnvBase::getSync() {return this->is_syncing;}

void FrankaEnvBase::reset()
{
  std::lock_guard lock(physics_data_mutex);

  // set initial robot configuration and control signal
  mju_copy(d->qpos, init_qpos, 7);
  mju_copy(d->ctrl, current_ctrl, 8);
  mj_forward(m, d);
}

void FrankaEnvBase::step()
{ 
  if (this->is_syncing) // do nothing if syncing interactive viewer 
  {
    // don't apply control
  }
  else if (!this->is_running) // do nothing if controller isn't running 
  { 
    // don't apply control
  }
  else // apply control and step simulation
  {
    std::lock_guard lock(physics_data_mutex);
    d->ctrl = this->current_ctrl;
    for (int i = 0; i < this->control_steps; i++){
      mj_step(m, d);
    }
  }

  // publish joint state of franka emika panda
  rclcpp::Time current_time = this->now();
  auto joint_state = sensor_msgs::msg::JointState();
  joint_state.header.stamp = current_time;
  for (int i = 0; i < joint_names.size(); i++)
  {
    joint_state.name.push_back(joint_names[i]);
    joint_state.position.push_back(d->qpos[i]);
    joint_state.velocity.push_back(d->qvel[i]);
  }
  joint_state_publisher_->publish(joint_state);
}

void FrankaEnvBase::effort_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(physics_data_mutex);
  this->is_running = true;
  for (int i = 0; i < 7; i++)
  {
    current_ctrl[i] = msg->effort[i];
  }
}

void FrankaEnvBase::position_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(physics_data_mutex);
  this->is_running = true;
  for (int i = 0; i < 7; i++)
  {
    current_ctrl[i] = msg->position[i];
  }
}

void FrankaEnvBase::robotiq_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(physics_data_mutex);
  // custom mapping required see below link for details of MuJoCo implementation
  // https://github.com/google-deepmind/mujoco_menagerie/blob/8ef01e87fffaa8ec634a4826c5b2092733b2f3c8/robotiq_2f85/2f85.xml#L180
  current_ctrl[7] = msg->position[0] / (0.8 / 255);
}

FrankaEnvBlocks::FrankaEnvBlocks(
  py::object model, 
  py::object data, 
  const std::string command_interface,
  int control_steps,
  double control_timer_freq) : FrankaEnvBase(model, data, command_interface, control_steps, control_timer_freq)
{
  render_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  camera_publisher = this->create_publisher<sensor_msgs::msg::Image>("/overhead_camera", 10);

  // start rendering thread
  m_render = mj_copyModel(m_render, m);
  d_render = mj_copyData(d_render, m, d);
  this->start_render_thread();
}

void FrankaEnvBlocks::reset()
{
  std::lock_guard lock(physics_data_mutex);

  // set initial robot configuration and control signal
  mju_copy(d->qpos, init_qpos, 7);
  mju_copy(d->ctrl, current_ctrl, 8);

  // TODO: set the block positions

  mj_forward(m, d);
}

void FrankaEnvBlocks::start_render_thread()
{
  render_thread = std::thread([this]() {
    // init GLFW
    if (!glfwInit())
    {
      mju_error("Could not initialize GLFW");
    }
    glfwWindowHint(GLFW_VISIBLE, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(640, 480, "Invisible window", NULL, NULL);
    if (!window)
    {
      mju_error("Could not create GLFW window");
    }
    glfwMakeContextCurrent(window);

    // init render context and scene
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjv_makeScene(m_render, &scn, 2000);
    mjr_defaultContext(&con);
    mjr_makeContext(m_render, &con, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &con);

    viewport = mjr_maxViewport(&con);
    int W = viewport.width;
    int H = viewport.height;

    // allocate buffers for rendering
    camera_rgb = (unsigned char*)std::malloc(3 * W * H);

    while (true)
    {
      bool syncing;
      {
        std::lock_guard<std::mutex> lock(physics_data_mutex);
        syncing = this->is_syncing;
      }

      if (syncing) // check if syncing interactive viewer 
      {
        // do nothing
      }
      else
      {
        // copy data from sim to render buffer
        {
          std::lock_guard lock(physics_data_mutex);
          d_render = mj_copyData(d_render, m, d);
        }
      }
      
      // publish messages
      this->render_single_camera(&camera, camera_rgb, camera_publisher);
    }
  });
}

FrankaEnvApples::FrankaEnvApples(
  py::object model, 
  py::object data, 
  const std::string command_interface,
  int control_steps,
  double control_timer_freq) : FrankaEnvBase(model, data, command_interface, control_steps, control_timer_freq)
{
  render_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  camera_publisher = this->create_publisher<sensor_msgs::msg::Image>("/overhead_camera", 10);

  // start rendering thread
  m_render = mj_copyModel(m_render, m);
  d_render = mj_copyData(d_render, m, d);
  this->start_render_thread();
}

void FrankaEnvApples::reset()
{
  std::lock_guard lock(physics_data_mutex);

  // set initial robot configuration and control signal
  mju_copy(d->qpos, init_qpos, 7);
  mju_copy(d->ctrl, current_ctrl, 8);

  // set the apple positions
  mjtNum* apple_1 = new mjtNum[7]{ 0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0};
  mjtNum* apple_2 = new mjtNum[7]{ 0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0 };
  mju_copy(d->qpos+15, apple_1, 7);
  mju_copy(d->qpos+22, apple_2, 7);
  mj_forward(m, d);
}

void FrankaEnvApples::start_render_thread()
{
  render_thread = std::thread([this]() {
    // init GLFW
    if (!glfwInit())
    {
      mju_error("Could not initialize GLFW");
    }
    glfwWindowHint(GLFW_VISIBLE, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(640, 480, "Invisible window", NULL, NULL);
    if (!window)
    {
      mju_error("Could not create GLFW window");
    }
    glfwMakeContextCurrent(window);

    // init render context and scene
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjv_makeScene(m_render, &scn, 2000);
    mjr_defaultContext(&con);
    mjr_makeContext(m_render, &con, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &con);

    viewport = mjr_maxViewport(&con);
    int W = viewport.width;
    int H = viewport.height;

    // allocate buffers for rendering
    camera_rgb = (unsigned char*)std::malloc(3 * W * H);

    while (true)
    {
      bool syncing;
      {
        std::lock_guard<std::mutex> lock(physics_data_mutex);
        syncing = this->is_syncing;
      }

      if (syncing) // check if syncing interactive viewer 
      {
        // do nothing
      }
      else
      {
        // copy data from sim to render buffer
        {
          std::lock_guard lock(physics_data_mutex);
          d_render = mj_copyData(d_render, m, d);
        }
      }
      
      // publish messages
      this->render_single_camera(&camera, camera_rgb, camera_publisher);
    }
  });

}

}

PYBIND11_MODULE(franka_env, m)
{
    m.doc() = R"(
            Python bindings for moveit_ros environments for the purpose of supporting the python interactive viewer.
            )";
            
    // construct a python class for franka ros simulation instance  
    py::class_<franka_env::FrankaEnvBase, std::shared_ptr<franka_env::FrankaEnvBase>>(m, "FrankaBase", R"(
        A class to encapsulate ROS compatible mujoco simulation environment.
        )")

        .def(py::init([](py::object model, py::object data, const std::string command_interface, int control_steps, double control_timer_freq){
            rclcpp::init(0, nullptr);

            std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
                std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            auto custom_deleter = [executor](franka_env::FrankaEnvBase* franka_env) {
                executor->cancel();
                rclcpp::shutdown();
                delete franka_env;
            };
            std::shared_ptr<franka_env::FrankaEnvBase> node(new franka_env::FrankaEnvBase(model, data, command_interface, control_steps, control_timer_freq), custom_deleter);

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
        py::arg("command_interface").none(true) = "effort",
        py::arg("control_steps").none(true) = 10,
        py::arg("control_timer_freq").none(true) = 1e-2,
        py::return_value_policy::take_ownership,
        R"(Initialize mujoco ros franka simulation instance)")
        .def_property("is_syncing", &franka_env::FrankaEnvBase::getSync, &franka_env::FrankaEnvBase::setSync)
        .def("reset", &franka_env::FrankaEnvBase::reset);


    // construct a python class for franka ros simulation instance  
    py::class_<franka_env::FrankaEnvBlocks, std::shared_ptr<franka_env::FrankaEnvBlocks>>(m, "FrankaBlocks", R"(
        A class to encapsulate ROS compatible mujoco simulation environment.
        )")

        .def(py::init([](py::object model, py::object data, const std::string command_interface, int control_steps, double control_timer_freq){
            rclcpp::init(0, nullptr);

            std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
                std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            auto custom_deleter = [executor](franka_env::FrankaEnvBlocks* franka_env) {
                executor->cancel();
                rclcpp::shutdown();
                delete franka_env;
            };
            std::shared_ptr<franka_env::FrankaEnvBlocks> node(new franka_env::FrankaEnvBlocks(model, data, command_interface, control_steps, control_timer_freq), custom_deleter);

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
        py::arg("command_interface").none(true) = "effort",
        py::arg("control_steps").none(true) = 10,
        py::arg("control_timer_freq").none(true) = 1e-2,
        py::return_value_policy::take_ownership,
        R"(Initialize mujoco ros franka simulation instance)")
        .def_property("is_syncing", &franka_env::FrankaEnvBlocks::getSync, &franka_env::FrankaEnvBlocks::setSync)
        .def("reset", &franka_env::FrankaEnvBlocks::reset);


    // construct a python class for franka ros simulation instance  
    py::class_<franka_env::FrankaEnvApples, std::shared_ptr<franka_env::FrankaEnvApples>>(m, "FrankaApples", R"(
        A class to encapsulate ROS compatible mujoco simulation environment.
        )")

        .def(py::init([](py::object model, py::object data, const std::string command_interface, int control_steps, double control_timer_freq){
            rclcpp::init(0, nullptr);

            std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
                std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            auto custom_deleter = [executor](franka_env::FrankaEnvApples* franka_env) {
                executor->cancel();
                rclcpp::shutdown();
                delete franka_env;
            };
            std::shared_ptr<franka_env::FrankaEnvApples> node(new franka_env::FrankaEnvApples(model, data, command_interface, control_steps, control_timer_freq), custom_deleter);

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
        py::arg("command_interface").none(true) = "effort",
        py::arg("control_steps").none(true) = 10,
        py::arg("control_timer_freq").none(true) = 1e-2,
        py::return_value_policy::take_ownership,
        R"(Initialize mujoco ros franka simulation instance)")
        .def_property("is_syncing", &franka_env::FrankaEnvApples::getSync, &franka_env::FrankaEnvApples::setSync)
        .def("reset", &franka_env::FrankaEnvApples::reset);

}
