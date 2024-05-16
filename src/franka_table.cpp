#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

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
#include <opencv2/opencv.hpp>

#include "mujoco_ros.hpp"
#include "franka_table.hpp"
#include <pybind11/pybind11.h>


namespace py = pybind11;
using namespace std::chrono_literals;


namespace mujoco_ros 
{

FrankaMJROS::FrankaMJROS(py::object model, py::object data) : MJROS()
{
  // cast to mjModel pointer
  std::uintptr_t m_raw = model.attr("_address").cast<std::uintptr_t>();
	std::uintptr_t d_raw = data.attr("_address").cast<std::uintptr_t>();
  m = reinterpret_cast<mjModel *>(m_raw);
  d = reinterpret_cast<mjData *>(d_raw);

  // set up initial mujoco simulation scene
  this->init_scene();

  // set up ROS 2
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

  // callback groups
  physics_step_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  render_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // subscriptions
  joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/panda_arm_joint_commands", qos_reliable,
      std::bind(&FrankaMJROS::joint_command_callback, this, std::placeholders::_1));
  robotiq_joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robotiq_joint_commands", qos_reliable,
      std::bind(&FrankaMJROS::robotiq_joint_command_callback, this, std::placeholders::_1));

  // publishers
  rclcpp::PublisherOptions physics_publisher_options;
  physics_publisher_options.callback_group = physics_step_callback_group_;
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_states", qos_reliable,
                                                                                physics_publisher_options);
  overhead_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/overhead_camera", 10);

  /*
    * setup physics time stepping, currently the sim steps physics at 1e-3
    * control is applied at 5e-3, with these settings the simulation is roughly
    * realtime for interfacing with ros 2 control. The simulation environment is
    * mostly stable (e.g. can stack blocks with these settings) but it may be necessary
    * to tune further for contact rich environments.
    */
  timer_ = this->create_wall_timer(1ms, std::bind(&FrankaMJROS::step, this), physics_step_callback_group_);

  // start rendering thread
  m_render = mj_copyModel(m_render, m);
  d_render = mj_copyData(d_render, m, d);
  this->start_render_thread();
}

void FrankaMJROS::setSync(const bool &status) { is_syncing = status; }
bool FrankaMJROS::getSync() {return this->is_syncing;}

void FrankaMJROS::read_model_file()
{
  std::string model_file_path = "/home/peter/Code/temp/ros2_robotics_research_toolkit/src/mujoco/mujoco_ros/models/rearrangement_env.mjb"; // hardcode while debugging
  const char* filename = model_file_path.c_str();
  char error[1000] = "Could not load binary model";
  if (std::strlen(filename) > 4 && !std::strcmp(filename + std::strlen(filename) - 4, ".mjb"))
  {
    m = mj_loadModel(filename, 0);
  }
  else
  {
    std::cout << "Loading XML model" << std::endl;
    std::cout << filename << std::endl;
    m = mj_loadXML(filename, 0, error, 1000);
  }
  if (!m)
  {
    mju_error("Load model error: %s", error);
  }
};

void FrankaMJROS::init_scene()
{
  // set initial robot configuration and control signal
  mju_copy(d->qpos, init_qpos, 7);
  mju_copy(d->ctrl, current_ctrl, 8);

  // assign cameras
  overhead_cam.type = mjCAMERA_FIXED;
  overhead_cam.fixedcamid = 1;

  /*
    * There is a mismatch between mujoco menagerie joint names and those
    * used in the robotiq ROS package. Here we map the mujoco joint names
    * at initialization to those used by ROS and use this when publishing
    * the robot state. This mapping should be moved to config file.
    * Also note the panda joint mapping aligns with my fork of mujoco_menagerie
    * I plan to update this mapping to be consistent with the upstream version.
    */
  std::map<std::string, std::string> joint_name_map;
  joint_name_map["panda_joint1"] = "panda_joint1";
  joint_name_map["panda_joint2"] = "panda_joint2";
  joint_name_map["panda_joint3"] = "panda_joint3";
  joint_name_map["panda_joint4"] = "panda_joint4";
  joint_name_map["panda_joint5"] = "panda_joint5";
  joint_name_map["panda_joint6"] = "panda_joint6";
  joint_name_map["panda_joint7"] = "panda_joint7";
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

  mj_forward(m, d);
}

void FrankaMJROS::step()
{ 
  if (this->is_syncing) // check if syncing interactive viewer 
  {
    return;
  }
  else // apply control and step simulation
  {
    std::lock_guard lock(physics_data_mutex);
    d->ctrl = current_ctrl;
    mj_step(m, d);
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

void FrankaMJROS::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // ignore if command is all zeros (TODO: revise ROS 2 JTC hold command)
  if (std::all_of(msg->position.begin(), msg->position.end(), [](double v) { return v == 0; }))
  {
    return;
  }
  else
  {
    std::lock_guard<std::mutex> lock(physics_data_mutex);
    for (int i = 0; i < 8; i++)
    {
      current_ctrl[i] = msg->position[i];
    }
  }
}

void FrankaMJROS::robotiq_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // only perform lock if gripper command value changed
  if (msg->position[0] == current_ctrl[8])
  {
    return;
  }
  else
  {
    std::lock_guard<std::mutex> lock(physics_data_mutex);
    // custom mapping required see below link for details of MuJoCo implementation
    // https://github.com/google-deepmind/mujoco_menagerie/blob/8ef01e87fffaa8ec634a4826c5b2092733b2f3c8/robotiq_2f85/2f85.xml#L180
    current_ctrl[7] = msg->position[0] / (0.8 / 255);
  }
}

void FrankaMJROS::start_render_thread()
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
    overhead_rgb = (unsigned char*)std::malloc(3 * W * H);

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
      this->render_single_camera(&overhead_cam, overhead_rgb, overhead_camera_publisher_);
    }
  });
}

}