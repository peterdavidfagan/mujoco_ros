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

#include <mujoco_ros.hpp>

using namespace std::chrono_literals;

class FrankaMJROS : public mujoco_ros::MJROS
{
public:
  FrankaMJROS() : mujoco_ros::MJROS()
  {
    // read in model file
    this->read_model_file();

    // setup mujoco simulation and rendering
    this->init_scene();

    // setup ROS 2
    // quality of service profile
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

    //  callback groups
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
    front_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/front_camera", 10);
    left_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/left_camera", 10);
    right_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera", 10);

    /*
     * setup physics time stepping, currently the sim steps physics at 1e-3
     * control is applied at 5e-3, with these settings the simulation is roughly
     * realtime for interfacing with ros 2 control. The simulation environment is
     * mostly stable (e.g. can stack blocks with these settings) but it may be necessary
     * to tune further for contact rich environments.
     */
    timer_ = this->create_wall_timer(5ms, std::bind(&FrankaMJROS::step, this), physics_step_callback_group_);

    // setup rendering thread
    m_render = mj_copyModel(m_render, m);
    d_render = mj_copyData(d_render, m, d);
    this->start_render_thread();
  }

private:
  void init_scene()
  {
    d = mj_makeData(m);

    // set initial robot configuration and control signal
    mju_copy(d->qpos, init_qpos, 7);
    mju_copy(d->ctrl, current_ctrl, 8);

    // assign cameras
    overhead_cam.type = mjCAMERA_FIXED;
    overhead_cam.fixedcamid = 1;
    front_cam.type = mjCAMERA_FIXED;
    front_cam.fixedcamid = 2;
    left_cam.type = mjCAMERA_FIXED;
    left_cam.fixedcamid = 3;
    right_cam.type = mjCAMERA_FIXED;
    right_cam.fixedcamid = 4;

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

  void step()
  {
    // apply control and step simulation
    std::unique_lock<std::mutex> lock(physics_data_mutex);
    for (int i = 0; i < 5; i++)  // 5 as timer is set to 5ms and physics step is 1ms
    {
      d->ctrl = current_ctrl;
      mj_step(m, d);
    }
    lock.unlock();

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

  void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // ignore if command is all zeros (TODO: revise ROS 2 JTC hold command)
    if (std::all_of(msg->position.begin(), msg->position.end(), [](double v) { return v == 0; }))
    {
      return;
    }

    std::unique_lock<std::mutex> lock(physics_data_mutex);
    for (int i = 0; i < 8; i++)
    {
      current_ctrl[i] = msg->position[i];
    }
    lock.unlock();
  }

  void robotiq_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // only perform lock if gripper command value changed
    if (msg->position[0] == current_ctrl[8])
    {
      return;
    }
    std::unique_lock<std::mutex> lock(physics_data_mutex);
    // custom mapping required see below link for details of MuJoCo implementation
    // https://github.com/google-deepmind/mujoco_menagerie/blob/8ef01e87fffaa8ec634a4826c5b2092733b2f3c8/robotiq_2f85/2f85.xml#L180
    current_ctrl[7] = msg->position[0] / (0.8 / 255);
    lock.unlock();
  }

  void start_render_thread()
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
      front_rgb = (unsigned char*)std::malloc(3 * W * H);
      left_rgb = (unsigned char*)std::malloc(3 * W * H);
      right_rgb = (unsigned char*)std::malloc(3 * W * H);

      while (true)
      {
        // copy data from sim to render buffer
        std::unique_lock<std::mutex> lock(physics_data_mutex);
        d_render = mj_copyData(d_render, m, d);
        lock.unlock();

        // publish messages
        this->render_single_camera(&front_cam, front_rgb, front_camera_publisher_);
        this->render_single_camera(&left_cam, left_rgb, left_camera_publisher_);
        this->render_single_camera(&right_cam, right_rgb, right_camera_publisher_);
        this->render_single_camera(&overhead_cam, overhead_rgb, overhead_camera_publisher_);
      }
    });
  }

  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robotiq_joint_command_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overhead_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_camera_publisher_;
  rclcpp::CallbackGroup::SharedPtr physics_step_callback_group_;
  rclcpp::CallbackGroup::SharedPtr render_callback_group_;

  // initial joint positions
  mjtNum* init_qpos = new mjtNum[8]{ 0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0 };
  mjtNum* current_ctrl = new mjtNum[8]{ 0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0 };

  // MuJoCo variables
  mjvCamera overhead_cam;
  mjvCamera front_cam;
  mjvCamera left_cam;
  mjvCamera right_cam;
  std::vector<std::string> joint_names;

  // GLFW variables
  unsigned char* overhead_rgb;
  unsigned char* front_rgb;
  unsigned char* left_rgb;
  unsigned char* right_rgb;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<FrankaMJROS> node = std::make_shared<FrankaMJROS>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
