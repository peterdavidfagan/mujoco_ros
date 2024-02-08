#include <chrono>
#include <functional>
#include <memory>
#include <string>
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

using namespace std::chrono_literals;

class MJSimulation : public rclcpp::Node
{
public:
  MJSimulation() : Node("mujoco_simulation"), count_(0)
  {
    
    // read mujoco model from file and check for errors
    this->declare_parameter("model_file", "./models/franka.mjb");
    std::string model_file_path = this->get_parameter("model_file").as_string();
    const char* filename = model_file_path.c_str();
    char error[1000] = "Could not load binary model";
    if (std::strlen(filename) > 4 && !std::strcmp(filename + std::strlen(filename) - 4, ".mjb"))
    {
      m = mj_loadModel(filename, 0);
    }
    else
    {
      m = mj_loadXML(filename, 0, error, 1000);
    }
    if (!m)
    {
      mju_error("Could not load model");
    }

    // setup mujoco simulation and rendering
    d = mj_makeData(m);
    mj_forward(m, d);
    mju_copy(d->qpos, hold_qpos, 7);

    
    // outline fixed cameras you want to use
    overhead_cam.type = mjCAMERA_FIXED;
    overhead_cam.fixedcamid = 1;
    front_cam.type = mjCAMERA_FIXED;
    front_cam.fixedcamid = 2;
    left_cam.type = mjCAMERA_FIXED;
    left_cam.fixedcamid = 3;
    right_cam.type = mjCAMERA_FIXED;
    right_cam.fixedcamid = 4;

    
    // setup ROS 2 communication
    
    // quality of service and callback groups
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
    render_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // subscriptions
    joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/mujoco_joint_commands", qos_reliable,
        std::bind(&MJSimulation::joint_command_callback, this, std::placeholders::_1));

    // publishers
    rclcpp::PublisherOptions physics_publisher_options;
    physics_publisher_options.callback_group = physics_step_callback_group_;
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_states", qos_reliable,
                                                                                  physics_publisher_options);
    rclcpp::PublisherOptions camera_publisher_options;
    camera_publisher_options.callback_group = render_callback_group_;
    overhead_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/overhead_camera", 10);
    front_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/front_camera", 10);
    left_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/left_camera", 10);
    right_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera", 10);

    // setup physics timestepping (control is applied at 100Hz for now)
    timer_ = this->create_wall_timer(10ms, std::bind(&MJSimulation::step, this), physics_step_callback_group_);

    // setup rendering thread
    m_render = mj_copyModel(m_render, m);
    d_render = mj_copyData(d_render, m, d);
    this->start_render_thread();
  }

private:
  void step()
  {
    // apply control and step simulation
    std::unique_lock<std::mutex> lock(physics_data_mutex);
    if (hold)
    {
      d->ctrl = hold_qpos;
    }
    for (int i = 0; i < 10; i++)
    {
      mj_step(m, d);
    }
    lock.unlock();

    // publish joint state of franka emika panda
    rclcpp::Time current_time = this->now();
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = current_time;
    for (int i = 0; i < 8; i++)
    {
      joint_state.name.push_back("panda_joint" + std::to_string(i + 1));
      joint_state.position.push_back(d->qpos[i]);
      joint_state.velocity.push_back(d->qvel[i]);
    }
    joint_state_publisher_->publish(joint_state);
  }

  void start_render_thread()
  {
    render_thread_ = std::thread([this]() {
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

      while (running_)
      {
        this->render();
      }
    });
  }
  
  void publish_single_camera(mjvCamera* cam, unsigned char* rgb, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
  {

    // update scene and render
    mjv_updateScene(m_render, d_render, &opt, NULL, cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    mjr_readPixels(rgb, NULL, viewport, &con);
    
    // OpenGL uses bottom-left origin, so we need to flip the image
    cv::Mat image_mat(viewport.height, viewport.width, CV_8UC3, rgb);
    cv::Mat flipped_image_mat;
    cv::flip(image_mat, flipped_image_mat, 0);

    // create image message
    auto image_msg = sensor_msgs::msg::Image();
    image_msg.header.stamp = this->now();
    image_msg.height = viewport.height;
    image_msg.width = viewport.width;
    image_msg.encoding = "rgb8";
    image_msg.is_bigendian = false;
    image_msg.step = 3 * viewport.width;
    size_t data_size = 3 * viewport.width * viewport.height;
    image_msg.data.resize(data_size);
    std::memcpy(image_msg.data.data(), flipped_image_mat.data, data_size);

    // publish image
    publisher->publish(image_msg);
  }

  void render()
  {
    // copy data from sim to render buffer
    std::unique_lock<std::mutex> lock(physics_data_mutex);
    d_render = mj_copyData(d_render, m, d);
    lock.unlock();

    // publish messages
    this->publish_single_camera(&front_cam, front_rgb, front_camera_publisher_);
    this->publish_single_camera(&left_cam, left_rgb, left_camera_publisher_);
    this->publish_single_camera(&right_cam, right_rgb, right_camera_publisher_);
    this->publish_single_camera(&overhead_cam, overhead_rgb, overhead_camera_publisher_);
  }

  void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(physics_data_mutex);
    for (int i = 0; i < 8; i++)
    {
      d->ctrl[i] = msg->position[i];
    }
    hold = false;
    lock.unlock();
  }

  // ROS 2
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overhead_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_camera_publisher_;
  rclcpp::CallbackGroup::SharedPtr physics_step_callback_group_;
  rclcpp::CallbackGroup::SharedPtr render_callback_group_;
  size_t count_;
  
  // simulation status
  bool running_ = true;
  bool hold = true;
  mjtNum* hold_qpos = new mjtNum[8]{ 0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0 };
  
  // memory management
  std::mutex physics_data_mutex;
  std::thread render_thread_;

  // MuJoCo variables
  mjModel* m;              
  mjData* d;               
  mjModel* m_render;       
  mjData* d_render;        
  mjvCamera overhead_cam;  
  mjvCamera front_cam;     
  mjvCamera left_cam;      
  mjvCamera right_cam;    
  mjvOption opt;           
  mjvScene scn;            
  mjrContext con;          
  mjrRect viewport;

  // GLFW variables
  GLFWwindow* window;
  unsigned char* overhead_rgb;
  unsigned char* front_rgb;
  unsigned char* left_rgb;
  unsigned char* right_rgb;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MJSimulation> node = std::make_shared<MJSimulation>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
