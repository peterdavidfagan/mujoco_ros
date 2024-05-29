#include <string>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <pybind11/pybind11.h>


namespace py = pybind11;


namespace franka_env
{

class FrankaEnvBase : public mujoco_ros::MJROS {
public:
  FrankaEnvBase(py::object model, py::object data, const std::string command_interface, int control_steps, double control_timer_freq);
  ~FrankaEnvBase() {};

  void setSync(const bool &status);
  bool getSync();

  void reset();
  void step();
  void effort_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void position_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void robotiq_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:

  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robotiq_joint_command_subscription_;
  rclcpp::CallbackGroup::SharedPtr physics_step_callback_group_;

  // initial joint positions
  mjtNum* init_qpos = new mjtNum[7]{ 0, -0.785, 0, -2.356, 0, 1.571, 0.785};
  mjtNum* current_ctrl = new mjtNum[8]{ 0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0 };
  std::vector<std::string> joint_names;

  // control variables
  int control_steps;

  // booleans to track simulation state
  bool is_syncing = false;
  bool is_running = false;

  // friend classes
  friend class FrankaEnvBlocks; 
  friend class FrankaEnvApples;
};

class FrankaEnvBlocks : public FrankaEnvBase{
public:
  FrankaEnvBlocks(py::object model, py::object data, const std::string command_interface, int control_steps, double control_timer_freq);
  ~FrankaEnvBlocks() {};

  void reset();
  void start_render_thread();

private:

  // ROS 2
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher;
  rclcpp::CallbackGroup::SharedPtr render_callback_group_;

  // MuJoCo variables
  mjvCamera camera;

  // GLFW variables
  unsigned char* camera_rgb;
};


class FrankaEnvApples : public FrankaEnvBase{
public:
  FrankaEnvApples(py::object model, py::object data, const std::string command_interface, int control_steps, double control_timer_freq);
  ~FrankaEnvApples() {};
  
  void reset();
  void start_render_thread();

private:

  // ROS 2
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher;
  rclcpp::CallbackGroup::SharedPtr render_callback_group_;

  // MuJoCo variables
  mjvCamera camera;

  // GLFW variables
  unsigned char* camera_rgb;
};

} 
