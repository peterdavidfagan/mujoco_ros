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


namespace franka_table
{

class FrankaTableEnv : public mujoco_ros::MJROS {
public:
  FrankaTableEnv(py::object model, py::object data);
  ~FrankaTableEnv() {};

  void setSync(const bool &status);
  bool getSync();

private:
  void init_scene();
  void step();
  void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void robotiq_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void start_render_thread();

  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robotiq_joint_command_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overhead_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_camera_publisher_;
  rclcpp::CallbackGroup::SharedPtr physics_step_callback_group_;
  rclcpp::CallbackGroup::SharedPtr render_callback_group_;

  // initial joint positions
  mjModel* m;
  mjData* d;
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

  bool is_syncing = false;
};

}  // namespace mujoco_ros
