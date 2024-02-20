#include <string>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace mujoco_ros
{

class MJROS : public rclcpp::Node
{
public:
  void read_model_file();
  void render_single_camera(mjvCamera*, unsigned char*, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr);

  virtual void init_scene() = 0;
  virtual void step() = 0;
  virtual void start_render_thread() = 0;
  virtual void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) = 0;

  std::mutex physics_data_mutex;
  std::thread render_thread;
  std::string model_filepath;

  mjModel* m;
  mjData* d;
  mjModel* m_render;
  mjData* d_render;
  mjvOption opt;
  mjvScene scn;
  mjrContext con;
  mjrRect viewport;

  GLFWwindow* window;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;
  size_t count_;

  ~MJROS() = default;

protected:
  MJROS() : Node("mujoco_ros"), count_(0){};
};

}  // namespace mujoco_ros
