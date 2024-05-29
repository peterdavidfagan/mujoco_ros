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
  void render_camera(mjvCamera*, unsigned char*, float*, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr);
  
  virtual void reset() = 0;
  virtual void step() = 0;

  std::mutex physics_data_mutex;
  std::thread render_thread;

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

  virtual ~MJROS() = default;

protected:
  MJROS() : Node("mujoco_ros"), count_(0){};
};

}  // namespace mujoco_ros
