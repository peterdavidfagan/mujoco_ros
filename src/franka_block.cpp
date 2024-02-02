#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include <mujoco/mujoco.h>

using namespace std::chrono_literals;

class MJSimulation : public rclcpp::Node
{
  public:
    MJSimulation()
    : Node("mujoco_simulation"), count_(0)
    {

      // subscribe to joint commands from ros control topic
      joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		      "/mujo_joint_commands", 
		      10, 
		      std::bind(&MJSimulation::joint_command_callback, this, std::placeholders::_1)
		      );

      // publish joint states from mujoco
      joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_states", 10);

      // publish camera images
      camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

      // publish simulation time
      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
      
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MJSimulation::update_sim, this));

      // MuJoCo data structures
      mjModel* m = NULL;                  // MuJoCo model
      mjData* d = NULL;                   // MuJoCo data
      mjvCamera cam;                      // abstract camera
      mjvOption opt;                      // visualization options
      mjvScene scn;                       // abstract scene
      mjrContext con;                     // custom GPU context
      
    }

  private:
    void update_sim()
    {
      RCLCPP_INFO(this->get_logger(), "Updating simulation");
    }

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->name[0].c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MJSimulation>());
  rclcpp::shutdown();
  return 0;
}
