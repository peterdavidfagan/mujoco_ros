#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MJSimulation : public rclcpp::Node
{
  public:
    MJSimulation()
    : Node("mujoco_simulation"), count_(0)
    {
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MJSimulation::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Hello");
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MJSimulation>());
  rclcpp::shutdown();
  return 0;
}
