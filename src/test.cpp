#include "mujoco_ros.hpp"
#include "franka_table.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<mujoco_ros::FrankaMJROS> node = std::make_shared<mujoco_ros::FrankaMJROS>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}