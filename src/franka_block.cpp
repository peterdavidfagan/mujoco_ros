#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <mujoco/mujoco.h>

#include <GLFW/glfw3.h>

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
      joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_states", rclcpp::SensorDataQoS());

      // publish camera images
      camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/overhead_camera", rclcpp::SensorDataQoS());

      // publish simulation time
      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS());
      
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MJSimulation::update_sim, this));

      // MuJoCo data structures
      mjModel* m = NULL;                  // MuJoCo model
      mjData* d = NULL;                   // MuJoCo data
      mjvCamera cam;                      // abstract camera
      mjvOption opt;                      // visualization options
      mjvScene scn;                       // abstract scene
      mjrContext con;                     // custom GPU context
      
      // read model from file and check for errors
      char error[1000] = "Could not load binary model";
      m = mj_loadXML("/home/peter/Code/research_projects/mujoco_debug/src/mujoco_ros_sim/models/scene.xml", 0, error, 1000);
      if (!m) {
      	mju_error("Could not load model");
      }

      // make data
      d = mj_makeData(m);

      // init GLFW
      if (!glfwInit()) {
    	mju_error("Could not initialize GLFW");
  	}

      // create window, make OpenGL context current, request v-sync
      GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
      glfwMakeContextCurrent(window);
      glfwSwapInterval(1);

      // initialize visualization data structures
      mjv_defaultCamera(&cam);
      mjv_defaultOption(&opt);
      mjv_defaultScene(&scn);
      mjr_defaultContext(&con);

      // create scene and context
      mjv_makeScene(m, &scn, 2000);
      mjr_makeContext(m, &con, mjFONTSCALE_150);

      // just for testing
      // run main loop, target real-time simulation and 60 fps rendering
      while (!glfwWindowShouldClose(window)) {
         mjtNum simstart = d->time;
         while (d->time - simstart < 1.0/60.0) {
            mj_step(m, d);
         }

         // get framebuffer viewport
         mjrRect viewport = {0, 0, 0, 0};
         glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

         // update scene and render
         mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
         mjr_render(viewport, &scn, &con);

         // swap OpenGL buffers (blocking call due to v-sync)
         glfwSwapBuffers(window);

         // process pending GUI events, call GLFW callbacks
         glfwPollEvents();
      }
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
