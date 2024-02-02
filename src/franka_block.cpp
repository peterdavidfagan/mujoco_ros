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
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MJSimulation : public rclcpp::Node
{
  public:
    MJSimulation()
    : Node("mujoco_simulation"), count_(0)
    {

      // subscriptions
      joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		      "/mujoco_joint_commands", 
		      10, 
		      std::bind(&MJSimulation::joint_command_callback, this, std::placeholders::_1)
		      );

      // publishers
      joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_states", rclcpp::SensorDataQoS());
      camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/overhead_camera", 10);
      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS());
      
      // simulation update timer
      timer_ = this->create_wall_timer(
      10ms, std::bind(&MJSimulation::update_sim, this));
      
      // init GLFW
      if (!glfwInit()) {
    	mju_error("Could not initialize GLFW");
  	}
      glfwWindowHint(GLFW_VISIBLE, 0);
      glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
      GLFWwindow* window = glfwCreateWindow(640, 480, "Invisible window", NULL, NULL);
      if (!window) {
        mju_error("Could not create GLFW window");
      }
      glfwMakeContextCurrent(window);
     
      // read model from file and check for errors
      char error[1000] = "Could not load binary model";
      m = mj_loadXML("/home/peter/Code/research_projects/mujoco_debug/src/mujoco_ros_sim/models/scene.xml", 0, error, 1000);
      if (!m) {
      	mju_error("Could not load model");
      }

      // setup mujoco
      d = mj_makeData(m);
      mj_forward(m, d);
      
      mjv_defaultCamera(&cam);
      mjv_defaultOption(&opt);
      mjv_defaultScene(&scn);
      mjr_defaultContext(&con);
      
      mjv_makeScene(m, &scn, 2000);
      mjr_makeContext(m, &con, mjFONTSCALE_150);
      

      // set rendering to offscreen buffer
      mjr_setBuffer(mjFB_OFFSCREEN, &con);
      if (con.currentBuffer!=mjFB_OFFSCREEN) {
        std::printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");
      }

      // get size of active renderbuffer
      mjrRect viewport =  mjr_maxViewport(&con);
      int W = viewport.width;
      int H = viewport.height;
      RCLCPP_INFO(this->get_logger(), "Width: %d, Height: %d", W, H);

      // allocate rgb and depth buffers
      rgb = (unsigned char*)std::malloc(3*W*H);

      last_camera_publish_time = 0.0;
    }

  private:
    void update_sim()
    {
      RCLCPP_INFO(this->get_logger(), "Updating simulation");
      
      // update data d with new joint commands

      // step simulation
      mj_step(m, d);

      // publish clock and joint states
      auto message = rosgraph_msgs::msg::Clock();
      int64_t time = d->time * 1e9;
      message.clock.sec = rclcpp::Time(time).seconds();
      message.clock.nanosec = rclcpp::Time(time).nanoseconds();
      clock_publisher_->publish(message);

      //auto joint_state = sensor_msgs::msg::JointState();
      //joint_state.header.stamp = rclcpp::Time(d->time);
      //joint_state.name.push_back("joint1");
      //joint_state.position.push_back(d->qpos[0]);
      //joint_state_publisher_->publish(joint_state);
	
      // periodically publish camera images
      if (d->time - last_camera_publish_time > 1.0/30.0) {
	RCLCPP_INFO(this->get_logger(), "Publishing camera image");

	// render offscreen
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn); 
	mjr_render(viewport, &scn, &con);
	mjr_readPixels(rgb, NULL, viewport, &con);
	
	// try plot with opencv
	cv::Mat img = cv::Mat(480, 640, CV_8UC3, rgb);
	cv::imshow("image", img);
	cv::waitKey(1);

	// create image message
	auto image = sensor_msgs::msg::Image();
	image.header.stamp = rclcpp::Time(time);
	image.height = 480;
	image.width = 640;
	image.encoding = "rgb8";
	image.is_bigendian = 0;
	image.step = image.width * 3;
	size_t data_size = image.width * image.height * 3;
	image.data.resize(data_size);
	std::memcpy(&image.data[0], rgb, data_size);
	
	// publish image
	camera_publisher_->publish(image);
	
	// update last publish time
	last_camera_publish_time = d->time;
      }

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
    double last_camera_publish_time;

    mjModel* m;                  // MuJoCo model
    mjData* d;                   // MuJoCo data
    mjvCamera cam;               // abstract camera
    mjvOption opt;               // visualization options
    mjvScene scn;                // abstract scene
    mjrContext con;              // custom GPU context

    // GLFW variables
    GLFWwindow* window;
    unsigned char* rgb;
    mjrRect viewport;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MJSimulation>());
  rclcpp::shutdown();
  return 0;
}
