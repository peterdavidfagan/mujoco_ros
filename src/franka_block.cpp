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
      const char* filename = "/home/peter/Code/research_projects/mujoco_debug/src/mujoco_ros_sim/models/colour_splitter.mjb";
      if (std::strlen(filename)>4 && !std::strcmp(filename+std::strlen(filename)-4, ".mjb")) {
          m = mj_loadModel(filename, 0);
      } else {
          m = mj_loadXML(filename, 0, error, 1000);
      }
      if (!m) {
      	mju_error("Could not load model");
      }

      // setup mujoco
      d = mj_makeData(m);
      mj_forward(m, d);
      mju_copy(d->qpos, hold_qpos, 7);

      overhead_cam.type = mjCAMERA_FIXED;
      overhead_cam.fixedcamid = 1;
      front_cam.type = mjCAMERA_FIXED;
      front_cam.fixedcamid = 2;

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
      viewport =  mjr_maxViewport(&con);
      int W = viewport.width;
      int H = viewport.height;

      // allocate rgb and depth buffers
      overhead_rgb = (unsigned char*)std::malloc(3*W*H);
      front_rgb = (unsigned char*)std::malloc(3*W*H);

      last_camera_publish_time = 0.0;
      
      // subscriptions
      joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		      "/mujoco_joint_commands", 
		      10, 
		      std::bind(&MJSimulation::joint_command_callback, this, std::placeholders::_1)
		      );

      // publishers
      joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_states", rclcpp::SensorDataQoS());
      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS());
      
      // create a publisher for the overhead camera
      overhead_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/overhead_camera", 10);
      front_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/front_camera", 10);

      // simulation update timer
      timer_ = this->create_wall_timer(
      10ms, std::bind(&MJSimulation::update_sim, this));
    }

  private:
    void update_sim()
    {
      
      // check that all control values are non-zero
      if (hold) {
	d->ctrl = hold_qpos;
      } 

      // step simulation
      for (int i=0; i<10; i++) {
      	mj_step(m, d);
      }

      // publish clock and joint states
      auto message = rosgraph_msgs::msg::Clock();
      int64_t time = d->time * 1e9;
      message.clock.sec = rclcpp::Time(time).seconds();
      message.clock.nanosec = rclcpp::Time(time).nanoseconds();
      clock_publisher_->publish(message);

      // publish joint state of franka emika panda
      auto joint_state = sensor_msgs::msg::JointState();
      joint_state.header.stamp.sec = rclcpp::Time(time).seconds();
      joint_state.header.stamp.nanosec = rclcpp::Time(time).nanoseconds();
      for (int i=0; i<8; i++) {
	joint_state.name.push_back("panda_joint" + std::to_string(i+1));
	joint_state.position.push_back(d->qpos[i]);
	joint_state.velocity.push_back(d->qvel[i]);
      }
      joint_state_publisher_->publish(joint_state);
	
      // periodically publish camera images
      if (d->time - last_camera_publish_time > 0.2) {

         // render offscreen
         mjv_updateScene(m, d, &opt, NULL, &overhead_cam, mjCAT_ALL, &scn); 
         mjr_render(viewport, &scn, &con);
         mjr_readPixels(overhead_rgb, NULL, viewport, &con);
	
	mjv_updateScene(m, d, &opt, NULL, &front_cam, mjCAT_ALL, &scn);
	mjr_render(viewport, &scn, &con);
	mjr_readPixels(front_rgb, NULL, viewport, &con);

	// convert opengl image covention to ros image (flip image)
	int H = viewport.height;
	int W = viewport.width;
	for (int i=0; i<H/2; i++) {
		for (int j=0; j<W*3; j++) {
			std::swap(overhead_rgb[i*W*3+j], overhead_rgb[(H-i-1)*W*3+j]);
			std::swap(front_rgb[i*W*3+j], front_rgb[(H-i-1)*W*3+j]);
		}
	}


        // create overhead camera image message
        auto overhead_image = sensor_msgs::msg::Image();
        overhead_image.header.stamp.sec = rclcpp::Time(time).seconds();
	overhead_image.header.stamp.nanosec = rclcpp::Time(time).nanoseconds();
        overhead_image.height = 480;
        overhead_image.width = 640;
        overhead_image.encoding = "rgb8";
        overhead_image.is_bigendian = 0;
        overhead_image.step = overhead_image.width * 3;
        size_t data_size = overhead_image.width * overhead_image.height * 3;
        overhead_image.data.resize(data_size);
        std::memcpy(&overhead_image.data[0], overhead_rgb, data_size);

	// create front camera image message
	auto front_image = sensor_msgs::msg::Image();
	front_image.header.stamp.sec = rclcpp::Time(time).seconds();
	front_image.header.stamp.nanosec = rclcpp::Time(time).nanoseconds();
	front_image.height = 480;
	front_image.width = 640;
	front_image.encoding = "rgb8";
	front_image.is_bigendian = 0;
	front_image.step = front_image.width * 3;
	size_t front_data_size = front_image.width * front_image.height * 3;
	front_image.data.resize(front_data_size);
	std::memcpy(&front_image.data[0], front_rgb, front_data_size);

        // publish image
        overhead_camera_publisher_->publish(overhead_image);
	front_camera_publisher_->publish(front_image);

       // update last publish time
       last_camera_publish_time = d->time;
      }

    }

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      // update joint commands (as we run a single thread, we don't need to lock the data)
      for (int i=0; i<8; i++) {
	d->ctrl[i] = msg->position[i];
      }
      hold = false;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overhead_camera_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_camera_publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    
    size_t count_;
    double last_camera_publish_time;
    bool hold = true;	
    mjtNum* hold_qpos = new mjtNum[8]{0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0};

    mjModel* m;                  // MuJoCo model
    mjData* d;                   // MuJoCo data
    mjvCamera overhead_cam;      // abstract camera
    mjvCamera front_cam;         // abstract camera
    mjvOption opt;               // visualization options
    mjvScene scn;                // abstract scene
    mjrContext con;              // custom GPU context

    // GLFW variables
    GLFWwindow* window;
    unsigned char* overhead_rgb;
    unsigned char* front_rgb;
    mjrRect viewport;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MJSimulation> node = std::make_shared<MJSimulation>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
