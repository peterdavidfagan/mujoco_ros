#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rmw/types.h>

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
      // read mujoco model from file and check for errors
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

      // setup mujoco simulation and rendering
      d = mj_makeData(m);
      mj_forward(m, d);
      mju_copy(d->qpos, hold_qpos, 7);

      overhead_cam.type = mjCAMERA_FIXED;
      overhead_cam.fixedcamid = 1;
      front_cam.type = mjCAMERA_FIXED;
      front_cam.fixedcamid = 2;
      
      // configure ROS communication
      static const rmw_qos_profile_t rmw_qos_profile_reliable =
      {
  	RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  	10,
  	RMW_QOS_POLICY_RELIABILITY_RELIABLE,
	RMW_QOS_POLICY_DURABILITY_VOLATILE,
	RMW_QOS_DEADLINE_DEFAULT,
	RMW_QOS_LIFESPAN_DEFAULT,
	RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
	RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
	false
      };
      
      // quality of service and callback groups
      auto qos_reliable = rclcpp::QoS(
		      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_reliable),
		      rmw_qos_profile_reliable
		      );

      physics_step_callback_group_ = this->create_callback_group(
		      rclcpp::CallbackGroupType::MutuallyExclusive
		      );

      render_callback_group_ = this->create_callback_group(
		      rclcpp::CallbackGroupType::MutuallyExclusive
		      );

      
      // subscriptions
      joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      		      "/mujoco_joint_commands", 
      		      qos_reliable,
      		      std::bind(&MJSimulation::joint_command_callback, this, std::placeholders::_1)
      		      );

      // publishers
      rclcpp::PublisherOptions physics_publisher_options;
      physics_publisher_options.callback_group = physics_step_callback_group_;
      joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
		      "/mujoco_joint_states", 
		      qos_reliable,
		      physics_publisher_options
		      );

      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
		      "/clock", 
		      qos_reliable,
		      physics_publisher_options
		      );
      
      // create a publisher for the overhead camera
      rclcpp::PublisherOptions camera_publisher_options;
      camera_publisher_options.callback_group = render_callback_group_;
      overhead_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
		      "/overhead_camera",
		      qos_reliable,
		      camera_publisher_options
		      );
      front_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
		      "/front_camera",
		      qos_reliable,
		      camera_publisher_options
		      );

      // simulation stepping timer
      timer_ = this->create_wall_timer(
	10ms, 
	std::bind(&MJSimulation::update_sim, this),
	physics_step_callback_group_
	);
      
      // start a separate thread for rendering that runs at 10Hz
      m_render = mj_copyModel(m_render, m);
      d_render = mj_copyData(d_render, m, d);
      this->start_render_thread();
      
    } 

  private:
    void update_sim()
    {
      std::unique_lock<std::mutex> lock(render_mutex);  
      if (hold) {
	d->ctrl = hold_qpos;
      } 
      for (int i=0; i<10; i++) {
         mj_step(m, d);
      }
      lock.unlock();

      // publish clock and joint states
      auto message = rosgraph_msgs::msg::Clock();
      int64 current_time = d->time * 1e9;
      message.clock.sec = rclcpp::Time(current_time).seconds();
      message.clock.nanosec = rclcpp::Time(current_time).nanoseconds();
      clock_publisher_->publish(message);

      // publish joint state of franka emika panda
      auto joint_state = sensor_msgs::msg::JointState();
      joint_state.header.stamp.sec = rclcpp::Time(current_time).seconds();
      joint_state.header.stamp.nanosec = rclcpp::Time(current_time).nanoseconds();
      for (int i=0; i<8; i++) {
	joint_state.name.push_back("panda_joint" + std::to_string(i+1));
	joint_state.position.push_back(d->qpos[i]);
	joint_state.velocity.push_back(d->qvel[i]);
      }
      joint_state_publisher_->publish(joint_state);
    }

    void start_render_thread()
    {
	  render_thread_ = std::thread([this]() {
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
                mjv_defaultOption(&opt);
                mjv_defaultScene(&scn);
                mjv_makeScene(m_render, &scn, 2000);
	        //mjrContext con;
	        mjr_defaultContext(&con);
                mjr_makeContext(m_render, &con, mjFONTSCALE_150);      
                mjr_setBuffer(mjFB_OFFSCREEN, &con);
	        //mjrRect viewport =  mjr_maxViewport(&con);
                viewport =  mjr_maxViewport(&con);
                int W = viewport.width;
                int H = viewport.height;
                front_rgb = (unsigned char*)std::malloc(3*W*H);
		while (running_) {
			    this->render();
			    std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	    });
    }

    void render()
    {
      	// copy data from sim to render buffer
	std::unique_lock<std::mutex> lock(render_mutex);
	d_render = mj_copyData(d_render, m, d);
	lock.unlock();
	int64 current_time = d_render->time * 1e9;

	// render offscreen
	mjv_updateScene(m_render, d_render, &opt, NULL, &front_cam, mjCAT_ALL, &scn);
	mjr_render(viewport, &scn, &con);
	mjr_readPixels(front_rgb, NULL, viewport, &con);

        // create front camera image message
        auto front_image = sensor_msgs::msg::Image();
        front_image.header.stamp.sec = rclcpp::Time(current_time).seconds();
        front_image.header.stamp.nanosec = rclcpp::Time(current_time).nanoseconds();
        front_image.height = 480;
        front_image.width = 640;
        front_image.encoding = "rgb8";
        front_image.is_bigendian = 0;
        front_image.step = front_image.width * 3;
        size_t front_data_size = front_image.width * front_image.height * 3;
        front_image.data.resize(front_data_size);
        std::memcpy(&front_image.data[0], front_rgb, front_data_size);

        // publish image
        front_camera_publisher_->publish(front_image);
    }


    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::unique_lock<std::mutex> lock(render_mutex);
      for (int i=0; i<8; i++) {
	d->ctrl[i] = msg->position[i];
      }
      hold = false;
      lock.unlock();
    }
    
    // ROS 2 
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;
    
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overhead_camera_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_camera_publisher_;
    
    rclcpp::CallbackGroup::SharedPtr physics_step_callback_group_;
    rclcpp::CallbackGroup::SharedPtr render_callback_group_;
    
    size_t count_;
    std::mutex render_mutex;
    std::thread render_thread_;
    bool running_ = true;
    bool hold = false;	

    // MuJoCo variables
    mjModel* m;                  // MuJoCo model
    mjData* d;                   // MuJoCo data
    mjModel* m_render;           // MuJoCo model for rendering
    mjData* d_render;            // MuJoCo data for rendering
    mjvCamera overhead_cam;      // abstract camera
    mjvCamera front_cam;         // abstract camera
    mjvOption opt;               // visualization options
    mjvScene scn;                // abstract scene
    mjrContext con;              // custom GPU context
    mjtNum* hold_qpos = new mjtNum[8]{0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0};

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
  //rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
