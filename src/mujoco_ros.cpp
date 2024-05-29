#include "mujoco_ros.hpp"
#include <opencv2/opencv.hpp>

namespace mujoco_ros
{

void MJROS::render_camera(mjvCamera* cam, 
                          unsigned char* rgb, 
                          float* depth, 
                          rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher,
                          rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher
                          )
{
  // update scene and render
  mjv_updateScene(m_render, d_render, &opt, NULL, cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);
  mjr_readPixels(rgb, depth, viewport, &con);

  // OpenGL uses bottom-left origin, so we need to flip the image
  cv::Mat image_mat(viewport.height, viewport.width, CV_8UC3, rgb);
  cv::Mat flipped_image_mat;
  cv::flip(image_mat, flipped_image_mat, 0);

  // create image message
  auto image_msg = sensor_msgs::msg::Image();
  image_msg.header.stamp = this->now();
  image_msg.height = viewport.height;
  image_msg.width = viewport.width;
  image_msg.encoding = "rgb8";
  image_msg.is_bigendian = false;
  image_msg.step = 3 * viewport.width;
  size_t data_size = 3 * viewport.width * viewport.height;
  image_msg.data.resize(data_size);
  std::memcpy(image_msg.data.data(), flipped_image_mat.data, data_size);

  // publish image
  rgb_publisher->publish(image_msg);


  if (depth != nullptr) {
    // OpenGL uses bottom-left origin, so we need to flip the depth image
    cv::Mat depth_mat(viewport.height, viewport.width, CV_32FC1, depth);
    cv::Mat flipped_depth_mat;
    cv::flip(depth_mat, flipped_depth_mat, 0);

    // create depth image message
    auto depth_msg = sensor_msgs::msg::Image();
    depth_msg.header.stamp = this->now();
    depth_msg.height = viewport.height;
    depth_msg.width = viewport.width;
    depth_msg.encoding = "32FC1"; // Floating point 32-bit depth
    depth_msg.is_bigendian = false;
    depth_msg.step = sizeof(float) * viewport.width;
    size_t depth_data_size = sizeof(float) * viewport.width * viewport.height;
    depth_msg.data.resize(depth_data_size);
    std::memcpy(depth_msg.data.data(), flipped_depth_mat.data, depth_data_size);

    // publish depth image
    depth_publisher->publish(depth_msg);
  }
};

}  // namespace mujoco_ros
