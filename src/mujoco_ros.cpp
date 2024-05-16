#include "mujoco_ros.hpp"
#include <opencv2/opencv.hpp>

namespace mujoco_ros
{

void MJROS::render_single_camera(mjvCamera* cam, unsigned char* rgb,
                                 rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
{
  // update scene and render
  mjv_updateScene(m_render, d_render, &opt, NULL, cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);
  mjr_readPixels(rgb, NULL, viewport, &con);

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
  publisher->publish(image_msg);
};

}  // namespace mujoco_ros
