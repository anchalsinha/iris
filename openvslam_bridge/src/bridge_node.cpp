// Copyright (c) 2020, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "bridge.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <iostream>
#include <pcl/correspondence.h>
#include <pcl_ros/point_cloud.h>
#include <rclcpp/rclcpp.h>
#include <tf/transform_broadcaster.h>

void publishPose(const Eigen::Matrix4f& T, const std::string& child_frame_id, const rclcpp::Time& stamp)
{
  static tf2_ros::TransformBroadcaster br;
  tf2_ros::Transform transform;
  transform.setFromOpenGLMatrix(T.cast<double>().eval().data());
  br.sendTransform(tf2_ros::StampedTransform(transform, stamp, "world", child_frame_id));
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("openvslam_bridge_node");
  node->declare_parameter("vocab_path", "");
  node->declare_parameter("vslam_config_path", "");
  node->declare_parameter("image_topic_name0", "");
  node->declare_parameter("is_image_compressed", false);
  node->declare_parameter("keyframe_recollection", 0);

  // Get rosparams
  bool is_image_compressed;
  int recollection = 30;
  std::string vocab_path, vslam_config_path, image_topic_name;
  node->get_parameter("vocab_path", vocab_path);
  node->get_parameter("vslam_config_path", vslam_config_path);
  node->get_parameter("image_topic_name0", image_topic_name);
  node->get_parameter("is_image_compressed", is_image_compressed);
  node->get_parameter("keyframe_recollection", recollection);
  RCLCPP_INFO(node->get_logger(), "vocab_path: %s, vslam_config_path: %s, image_topic_name: %s, is_image_compressed: %d",
      vocab_path.c_str(), vslam_config_path.c_str(), image_topic_name.c_str(), is_image_compressed);


  const int lower_threshold_of_points = 1500;
  const int upper_threshold_of_points = 2000;

  // Setup subscriber
  image_transport::ImageTransport it(node);
  rclcpp::Time subscribed_stamp;
  cv::Mat subscribed_image;
  image_transport::TransportHints hints("raw");
  if (is_image_compressed) hints = image_transport::TransportHints("compressed");

  auto callback = [&subscribed_image, &subscribed_stamp](const sensor_msgs::ImageConstPtr& msg) -> void {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    subscribed_image = cv_ptr->image.clone();
    subscribed_stamp = msg->header.stamp;
  };


  image_transport::Subscriber image_subscriber = it.subscribe(image_topic_name, 5, callback, rclcpp::VoidPtr(), hints);

  // Setup publisher
  auto vslam_publisher = node->create_publisher<pcl::PointCloud<pcl::PointXYZINormal>>("iris/vslam_data", 1);
  image_transport::Publisher image_publisher = it.advertise("iris/processed_image", 5);

  // Setup for OpenVSLAM
  iris::BridgeOpenVSLAM bridge;
  bridge.setup(vslam_config_path, vocab_path);

  std::chrono::system_clock::time_point m_start;
  rclcpp::Rate loop_rate(20);
  float accuracy = 0.5f;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr vslam_data(new pcl::PointCloud<pcl::PointXYZINormal>);
  std::ofstream ofs_time("vslam_time.csv");

  // Start main loop
  RCLCPP_INFO(node->get_logger(), "start main loop.");
  while (rclcpp::ok()) {
    if (!subscribed_image.empty()) {
      m_start = std::chrono::system_clock::now();  // start timer
      rclcpp::Time process_stamp = subscribed_stamp;

      // process OpenVSLAM
      bridge.execute(subscribed_image);
      bridge.setCriteria(recollection, accuracy);
      bridge.getLandmarksAndNormals(vslam_data, std::numeric_limits<float>::max());

      // Reset input
      subscribed_image = cv::Mat();

      // Update threshold to adjust the number of points
      if (vslam_data->size() < lower_threshold_of_points && accuracy > 0.10) accuracy -= 0.01f;
      if (vslam_data->size() > upper_threshold_of_points && accuracy < 0.90) accuracy += 0.01f;
      std::cout << "accuracy: " << accuracy << std::endl;
      {
        sensor_msgs::msg::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bridge.getFrame()).toImageMsg();
        image_publisher.publish(msg);
      }
      {
        pcl_conversions::toPCL(process_stamp, vslam_data->header.stamp);
        vslam_data->header.frame_id = "world";
        vslam_publisher.publish(vslam_data);
      }

      // Inform processing time
      std::stringstream ss;
      long time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_start).count();
      ss << "processing time= \033[35m"
         << time_ms
         << "\033[m ms";
      ofs_time << time_ms << std::endl;
      RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
    }
    publishPose(bridge.getCameraPose().inverse(), "iris/vslam_pose", node->get_clock().now());

    // Spin and wait
    loop_rate.sleep();
    rclcpp::spin_once();
  }

  RCLCPP_INFO(node->get_logger(), "Finalize openvslam_bridge::bridge_node");
  return 0;
}