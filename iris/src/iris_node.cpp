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

#include "core/types.hpp"
#include "map/map.hpp"
#include "publish/publish.hpp"
#include "system/system.hpp"
#include <chrono>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <rclcpp/rclcpp.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


//
Eigen::Matrix4f listenTransform(tf::TransformListener& listener);

//
pcl::PointCloud<pcl::PointXYZINormal>::Ptr vslam_data(new pcl::PointCloud<pcl::PointXYZINormal>);
bool vslam_update = false;
void callback(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& msg)
{
  *vslam_data = *msg;
  if (vslam_data->size() > 0)
    vslam_update = true;
}

//
Eigen::Matrix4f T_recover = Eigen::Matrix4f::Zero();
pcl::PointCloud<pcl::PointXYZ>::Ptr whole_pointcloud = nullptr;
void callbackForRecover(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  ROS_INFO("/initial_pose is subscribed");

  float x = static_cast<float>(msg->pose.pose.position.x);
  float y = static_cast<float>(msg->pose.pose.position.y);
  float qw = static_cast<float>(msg->pose.pose.orientation.w);
  float qz = static_cast<float>(msg->pose.pose.orientation.z);

  float z = std::numeric_limits<float>::max();

  if (whole_pointcloud == nullptr) {
    std::cout << "z=0 because whole_pointcloud is nullptr" << std::endl;
    z = 0;
  } else {
    for (const pcl::PointXYZ& p : *whole_pointcloud) {
      constexpr float r2 = 5 * 5;  // [m^2]
      float dx = x - p.x;
      float dy = y - p.y;
      if (dx * dx + dy * dy < r2) {
        z = std::min(z, p.z);
      }
    }
  }

  T_recover.setIdentity();
  T_recover(0, 3) = x;
  T_recover(1, 3) = y;
  T_recover(2, 3) = z;
  float theta = 2 * std::atan2(qz, qw);
  Eigen::Matrix3f R;
  R << 0, 0, 1,
      -1, 0, 0,
      0, -1, 0;
  T_recover.topLeftCorner(3, 3) = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()).toRotationMatrix() * R;
  std::cout << "T_recover:\n"
            << T_recover << std::endl;
}

void writeCsv(std::ofstream& ofs, const rclcpp::Time& timestamp, const Eigen::Matrix4f& iris_pose)
{
  auto convert = [](const Eigen::MatrixXf& mat) -> Eigen::VectorXf {
    Eigen::MatrixXf tmp = mat.transpose();
    return Eigen::VectorXf(Eigen::Map<Eigen::VectorXf>(tmp.data(), mat.size()));
  };

  ofs << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
  ofs << timestamp.toSec() << " " << convert(iris_pose).transpose() << std::endl;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("iris_node");
  node->declare_parameter("iris_config_path", "");
  node->declare_parameter("pcd_path", "");

  // Setup subscriber
  auto vslam_subscriber = node->create_subscription<pcl::PointCloud<pcl::PointXYZINormal>>("iris/vslam_data", 5, callback);
  auto recover_pose_subscriber = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 5, callbackForRecover);
  tf::TransformListener listener;

  // Setup publisher
  auto target_pc_publisher = node->create_publisher<pcl::PointCloud<pcl::PointXYZI>>("iris/target_pointcloud", 1, true);
  auto whole_pc_publisher = node->create_publisher<pcl::PointCloud<pcl::PointXYZI>>("iris/whole_pointcloud", 1, true);
  auto source_pc_publisher = node->create_publisher<pcl::PointCloud<pcl::PointXYZI>>("iris/source_pointcloud", 1);
  auto iris_path_publisher = node->create_publisher<nav_msgs::msg::Path>("iris/iris_path", 1);
  auto vslam_path_publisher = node->create_publisher<nav_msgs::msg::Path>("iris/vslam_path", 1);
  auto correspondences_publisher = node->create_publisher<visualization_msgs::msg::Marker>("iris/correspondences", 1);
  auto scale_publisher = node->create_publisher<std_msgs::msg::Float32>("iris/align_scale", 1);
  auto processing_time_publisher = node->create_publisher<std_msgs::msg::Float32>("iris/processing_time", 1);
  // auto normal_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("iris/normals", 1);
  // auto covariance_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("iris/covariances", 1);
  iris::Publication publication;

  // Get rosparams
  std::string config_path, pcd_path;
  node->get_parameter("iris_config_path", config_path);
  node->get_parameter("pcd_path", pcd_path);
  RCLCPP_INFO(node->get_logger(), "config_path: %s, pcd_path: %s", config_path.c_str(), pcd_path.c_str());

  // Initialize config
  iris::Config config(config_path);

  // Load LiDAR map
  iris::map::Parameter map_param(
      pcd_path, config.voxel_grid_leaf, config.normal_search_leaf, config.submap_grid_leaf);
  std::shared_ptr<iris::map::Map> map = std::make_shared<iris::map::Map>(map_param, config.T_init);

  // Initialize system
  std::shared_ptr<iris::System> system = std::make_shared<iris::System>(config, map);

  std::chrono::system_clock::time_point m_start;
  Eigen::Matrix4f offseted_vslam_pose = config.T_init;
  Eigen::Matrix4f iris_pose = config.T_init;

  // Publish map
  iris::publishPointcloud(whole_pc_publisher, map->getSparseCloud(), node->get_clock().now());
  iris::publishPointcloud(target_pc_publisher, map->getTargetCloud(), node->get_clock().now());
  whole_pointcloud = map->getSparseCloud();
  std::ofstream ofs_track("trajectory.csv");
  std::ofstream ofs_time("iris_time.csv");

  iris::map::Info last_map_info;

  // Start main loop
  rclcpp::Rate loop_rate(20);
  ROS_INFO("start main loop.");
  while (rclcpp::ok()) {

    Eigen::Matrix4f T_vslam = listenTransform(listener);
    if (!T_recover.isZero()) {
      std::cout << "apply recover pose" << std::endl;
      system->specifyTWorld(T_recover);
      T_recover.setZero();
    }

    if (vslam_update) {
      vslam_update = false;
      m_start = std::chrono::system_clock::now();
      rclcpp::Time process_stamp;
      pcl_conversions::fromPCL(vslam_data->header.stamp, process_stamp);

      // Execution
      system->execute(2, T_vslam, vslam_data);

      // Publish for rviz
      system->popPublication(publication);
      iris::publishPointcloud(source_pc_publisher, publication.cloud, node->get_clock().now());
      iris::publishPath(iris_path_publisher, publication.iris_trajectory, node->get_clock().now());
      iris::publishPath(vslam_path_publisher, publication.offset_trajectory, node->get_clock().now());
      iris::publishCorrespondences(correspondences_publisher, publication.cloud, map->getTargetCloud(), publication.correspondences, node->get_clock().now());
      // iris::publishNormal(normal_publisher, publication.cloud, publication.normals, node->get_clock().now());
      // iris::publishCovariance(covariance_publisher, publication.cloud, publication.normals, node->get_clock().now());

      if (last_map_info != map->getLocalmapInfo()) {
        iris::publishPointcloud(target_pc_publisher, map->getTargetCloud(), node->get_clock().now());
      }
      last_map_info = map->getLocalmapInfo();
      std::cout << "map: " << last_map_info.toString() << std::endl;


      // Processing time
      long time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_start).count();
      std::stringstream ss;
      ss << "processing time= \033[35m"
         << time_ms
         << "\033[m ms";
      ofs_time << time_ms << std::endl;
      ROS_INFO("Iris/ALIGN: %s", ss.str().c_str());
      {
        std_msgs::Float32 scale;
        scale.data = iris::util::getScale(publication.T_align);
        scale_publisher.publish(scale);

        std_msgs::Float32 processing_time;
        processing_time.data = static_cast<float>(time_ms);
        processing_time_publisher.publish(processing_time);
      }

      offseted_vslam_pose = publication.offset_camera;
      iris_pose = publication.iris_camera;

      writeCsv(ofs_track, process_stamp, iris_pose);
    }

    iris::publishPose(offseted_vslam_pose, "iris/offseted_vslam_pose", node->get_clock().now());
    iris::publishPose(iris_pose, "iris/iris_pose", node->get_clock().now());


    // Spin and wait
    rclcpp::spin_once();
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Finalize the system");
  return 0;
}


Eigen::Matrix4f listenTransform(tf::TransformListener& listener)
{
  tf::StampedTransform transform;
  try {
    listener.lookupTransform("world", "iris/vslam_pose", rclcpp::Time(0), transform);
  } catch (...) {
  }

  double data[16];
  transform.getOpenGLMatrix(data);
  Eigen::Matrix4d T(data);
  return T.cast<float>();
}