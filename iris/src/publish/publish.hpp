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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>

namespace iris
{
// void publishImage(image_transport::Publisher& publisher, const cv::Mat& image, rclcpp::Time stamp);

void publishPose(const Eigen::Matrix4f& T, const std::string& child_frame_id, tf2_ros::TransformBroadcaster& tf_broadcaster, rclcpp::Time stamp);
// void publishPointcloud(const rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZI>::Ptr>::SharedPtr& publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, rclcpp::Time stamp);
void publishPath(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher, const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& path, rclcpp::Time stamp);
void publishCorrespondences(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const pcl::CorrespondencesPtr& correspondences,
    rclcpp::Time stamp);
void publishNormal(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    rclcpp::Time stamp);
void publishCovariance(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    rclcpp::Time stamp);

// void publishResetPointcloud(const rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZ>::Ptr>::SharedPtr& publisher, rclcpp::Time stamp);
void publishResetCorrespondences(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher, rclcpp::Time stamp);

std::function<void(const sensor_msgs::msg::Image::SharedPtr&)> imageCallbackGenerator(cv::Mat& subscribed_image);
std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr&)> compressedImageCallbackGenerator(cv::Mat& subscribed_image);

}  // namespace iris
