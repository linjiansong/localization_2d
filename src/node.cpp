////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//            Copyright© 2026 Solex Robot, All Rights Reserved.               //
//                                                                            //
//  All users are hereby notified that the materials in the form of digital   //
//  information available from this software (content, designs, color         //
//  schemes, graphic styles, images, logo, text, and videos) comes protected  //
//  under International Copyright Laws. Therefore it should not be reproduced //
//  in any form digital or offline without prior written permission of        //
//  Solex Robot.                                                              //
//                                                                            //
//  Any unauthorized reprint or material usage (Solex Robot) either manually  //
//  or digitally, is strictly prohibited.                                     //
//                                                                            //
//  Any further unauthorized digital copying of this material via copying,    //
//  publication, reproduction or distribution of copyrighted works is an      //
//  infringement of the copyright owners' rights may be the subject of the    //
//  copyright of performers' protection under the Copyright Act. For such     //
//  illegal activities you will be strictly liable to Solox Robot for any and //
//  or all damages (including recovery of attorneys' fees) which may be       //
//  suffered and or incurred as a result of your infringement.                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "src/node.h"

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "include/scan_match/distance_field.h"
#include "include/scan_match/probability_grid.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace solex_robot::navigation::localization_2d {
namespace {
Eigen::Vector3d TransformPoint(const Eigen::Matrix4d& transform,
                               const Eigen::Vector3d& point) {
  return transform.block<3, 3>(0, 0) * point + transform.block<3, 1>(0, 3);
}
}  // namespace

LocalizationNode::LocalizationNode() : rclcpp::Node("sensor_subscriber_node") {
  std::cout << "You are zhuangzhuang" << std::endl;
  // 使用 SensorDataQoS，这对于传感器高频数据至关重要
  auto qos = rclcpp::SensorDataQoS();

  // 订阅 LaserScan
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->HandleScanMessage(msg);
      });

  // 订阅 Odometry
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->HandleOdometryMessage(msg);
      });

  // 订阅IMU
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->HandleImuMessage(msg);
      });

  // 订阅initialpose
  initial_pose_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos,
          [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                     msg) { this->HandleInitialposeMessage(msg); });

  // 订阅grid map
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  grid_map_subscriber_ =
      this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/map",
          map_qos,  // 使用专用的 map_qos
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            this->HandleGridMapMessage(msg);
          });

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  point_cloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", 10);

  pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  locator_ = std::make_unique<Localization>();
  locator_->Init();

  // 创建定时器，10Hz 发布
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    this->PublishPointCloud();
    this->PublishTransform();
    this->PublishRobotPose();
  });

  LOG(INFO) << "Sensor Subscriber Node has been started.";
}

void LocalizationNode::HandleScanMessage(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  static Eigen::Isometry3d laser_to_base_transform =
      Eigen::Isometry3d::Identity();
  static bool tf_initialized = false;

  if (!tf_initialized) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
          "base_link", msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(1.0));

      laser_to_base_transform =
          tf2::transformToEigen(transform_stamped.transform);
      tf_initialized = true;
      LOG(INFO) << "Successfully obtained laser to base_link TF!";
      LOG(INFO) << "laser_to_base_transform matrix =\n"
                << laser_to_base_transform.matrix();
    } catch (tf2::TransformException& ex) {
      LOG(ERROR) << "Could not get transform from " << msg->header.frame_id
                 << "to base_link";
      return;  // 如果还没获取到TF，暂时跳过该帧
    }
  }

  const double start_time = rclcpp::Time(msg->header.stamp).seconds();
  std::vector<sensor::TimedPointCloudPtr> timed_points;
  timed_points.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float range = msg->ranges[i];
    const double time_offset = msg->time_increment * i;  // second

    // 过滤超出物理量程的点
    if (range < msg->range_min || range > msg->range_max) {
      continue;
    }

    // 计算角度
    const double angle = msg->angle_min + i * msg->angle_increment;

    // 转换为极坐标
    const Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d position =
        laser_to_base_transform * rotation * (Eigen::Vector3d(range, 0.0, 0.0));

    sensor::TimedPointCloudPtr timed_point =
        std::make_shared<sensor::TimedPointCloud>();
    timed_point->position = position;
    timed_point->timestamp = start_time + time_offset;
    timed_points.emplace_back(timed_point);
  }

  CHECK_NOTNULL(locator_);
  if (!timed_points.empty()) {
    sensor::LaserData laser_data = sensor::LaserData(start_time);
    *(laser_data.mutable_points()) = timed_points;
    locator_->AddLaserData(laser_data);
  }
}

void LocalizationNode::HandleOdometryMessage(
    const nav_msgs::msg::Odometry::SharedPtr msg) {}

void LocalizationNode::HandleImuMessage(
    const sensor_msgs::msg::Imu::SharedPtr msg) {}

void LocalizationNode::HandleInitialposeMessage(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  LOG(INFO) << "Receive initial pose";
  Eigen::Affine3d affine;
  tf2::fromMsg(msg->pose.pose, affine);
  const Eigen::Matrix4d rigid_pose = affine.matrix();
  locator_->AddInitialPose(rigid_pose);
}

void LocalizationNode::HandleGridMapMessage(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  LOG(INFO)
      << "------------------------------------------------------------------";
  LOG(INFO) << "Received a new grid map. Resolution: " << msg->info.resolution
            << ", Size: " << msg->info.width << "x" << msg->info.height;
  LOG(INFO)
      << "------------------------------------------------------------------";

  // 1. 检查数据合法性
  if (msg->data.empty()) {
    LOG(WARNING) << "Received an empty grid map!";
    return;
  }

  const double resolution = msg->info.resolution;
  const int width = msg->info.width;
  const int height = msg->info.height;
  LOG(INFO) << "width = " << width << ", height = " << height;

  std::vector<int8_t> occupied_cells;
  occupied_cells.reserve(width * height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      // ROS数据是行主序(Row-major)，(0,0) 为左下角
      const int8_t value = msg->data[y * width + x];
      occupied_cells.emplace_back(value);
    }
  }

  const std::vector<float> distance_field =
      ComputeDistanceField(occupied_cells, height, width);
  std::vector<uint16_t> correspondence_cost_cells;
  correspondence_cost_cells.resize(occupied_cells.size());
  LOG(INFO) << "distance_field = " << distance_field.size();
  const float scale =
      (kMaxCorrespondenceCost - kMinCorrespondenceCost) / (kValueCount - 2.f);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const float distance = distance_field[y * width + x];
      const float probability = std::exp(-(distance * distance) / 1.0);
      const float cost = std::clamp(1.f - probability, kMinCorrespondenceCost,
                                    kMaxCorrespondenceCost);
      const float value = (cost - kMinCorrespondenceCost) / scale + 1.0f;
      const int flat_index = (height - y - 1) * width + x;
      correspondence_cost_cells[flat_index] = value;
    }
  }

  // 计算左上角坐标 (max)
  // 这是 Cartographer MapLimits 所需要的坐标基准
  const Eigen::Vector2d grid_origin(
      msg->info.origin.position.x,
      msg->info.origin.position.y + height * resolution);
  const MapLimits map_limits(resolution, grid_origin,
                             CellLimits(width, height));

  auto probability_grid = std::make_shared<ProbabilityGrid>(
      map_limits, correspondence_cost_cells, nullptr);

  {
    cv::Mat image(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int flat_index = y * width + x;
        const double distance = distance_field[flat_index];
        const float probability = std::exp(-(distance * distance) / 25.0);
        const uchar pixel_value = static_cast<uchar>(
            std::clamp((1.0f - probability) * 255.0f, 0.0f, 255.0f));
        image.at<uchar>(height - 1 - y, x) = pixel_value;
      }
    }

    cv::imwrite("/home/linjs/图片/correspondence_cost_cells.png", image);
  }

  probability_grid->VisualizeGrid();

  locator_->AddGridMap(probability_grid);

  LOG(INFO) << "Grid map successfully loaded in locator.";
}

void LocalizationNode::PublishPointCloud() {
  // const auto keyframe_buffer = locator_->keyframe_buffer();

  // pcl::PointCloud<pcl::PointXYZI> cloud_points;
  // for (const auto& keyframe : keyframe_buffer) {
  //   for (const sensor::TimedPointCloud& timed_point : keyframe->point_cloud.points) {
  //     const Eigen::Vector3d transformed_point =
  //         TransformPoint(keyframe->optimized_pose, timed_point.position);
  //     pcl::PointXYZI pcl_point;
  //     pcl_point.x = transformed_point.x();
  //     pcl_point.y = transformed_point.y();
  //     pcl_point.z = transformed_point.z();
  //     pcl_point.intensity = 0.0;
  //     cloud_points.push_back(pcl_point);
  //   }
  // }

  // // Convert to ROS2 message
  // sensor_msgs::msg::PointCloud2 output_msg;
  // pcl::toROSMsg(cloud_points, output_msg);

  // // Add timestamp
  // output_msg.header.stamp = this->now();
  // output_msg.header.frame_id = "map";

  // // Publish
  // point_cloud_publisher_->publish(output_msg);
}

void LocalizationNode::PublishRobotPose() {
  Eigen::Matrix4d delta_transform = Eigen::Matrix4d::Identity();
  delta_transform.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix4d curr_pose = locator_->curr_pose() * delta_transform;

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = curr_pose(0, 3);
  pose_msg.pose.position.y = curr_pose(1, 3);
  pose_msg.pose.position.z = curr_pose(2, 3);

  Eigen::Matrix3d rotation_matrix = curr_pose.block<3, 3>(0, 0);
  Eigen::Quaterniond q(rotation_matrix);

  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_publisher_->publish(pose_msg);
}

void LocalizationNode::PublishTransform() {
  const rclcpp::Time current_time = this->now();

  // ==================== map -> odom ====================
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = current_time;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform =
      tf2::eigenToTransform(Eigen::Affine3d::Identity()).transform;
  tf_broadcaster_->sendTransform(map_to_odom);

  // ================= odom -> base_link =================
  geometry_msgs::msg::TransformStamped odom_to_base;
  odom_to_base.header.stamp = current_time;
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "base_link";
  Eigen::Matrix4d odom_pose = Eigen::Matrix4d::Identity();
  odom_to_base.transform =
      tf2::eigenToTransform(Eigen::Affine3d(locator_->curr_pose())).transform;
  tf_broadcaster_->sendTransform(odom_to_base);
}

}  // namespace solex_robot::navigation::localization_2d
