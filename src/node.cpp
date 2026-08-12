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
constexpr double kSecondToNanoSecond = 1.e9;
constexpr double kDegreeToRadian = M_PI / 180.0;
}  // namespace

LocalizationNode::LocalizationNode()
    : rclcpp::Node("sensor_subscriber_node"),
      options_(LoadOptions()),
      locator_(std::make_unique<Localization>(
          options_.local_map_builder_options, options_.pose_graph_options)) {
  // 使用 SensorDataQoS，这对于传感器高频数据至关重要
  auto qos = rclcpp::SensorDataQoS();

  // 订阅 LaserScan
  if (options_.sensor_options.use_laser) {
    LOG(INFO) << "subscribe laser scan, topic = "
              << options_.sensor_options.laser_topic;
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        options_.sensor_options.laser_topic, qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->HandleScanMessage(msg);
        });
  }

  // 订阅 Odometry
  if (options_.sensor_options.use_odometry) {
    LOG(INFO) << "subscribe odometry, topic = "
              << options_.sensor_options.odometry_topic;
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        options_.sensor_options.odometry_topic, qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->HandleOdometryMessage(msg);
        });
  }

  // 订阅 IMU
  if (options_.sensor_options.use_imu) {
    LOG(INFO) << "subscribe imu data, topic = "
              << options_.sensor_options.imu_topic;
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        options_.sensor_options.imu_topic, qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          this->HandleImuMessage(msg);
        });
  }

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

  global_localization_server_ =
      this->create_service<solex_msgs::srv::GlobalLocalization>(
          "global_localization_service",
          std::bind(&LocalizationNode::HandleGlobalLocalizationService, this,
                    std::placeholders::_1, std::placeholders::_2));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 50);

  local_map_publisher_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 1);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 创建定时器，50Hz 发布
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() {
    this->PublishTransform();
    this->PublishRobotPose();
  });

  // local_map_timer_ = this->create_wall_timer(
  //     std::chrono::milliseconds(1000),
  //     std::bind(&LocalizationNode::PublishLocalMap, this));

  LOG(INFO) << "Sensor Subscriber Node has been started.";
}

options::LocalizationOptions LocalizationNode::LoadOptions() {
  options::LocalizationOptions options;

  // SensorOptions
  options.sensor_options.laser_topic =
      this->declare_parameter("sensors.lidar.topic", "/scan");
  options.sensor_options.use_laser =
      this->declare_parameter("sensors.lidar.enabled", true);
  options.sensor_options.imu_topic =
      this->declare_parameter("sensors.imu.topic", "/imu/data");
  options.sensor_options.use_imu =
      this->declare_parameter("sensors.imu.enabled", true);
  options.sensor_options.odometry_topic =
      this->declare_parameter("sensors.odom.topic", "/odom");
  options.sensor_options.use_odometry =
      this->declare_parameter("sensors.odom.enabled", true);

  // LocalMapBuilderOptions
  options.local_map_builder_options.use_real_time_correlative_scan_match =
      this->declare_parameter(
          std::string("local_map_builder.use_online_correlative_scan_match"),
          true);
  std::string prefix = "local_map_builder.real_time_correlative_scan_matcher.";
  options.local_map_builder_options.real_time_correlative_scan_matcher_options
      .linear_search_window = this->declare_parameter(
      prefix + std::string("linear_search_window"), 0.1);
  options.local_map_builder_options.real_time_correlative_scan_matcher_options
      .angular_search_window =
      this->declare_parameter(prefix + std::string("angular_search_window"),
                              5.0) *
      kDegreeToRadian;
  options.local_map_builder_options.real_time_correlative_scan_matcher_options
      .translation_delta_cost_weight = this->declare_parameter(
      prefix + std::string("translation_delta_cost_weight"), 10.0);
  options.local_map_builder_options.real_time_correlative_scan_matcher_options
      .rotation_delta_cost_weight = this->declare_parameter(
      prefix + std::string("rotation_delta_cost_weight"), 0.1);

  // Motion Filter
  prefix = "local_map_builder.motion_filter.";
  options.local_map_builder_options.motion_filter_options.max_time_seconds =
      (float)this->declare_parameter(prefix + std::string("max_time_seconds"),
                                     5.0);
  options.local_map_builder_options.motion_filter_options.max_distance =
      (float)this->declare_parameter(prefix + std::string("max_distance"), 0.2);
  options.local_map_builder_options.motion_filter_options.max_angle =
      (float)this->declare_parameter(prefix + std::string("max_angle"), 5.0) *
      kDegreeToRadian;

  // Pose Graph
  prefix = "pose_graph.";
  options.pose_graph_options.optimize_every_nodes =
      this->declare_parameter(prefix + std::string("optimize_every_nodes"), 10);
  options.pose_graph_options.max_node_buffer_length = this->declare_parameter(
      prefix + std::string("max_node_buffer_length"), 40);
  options.pose_graph_options.min_global_localization_score =
      (float)this->declare_parameter(
          prefix + std::string("min_global_localization_score"), 0.8);
  options.pose_graph_options.min_relocalization_score =
      (float)this->declare_parameter(
          prefix + std::string("min_relocalization_score"), 0.6);
  options.pose_graph_options.min_tracking_score = 
      (float)this->declare_parameter(
          prefix + std::string("min_tracking_score"), 0.5);

  // Real Time Correlative Scan Match
  prefix = "pose_graph.real_time_correlative_scan_matcher.";
  options.pose_graph_options.real_time_correlative_scan_matcher_options
      .linear_search_window = this->declare_parameter(
      prefix + std::string("linear_search_window"), 0.1);
  options.pose_graph_options.real_time_correlative_scan_matcher_options
      .angular_search_window =
      this->declare_parameter(prefix + std::string("angular_search_window"),
                              5.0) *
      kDegreeToRadian;
  options.pose_graph_options.real_time_correlative_scan_matcher_options
      .translation_delta_cost_weight = this->declare_parameter(
      prefix + std::string("translation_delta_cost_weight"), 10.0);
  options.pose_graph_options.real_time_correlative_scan_matcher_options
      .rotation_delta_cost_weight = this->declare_parameter(
      prefix + std::string("rotation_delta_cost_weight"), 0.1);

  // Fast Correlative Scan Match
  prefix = "pose_graph.fast_correlative_scan_matcher.";
  options.pose_graph_options.fast_correlative_scan_matcher_options
      .linear_search_window = this->declare_parameter(
      prefix + std::string("linear_search_window"), 8.0);
  options.pose_graph_options.fast_correlative_scan_matcher_options
      .angular_search_window =
      this->declare_parameter(prefix + std::string("angular_search_window"),
                              30.0) *
      kDegreeToRadian;
  options.pose_graph_options.fast_correlative_scan_matcher_options
      .branch_and_bound_depth = this->declare_parameter(
      prefix + std::string("branch_and_bound_depth"), 6);

  LOG(INFO) << "***************** Current Node Parameters *****************";
  auto all_params = this->get_parameters(this->list_parameters({}, 10).names);
  for (const auto& param : all_params) {
    LOG(INFO) << "Name: " << param.get_name()
              << " | Value: " << param.value_to_string();
  }
  LOG(INFO) << "***********************************************************";

  return options;
}

void LocalizationNode::HandleScanMessage(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  CHECK_NOTNULL(msg);
  std::unique_lock<std::mutex> lock(mutex_);

  static Eigen::Isometry3d laser_to_base_transform =
      Eigen::Isometry3d::Identity();
  static bool laser_to_base_tf_initialized = false;

  if (!laser_to_base_tf_initialized) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
          "base_link", msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(1.0));

      laser_to_base_transform =
          tf2::transformToEigen(transform_stamped.transform);
      laser_to_base_tf_initialized = true;
      LOG(INFO) << "Successfully obtained laser to base_link TF.";
      LOG(INFO) << "laser_to_base_transform matrix =\n"
                << laser_to_base_transform.matrix();
    } catch (tf2::TransformException& ex) {
      LOG(ERROR) << "Could not get transform from " << msg->header.frame_id
                 << "to base_link";
      return;  // 如果还没获取到TF，暂时跳过该帧
    }
  }

  const double start_time = rclcpp::Time(msg->header.stamp).seconds();
  sensor::LaserDataPtr laser_data =
      std::make_shared<sensor::LaserData>(start_time);
  for (size_t index = 0; index < msg->ranges.size(); ++index) {
    const float range = msg->ranges[index];
    if (range < msg->range_min) {
      continue;
    }

    // 计算角度
    const double time_offset = msg->time_increment * index;  // second
    const double angle = msg->angle_min + index * msg->angle_increment;

    // 转换为极坐标
    const Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d position =
        laser_to_base_transform * rotation * (Eigen::Vector3d(range, 0.0, 0.0));

    sensor::TimedPointPtr timed_point = std::make_shared<sensor::TimedPoint>();
    timed_point->position = position;
    timed_point->timestamp = start_time + time_offset;

    // 过滤超出物理量程的点
    if (range > msg->range_max) {
      laser_data->mutable_missing_points()->emplace_back(timed_point);
    } else {
      laser_data->mutable_hitting_points()->emplace_back(timed_point);
    }
  }

  locator_->AddLaserData(laser_data);
}

void LocalizationNode::HandleOdometryMessage(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  CHECK_NOTNULL(msg);
  std::unique_lock<std::mutex> lock(mutex_);

  const transform::Rigid3d eigen_pose = transform::Rigid3d(
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z),
      Eigen::Quaterniond(
          msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

  sensor::OdometryDataPtr odometry_data =
      std::make_shared<sensor::OdometryData>(
          rclcpp::Time(msg->header.stamp).seconds());
  odometry_data->set_pose(eigen_pose);

  locator_->AddOdometryData(odometry_data);
}

void LocalizationNode::HandleImuMessage(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  CHECK_NOTNULL(msg);
  std::unique_lock<std::mutex> lock(mutex_);

  // LOG(INFO) << "delta time = " << this->now().seconds() -
  // rclcpp::Time(msg->header.stamp).seconds();

  static Eigen::Isometry3d imu_to_base_transform =
      Eigen::Isometry3d::Identity();
  static bool imu_to_base_tf_initialized = false;

  if (!imu_to_base_tf_initialized) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
          "base_link", msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(1.0));

      imu_to_base_transform =
          tf2::transformToEigen(transform_stamped.transform);
      imu_to_base_tf_initialized = true;
      LOG(INFO) << "Successfully obtained imu to base_link TF.";
      LOG(INFO) << "imu_to_base_transform matrix =\n"
                << imu_to_base_transform.matrix();
    } catch (tf2::TransformException& ex) {
      LOG(ERROR) << "Could not get transform from " << msg->header.frame_id
                 << "to base_link";
      return;  // 如果还没获取到TF，暂时跳过该帧
    }
  }

  const Eigen::Vector3d angular_velocity(msg->angular_velocity.x,
                                         msg->angular_velocity.y,
                                         msg->angular_velocity.z);
  const Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x,
                                            msg->linear_acceleration.y,
                                            msg->linear_acceleration.z);

  sensor::ImuDataPtr imu_data = std::make_shared<sensor::ImuData>(
      rclcpp::Time(msg->header.stamp).seconds());
  imu_data->set_angular_velocity(imu_to_base_transform.rotation() *
                                 angular_velocity);
  imu_data->set_linear_acceleration(imu_to_base_transform.rotation() *
                                    linear_acceleration);
  locator_->AddImuData(imu_data);
}

void LocalizationNode::HandleInitialposeMessage(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  LOG(INFO) << "Receive initial pose";
  // const transform::Rigid2d rigid_pose = transform::Rigid2d(
  //     Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y),
  //     tf2::getYaw(msg->pose.pose.orientation));

  Eigen::Affine3d affine;
  tf2::fromMsg(msg->pose.pose, affine);
  const Eigen::Matrix4d eigen_pose = affine.matrix();
  double yaw = std::atan2(eigen_pose(1, 0), eigen_pose(0, 0));
  const transform::Rigid2d rigid_pose = transform::Rigid2d(
      Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y),
      yaw);

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
      const int flat_index = (height - y - 1) * width + (width - x - 1);
      correspondence_cost_cells[flat_index] = value;
    }
  }

  // 计算左上角坐标 (max)
  // 这是 Cartographer MapLimits 所需要的坐标基准
  const Eigen::Vector2d grid_max(
      msg->info.origin.position.x + width * resolution,
      msg->info.origin.position.y + height * resolution);
  const MapLimits map_limits(resolution, grid_max, CellLimits(width, height));

  auto probability_grid = std::make_shared<ProbabilityGrid>(
      map_limits, correspondence_cost_cells, nullptr);

  // {
  //   cv::Mat image(height, width, CV_8UC1);
  //   for (int y = 0; y < height; ++y) {
  //     for (int x = 0; x < width; ++x) {
  //       const int flat_index = y * width + x;
  //       const double distance = distance_field[flat_index];
  //       const float probability = std::exp(-(distance * distance) / 25.0);
  //       const uchar pixel_value = static_cast<uchar>(
  //           std::clamp((1.0f - probability) * 255.0f, 0.0f, 255.0f));
  //       image.at<uchar>(height - 1 - y, x) = pixel_value;
  //     }
  //   }

  //   cv::imwrite("/home/linjs/图片/correspondence_cost_cells.png", image);
  // }

  locator_->AddGridMap(probability_grid);

  LOG(INFO) << "Grid map successfully loaded in locator.";
}

void LocalizationNode::HandleGlobalLocalizationService(
    const solex_msgs::srv::GlobalLocalization::Request::SharedPtr request,
    solex_msgs::srv::GlobalLocalization::Response::SharedPtr response) {
  if (request->global_localizaiton_quest) {
    LOG(INFO) << "Request global localization";
    locator_->RequestGlobalLocalization();
    response->result = true;
  } else {
    response->result = false;
  }
}

void LocalizationNode::PublishRobotPose() {
  std::unique_lock<std::mutex> lock(mutex_);

  const rclcpp::Time current_time = this->now();
  const Eigen::Matrix4d curr_pose =
      locator_->GetLatestPose(current_time.seconds());

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = current_time;
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
  std::unique_lock<std::mutex> lock(mutex_);

  const rclcpp::Time current_time = this->now();
  const Eigen::Matrix4d curr_pose =
      locator_->GetLatestPose(current_time.seconds());

  // ==================== map -> odom ====================
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = current_time;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform =
      tf2::eigenToTransform(Eigen::Affine3d(curr_pose)).transform;
  tf_broadcaster_->sendTransform(map_to_odom);

  // ================= odom -> base_link =================
  geometry_msgs::msg::TransformStamped odom_to_base;
  odom_to_base.header.stamp = current_time;
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "base_link";
  odom_to_base.transform =
      tf2::eigenToTransform(Eigen::Affine3d::Identity()).transform;
  tf_broadcaster_->sendTransform(odom_to_base);
}

void LocalizationNode::PublishLocalMap() {
  const ProbabilityGrid* probability_grid = locator_->GetLocalMap();
  if (probability_grid == nullptr) {
    return;
  }

  const MapLimits& map_limits = probability_grid->map_limits();
  const int width = map_limits.cell_limits().num_x_cells;
  const int height = map_limits.cell_limits().num_y_cells;
  auto map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();

  map_msg->header.stamp = this->now();
  map_msg->header.frame_id = "map";
  map_msg->info.resolution = map_limits.resolution();
  map_msg->info.width = width;
  map_msg->info.height = height;

  map_msg->info.origin.position.x =
      map_limits.max().x() - map_limits.resolution() * width;
  map_msg->info.origin.position.y =
      map_limits.max().y() - map_limits.resolution() * height;
  map_msg->info.origin.position.z = 0.0;
  map_msg->info.origin.orientation.w = 1.0;

  map_msg->data.resize(width * height);
  for (int y_index = 0; y_index < height; ++y_index) {
    for (int x_index = 0; x_index < width; ++x_index) {
      const float probability =
          probability_grid->GetProbability(Eigen::Array2i(x_index, y_index));
      const int flat_index = (height - y_index - 1) * width + x_index;
      const int value = probability > 0.5 ? 100 : 0;
      map_msg->data[flat_index] = value;
    }
  }

  local_map_publisher_->publish(*map_msg);
}

}  // namespace solex_robot::navigation::localization_2d