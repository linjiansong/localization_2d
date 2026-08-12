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

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "common/options.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "solex_msgs/msg/localization_status.hpp"
#include "solex_msgs/srv/global_localization.hpp"
#include "src/localization.h"

namespace solex_robot::navigation::localization_2d {
class LocalizationNode : public rclcpp::Node {
 private:
  /* data */
 public:
  LocalizationNode();
  ~LocalizationNode() = default;

 private:
  options::LocalizationOptions LoadOptions();
  void HandleScanMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void HandleOdometryMessage(const nav_msgs::msg::Odometry::SharedPtr msg);
  void HandleImuMessage(const sensor_msgs::msg::Imu::SharedPtr msg);
  void HandleInitialposeMessage(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void HandleGridMapMessage(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void HandleGlobalLocalizationService(
      const solex_msgs::srv::GlobalLocalization::Request::SharedPtr request,
      solex_msgs::srv::GlobalLocalization::Response::SharedPtr response);

  void PublishTransform();
  void PublishRobotPose();
  void PublishLocalizationStatus();
  void PublishLocalMap();

 private:
  const options::LocalizationOptions options_;
  const std::unique_ptr<Localization> locator_;

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      grid_map_subscriber_;
  rclcpp::Service<solex_msgs::srv::GlobalLocalization>::SharedPtr
      global_localization_server_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<solex_msgs::msg::LocalizationStatus>::SharedPtr
      localization_status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      local_map_publisher_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr local_map_timer_;

  std::mutex mutex_;
};
}  // namespace solex_robot::navigation::localization_2d
