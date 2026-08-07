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

#include "src/localization.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <execution>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <vector>

#include "common/ceres_helper.h"
#include "include/scan_match/utility.h"

namespace solex_robot::navigation::localization_2d {
namespace {
constexpr double kRadianToDegree = 180.0 / M_PI;
constexpr double kDegreeToRadian = M_PI / 180.0;
constexpr double kMaxRoamingDistance = 100.0;  // meter
constexpr double kMaxRoamingAngle = 360.0;     // Degree

Eigen::Matrix4d ToMatrix4d(const transform::Rigid2d& rigid_pose) {
  Eigen::Matrix4d eigen_pose = Eigen::Matrix4d::Identity();
  eigen_pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(rigid_pose.rotation().angle(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  eigen_pose(0, 3) = rigid_pose.translation().x();
  eigen_pose(1, 3) = rigid_pose.translation().y();
  return eigen_pose;
}

std::vector<Eigen::Vector3d> ConvertPoint(
    const std::vector<sensor::TimedPointPtr>& timed_points) {
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.reserve(timed_points.size());
  for (const sensor::TimedPointPtr& timed_point : timed_points) {
    eigen_points.emplace_back(timed_point->position);
  }

  return eigen_points;
}
}  // namespace

Localization::~Localization() { pose_graph_->Finish(); }

void Localization::GlobalLocalization(const sensor::LaserDataPtr& laser_data) {
  CHECK_NOTNULL(pose_graph_);
  localization_status_ = LocalizationStatus::kRoaming;
  pose_graph_->Reset();

  pose_graph_->AddGlobalMatchConstraint(
      laser_data->timestamp(), ConvertPoint(laser_data->hitting_points()));

  local_map_builder_ = std::make_shared<LocalMapBuilder>();
  pose_extrapolator_ = std::make_shared<PoseExtrapolator>();
  pose_extrapolator_->AddPose(laser_data->timestamp(),
                              transform::Rigid3d::Identity());
}

void Localization::Relocalization(const sensor::LaserDataPtr& laser_data) {
  // CHECK_NOTNULL(fast_correlative_scan_matcher_);
  return;
}

void Localization::Track(const sensor::LaserDataPtr& laser_data) {
  CHECK_NOTNULL(pose_extrapolator_);

  transform::Rigid2d local_pose_estimate;
  float local_pose_score = 0.0;
  bool is_keyframe = false;

  local_map_builder_->AddLaserData(laser_data, &local_pose_estimate,
                                   &local_pose_score, &is_keyframe);

  pose_extrapolator_->AddPose(laser_data->timestamp(),
                              transform::Embed3D(local_pose_estimate));
  if (!is_keyframe) {
    return;
  }

  if (localization_status_ == LocalizationStatus::kSuccess) {
    pose_graph_->AddTrackingConstraint(
        laser_data->timestamp(), ConvertPoint(laser_data->hitting_points()),
        local_pose_estimate, local_pose_score);
  }

  // if (localization_status_ == LocalizationStatus::kSuccess) {
  //   pose_graph_->AddTrackingConstraint(laser_data->timestamp(),
  //                                   ConvertPoint(laser_data->hitting_points()),
  //                                   local_pose_estimate, local_pose_score);
  // } else if (localization_status_ == LocalizationStatus::kRoaming) {
  //   pose_graph_->AddTrackingConstraint(laser_data->timestamp(),
  //                                   ConvertPoint(laser_data->hitting_points()),
  //                                   local_pose_estimate, local_pose_score);
  // }
}

float Localization::CalculateMatchScore(
    const sensor::LaserDataPtr& laser_data) {
  if (pose_graph_ == nullptr || probability_grid_ == nullptr) {
    return 0.0;
  }

  NodePtr optimized_node = pose_graph_->optimized_node();
  if (optimized_node == nullptr) {
    return 0.0;
  }

  const transform::Rigid2d pose_estimate =
      optimized_node->optimized_pose *
      optimized_node->constraint_data->local_pose.inverse() *
      transform::Project2D(
          pose_extrapolator_->ExtrapolatePose(laser_data->timestamp()));

  const MapLimits& map_limits = probability_grid_->map_limits();

  float score = 0.0;
  for (const sensor::TimedPointPtr& timed_point :
       laser_data->hitting_points()) {
    const Eigen::Vector2d world_point =
        pose_estimate * timed_point->position.head<2>();

    const Eigen::Array2i proposed_xy_index =
        map_limits.GetCellIndex(world_point.cast<float>());
    const float probability =
        probability_grid_->GetProbability(proposed_xy_index);
    score += probability;
  }

  score /= static_cast<float>(laser_data->hitting_points().size());

  return score;
}

void Localization::EvaluateLocalizationStatus(
    const sensor::LaserDataPtr& laser_data) {
  const float score = CalculateMatchScore(laser_data);
  if (score > 0.3) {
    if (localization_status_ == LocalizationStatus::kSuccess) {
      return;
    } else {
      localization_status_ = LocalizationStatus::kSuccess;
      roaming_distance_ = 0.0;
      roaming_angle_ = 0.0;
    }
  } else {
    if (localization_status_ == LocalizationStatus::kSuccess) {
      localization_status_ = LocalizationStatus::kRoaming;
    } else if (localization_status_ == LocalizationStatus::kRoaming) {
      if (roaming_distance_ > kMaxRoamingDistance ||
          roaming_angle_ > kMaxRoamingAngle) {
        localization_status_ = LocalizationStatus::kFailed;
        LOG(INFO) << "Localize failed!";
      }
    }
  }
}

void Localization::AddLaserData(const sensor::LaserDataPtr& laser_data) {
  if (localization_status_ == LocalizationStatus::kUnknown ||
      localization_status_ == LocalizationStatus::kFailed) {
    return;
  } else if (localization_status_ == LocalizationStatus::kInitialization) {
    GlobalLocalization(laser_data);
  } else {
    // const auto t0 = std::chrono::steady_clock::now();
    Track(laser_data);
    // const auto t1 = std::chrono::steady_clock::now();
    // LOG(INFO)
    //     << "Track takes "
    //     << std::chrono::duration_cast<std::chrono::milliseconds>(t1 -
    //     t0).count()
    //     << "ms";
  }

  EvaluateLocalizationStatus(laser_data);
}

void Localization::AddGridMap(
    std::shared_ptr<ProbabilityGrid> probability_grid) {
  CHECK_NOTNULL(probability_grid);
  probability_grid_ = probability_grid;

  if (pose_graph_ != nullptr) {
    pose_graph_->Finish();
  }

  pose_graph_ = std::make_shared<PoseGraph>(probability_grid);
  pose_graph_->Init();
}

void Localization::AddInitialPose(const Eigen::Matrix4d& initial_pose) {
  // initial_pose_ = initial_pose; // Todo
  localization_status_ = LocalizationStatus::kInitialization;
}

void Localization::AddImuData(const sensor::ImuDataPtr& imu_data) {
  return;
  if (pose_extrapolator_ != nullptr) {
    pose_extrapolator_->AddImuData(*imu_data);
  }
}

void Localization::AddOdometryData(
    const sensor::OdometryDataPtr& odometry_data) {
  return;
  if (pose_extrapolator_ != nullptr) {
    pose_extrapolator_->AddOdometryData(*odometry_data);
  }
}

const Eigen::Matrix4d Localization::GetLatestPose(const double timestamp) {
  if (pose_graph_ == nullptr || pose_extrapolator_ == nullptr) {
    return Eigen::Matrix4d::Identity();
  }

  NodePtr optimized_node = pose_graph_->optimized_node();
  if (optimized_node == nullptr) {
    return Eigen::Matrix4d::Identity();
  }

  const transform::Rigid2d pose_estimate =
      optimized_node->optimized_pose *
      optimized_node->constraint_data->local_pose.inverse() *
      transform::Project2D(pose_extrapolator_->ExtrapolatePose(timestamp));

  return ToMatrix4d(pose_estimate);
}

const ProbabilityGrid* Localization::GetLocalMap() {
  if (local_map_builder_ == nullptr) {
    return nullptr;
  }

  const auto& submaps = local_map_builder_->GetLocalMap();
  if (submaps.empty()) {
    return nullptr;
  }

  return submaps.front()->probability_grid();
}

}  // namespace solex_robot::navigation::localization_2d