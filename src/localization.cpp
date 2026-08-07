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
  prev_local_pose_ = local_pose_estimate;

  // pose_extrapolator_->AddPose(laser_data->timestamp(),
  //                             transform::Embed3D(local_pose_estimate));

  // local_map_builder_->AddPointCloud(
  //     ConvertPoint(laser_data->hitting_points()),
  //     transform::Project2D(pose_extrapolator_->cached_extrapolated_pose().pose),
  //     &local_pose_estimate, &local_pose_score, &is_keyframe);

  // local_map_builder_->AddLaserData(
  //     laser_data,
  //     transform::Project2D(pose_extrapolator_->cached_extrapolated_pose().pose),
  //     &local_pose_estimate, &local_pose_score, &is_keyframe);

  // local_map_builder_->AddLaserData(laser_data, &local_pose_estimate,
  //                                  &local_pose_score, &is_keyframe);

  // prev_local_pose_ = local_pose_estimate;
  // pose_extrapolator_->AddPose(laser_data->timestamp(),
  //                             transform::Embed3D(local_pose_estimate));

  LOG(INFO) << "local_pose_score = " << local_pose_score;

  // if (!is_keyframe) {
  //   return;
  // }

  // if (localization_status_ == LocalizationStatus::kSuccess) {
  //   pose_graph_->AddTrackingConstraint(
  //       laser_data->timestamp(), ConvertPoint(laser_data->hitting_points()),
  //       local_pose_estimate, local_pose_score);
  // }

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

void Localization::DistordPointCloud(const sensor::LaserDataPtr& laser_data) {
  const transform::Rigid3d start_pose =
      pose_extrapolator_->ExtrapolatePose(laser_data->timestamp());

  for (const sensor::TimedPointPtr timed_point : laser_data->hitting_points()) {
    const transform::Rigid3d curr_pose =
        pose_extrapolator_->ExtrapolatePose(timed_point->timestamp);
    timed_point->position =
        start_pose.inverse() * curr_pose * timed_point->position;
  }
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
      optimized_node->constraint_data->local_pose.inverse() * prev_local_pose_;

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

  // {
  //   static int count = 0;
  //   const MapLimits& map_limits = probability_grid_->map_limits();
  //   const int width = map_limits.cell_limits().num_x_cells;
  //   const int height = map_limits.cell_limits().num_y_cells;

  //   cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
  //   for (int y = 0; y < height; ++y) {
  //     for (int x = 0; x < width; ++x) {
  //       const Eigen::Array2i index(x, y);
  //       const double probability = probability_grid_->GetProbability(index);
  //       const uint8_t value = 255 * (1.0 - probability);
  //       image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
  //     }
  //   }

  //   if (pose_extrapolator_ == nullptr) {
  //     return score;
  //   }

  //   const transform::Rigid2d pose_estimate2 =
  //       optimized_node->optimized_pose *
  //       optimized_node->constraint_data->local_pose.inverse() *
  //       transform::Project2D(pose_extrapolator_->latest_pose());

  //   for (const sensor::TimedPointPtr& timed_point : laser_data.points())
  //   {
  //     const Eigen::Vector2d world_point =
  //         pose_estimate2 * timed_point->position.head<2>();

  //     const Eigen::Array2i proposed_xy_index =
  //         map_limits.GetCellIndex(world_point.cast<float>());

  //     if (map_limits.Contains(proposed_xy_index)) {
  //       image.at<cv::Vec3b>(proposed_xy_index.y(), proposed_xy_index.x()) =
  //           cv::Vec3b(0, 0, 255);
  //     }
  //   }

  //   cv::imwrite("/home/linjs/图片/local_match/match_" +
  //                   std::to_string(count++) + ".png",
  //               image);
  // }

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
  if (local_map_builder_ == nullptr || pose_extrapolator_ == nullptr) {
    local_map_builder_ = std::make_shared<LocalMapBuilder>();
    pose_extrapolator_ = std::make_shared<PoseExtrapolator>();
  }

  Track(laser_data);

  return;
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
  return ToMatrix4d(prev_local_pose_);

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

void Localization::Test() {
  // auto read_file = [](const std::string& filename)
  //     -> std::vector<std::pair<double, transform::Rigid2d>> {
  //   std::vector<std::pair<double, transform::Rigid2d>> poses;
  //   std::ifstream file(filename);
  //   if (!file.is_open()) {
  //     std::cerr << "Failed to open file: " << filename << std::endl;
  //     return poses;
  //   }

  //   std::string line;
  //   while (std::getline(file, line)) {
  //     double time = 0.0;
  //     double x = 0.0;
  //     double y = 0.0;
  //     double angle = 0.0;
  //     if (sscanf(line.c_str(), "time = %lf, [%lf, %lf, %lf]", &time, &x, &y,
  //                &angle) == 4) {
  //       transform::Rigid2d pose =
  //           transform::Rigid2d({x, y}, Eigen::Rotation2Dd(angle));
  //       poses.emplace_back(time, pose);
  //     }
  //   }

  //   file.close();
  //   LOG(INFO) << "poses = " << poses.size();
  //   return poses;
  // };

  // const auto add_poses = read_file("/home/linjs/test_ws/add_pose.txt");
  // const auto guess_poses = read_file("/home/linjs/test_ws/guess_pose.txt");
  // auto pose_extrapolator = std::make_shared<PoseExtrapolator>();
  // for (std::size_t idx = 0; idx < add_poses.size(); ++idx) {
  //   pose_extrapolator->AddPose(add_poses[idx].first,
  //                              transform::Embed3D(add_poses[idx].second));
  //   if (idx > 0) {
  //     const auto& guess_pose = transform::Project2D(
  //         pose_extrapolator->ExtrapolatePose(add_poses[idx].first));
  //     LOG(INFO) << "delta_time = "
  //               << add_poses[idx].first - guess_poses[idx - 1].first
  //               << ", delta = ["
  //               << guess_pose.translation().x() -
  //                      guess_poses[idx - 1].second.translation().x()
  //               << ", "
  //               << guess_pose.translation().y() -
  //                      guess_poses[idx - 1].second.translation().y()
  //               << ", "
  //               << guess_pose.rotation().angle() -
  //                      guess_poses[idx - 1].second.rotation().angle()
  //               << "]";
  //   }
  // }
}
}  // namespace solex_robot::navigation::localization_2d