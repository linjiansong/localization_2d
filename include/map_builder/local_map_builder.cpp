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

#include "include/map_builder/local_map_builder.h"

#include <glog/logging.h>

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kDegreeToRadian = M_PI / 180.0;
constexpr double kRadianToDegree = 180.0 / M_PI;
constexpr double kKeyframeMinDistance = 0.2;  // meter
constexpr double kKeyframeMinAngle = 2.0;     // degree

}  // namespace

LocalMapBuilder::LocalMapBuilder()
    : ndt_aligner_(std::make_unique<NDTAligner>()),
      icp_aligner_(std::make_unique<ICPAligner>()),
      active_submaps_(std::make_unique<ActiveSubmap>()),
      ceres_scan_matcher_(std::make_unique<CeresScanMatcher2D>()) {
  RealTimeCorrelativeScanMatcherOptions2D real_time_options;
  real_time_options.linear_search_window = 0.2;
  real_time_options.angular_search_window = 2.5 * kDegreeToRadian;
  real_time_options.translation_delta_cost_weight = 0.1;
  real_time_options.rotation_delta_cost_weight = 0.1;
  real_time_correlative_scan_matcher_ =
      std::make_unique<RealTimeCorrelativeScanMatcher2D>(real_time_options);
}

bool LocalMapBuilder::IsKeyframe(const transform::Rigid2d& current_pose) {
  const transform::Rigid2d delta_pose =
      last_keyframe_pose_.inverse() * current_pose;
  const double distance = delta_pose.translation().norm();  // meter
  const double angle =
      delta_pose.rotation().angle() * kRadianToDegree;  // degree

  return distance > kKeyframeMinDistance || angle > kKeyframeMinAngle;
}

void LocalMapBuilder::MatchLocalMap(
    const transform::Rigid2d& pose_prediction,
    const std::vector<Eigen::Vector3d>& point_cloud,
    transform::Rigid2d* pose_estimate, float* score) {
  if (active_submaps_->submaps().empty()) {
    return;
  }

  const ProbabilityGrid* probability_grid =
      active_submaps_->submaps().front()->probability_grid();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  float real_time_score = 0.0;
  real_time_correlative_scan_matcher_->Match(
      pose_prediction, point_cloud, *probability_grid, &initial_ceres_pose,
      &real_time_score);
  // LOG(INFO) << "real_time_score = " << real_time_score;

  transform::Rigid2d ceres_pose_estimate = initial_ceres_pose;
  float ceres_match_score = real_time_score;
  // ceres_scan_matcher_->Match(initial_ceres_pose, point_cloud,
  // probability_grid,
  //                            &ceres_pose_estimate, &ceres_match_score);
  // LOG(INFO) << "ceres_match_score = " << ceres_match_score;

  *pose_estimate = ceres_pose_estimate;
  *score = ceres_match_score;
}

void LocalMapBuilder::AddLaserData(const sensor::LaserDataPtr& laser_data,
                                   const transform::Rigid2d& initial_pose,
                                   transform::Rigid2d* pose_estimate,
                                   float* score, bool* is_keyframe) {
  std::vector<Eigen::Vector3d> point_cloud;
  point_cloud.reserve(laser_data->hitting_points().size());
  for (const auto& hitting_point : laser_data->hitting_points()) {
    point_cloud.emplace_back(hitting_point->position);
  }
  MatchLocalMap(initial_pose, point_cloud, pose_estimate, score);

  // LOG(INFO) << "pose_estimate = [" << pose_estimate->translation().x() << ", "
  //           << pose_estimate->translation().x() << ", "
  //           << pose_estimate->rotation().angle() << "], score = " << *score;

  // update laser pose
  laser_data->set_pose(transform::Embed3D(*pose_estimate));

  active_submaps_->InsertLaserData(laser_data);

  *is_keyframe = IsKeyframe(*pose_estimate);
}

void LocalMapBuilder::AddPointCloud(std::vector<Eigen::Vector3d> point_cloud,
                                    const transform::Rigid2d& initial_pose,
                                    transform::Rigid2d* pose_estimate,
                                    float* score, bool* is_keyframe) {
  if (estimated_poses_.empty()) {
    last_keyframe_pose_ = initial_pose;
    // ndt_aligner_ = std::make_unique<NDTAligner>();
    // ndt_aligner_->AddPointCloud(point_cloud);

    icp_aligner_ = std::make_unique<ICPAligner>();
    icp_aligner_->AddPointCloud(point_cloud);
    estimated_poses_.emplace_back(initial_pose);

    *pose_estimate = initial_pose;
    *score = 1.0;
    *is_keyframe = true;
    return;
  }

  // 此时local map位于匹配器内部，直接配准即可
  // ndt_aligner_->Align(point_cloud, initial_pose, pose_estimate, score);
  icp_aligner_->Align(point_cloud, initial_pose, pose_estimate, score);
  estimated_poses_.emplace_back(*pose_estimate);

  transform::Rigid2d delta_pose = initial_pose.inverse() * (*pose_estimate);
  // LOG(INFO) << "delta = " << delta_pose.translation().x() << ", "
  //           << delta_pose.translation().x() << ", "
  //           << delta_pose.rotation().angle() * 180 / M_PI;

  if (!IsKeyframe(*pose_estimate)) {
    *is_keyframe = false;
    return;
  }

  *is_keyframe = true;

  const transform::Rigid3d rigid3d(
      Eigen::Vector3d(pose_estimate->translation().x(),
                      pose_estimate->translation().y(), 0.),
      Eigen::AngleAxisd(pose_estimate->rotation().angle(),
                        Eigen::Vector3d::UnitZ()));
  std::vector<Eigen::Vector3d> transformed_points;
  transformed_points.reserve(point_cloud.size());
  for (const Eigen::Vector3d& point : point_cloud) {
    transformed_points.emplace_back(rigid3d * point);
  }

  // ndt_aligner_->AddPointCloud(transformed_points);
  icp_aligner_->AddPointCloud(transformed_points);

  // update last keyframe
  last_keyframe_pose_ = *pose_estimate;
}

const std::vector<std::shared_ptr<Submap>> LocalMapBuilder::GetLocalMap()
    const {
  if (active_submaps_ == nullptr) {
    return {};
  }
  return active_submaps_->submaps();
}

}  // namespace solex_robot::navigation::localization_2d