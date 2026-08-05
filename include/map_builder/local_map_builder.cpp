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
#include <opencv2/opencv.hpp>

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kDegreeToRadian = M_PI / 180.0;
}  // namespace

LocalMapBuilder::LocalMapBuilder()
    : motion_filter_(std::make_unique<MotionFilter>()),
      pose_extrapolator_(std::make_unique<PoseExtrapolator>()),
      ndt_aligner_(std::make_unique<NDTAligner>()),
      icp_aligner_(std::make_unique<ICPAligner>()),
      active_submaps_(std::make_unique<ActiveSubmap>()),
      ceres_scan_matcher_(std::make_unique<CeresScanMatcher2D>()) {
  RealTimeCorrelativeScanMatcherOptions2D real_time_options;
  real_time_options.linear_search_window = 0.1;
  real_time_options.angular_search_window = 5.0 * kDegreeToRadian;
  real_time_options.translation_delta_cost_weight = 10;
  real_time_options.rotation_delta_cost_weight = 0.1;
  real_time_correlative_scan_matcher_ =
      std::make_unique<RealTimeCorrelativeScanMatcher2D>(real_time_options);
}

void LocalMapBuilder::AddLaserData(const sensor::LaserDataPtr& laser_data,
                                   transform::Rigid2d* pose_estimate,
                                   float* score, bool* is_keyframe) {
  if (active_submaps_->submaps().empty()) {
    pose_extrapolator_->AddPose(laser_data->timestamp(),
                                transform::Rigid3d::Identity());
    laser_data->set_pose(transform::Rigid3d::Identity());
    active_submaps_->InsertLaserData(laser_data);
    *pose_estimate = transform::Rigid2d::Identity();
    *score = 1.0;
    *is_keyframe = true;
    return;
  }

  const ProbabilityGrid* probability_grid =
      active_submaps_->submaps().front()->probability_grid();
  if (active_submaps_->submaps().front()->num_laser_data() == 1) {
    const auto map_limits = probability_grid->map_limits();
    const int width = map_limits.cell_limits().num_x_cells;
    const int height = map_limits.cell_limits().num_y_cells;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const Eigen::Array2i index(x, y);
        const double probability = probability_grid->GetProbability(index);
        const uint8_t value = 255 * (1.0 - probability);
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
      }
    }

    cv::imwrite("/home/linjs/图片/global_match/active_map.png", image);
    // LOG(INFO) << "resolution = " << limit.resolution()
    //           << ", width = " << limit.cell_limits().num_x_cells
    //           << ", height = " << limit.cell_limits().num_y_cells;

    // for (int y = 0; y < limit.cell_limits().num_y_cells; ++y) {
    //   std::stringstream ss;
    //   for (int x = 0; x < limit.cell_limits().num_x_cells; ++x) {
    //     const Eigen::Array2i index(x, y);
    //     const double probability = probability_grid->GetProbability(index);
    //     const int value = 255 * probability;
    //     ss << value << ", ";
    //   }
    //   LOG(INFO) << ss.str();
    // }

    getchar();
  }

  std::vector<Eigen::Vector3d> point_cloud;
  point_cloud.reserve(laser_data->hitting_points().size());
  for (const auto& hitting_point : laser_data->hitting_points()) {
    point_cloud.emplace_back(hitting_point->position);
  }

  // real time csm match
  const transform::Rigid2d initial_pose = transform::Project2D(
      pose_extrapolator_->ExtrapolatePose(laser_data->timestamp()));

  transform::Rigid2d real_time_pose_estimate = initial_pose;
  // float real_time_score = 0.0;
  // real_time_correlative_scan_matcher_->Match(
  //     initial_pose, point_cloud, *probability_grid, &real_time_pose_estimate,
  //     &real_time_score);
  // LOG(INFO) << "real_time_score = " << real_time_score;

  transform::Rigid2d ceres_pose_estimate;
  float ceres_match_score;
  ceres_scan_matcher_->Match(real_time_pose_estimate, point_cloud,
                             *probability_grid, &ceres_pose_estimate,
                             &ceres_match_score);
  LOG(INFO) << "ceres_match_score = " << ceres_match_score;
  pose_extrapolator_->AddPose(laser_data->timestamp(),
                              transform::Embed3D(ceres_pose_estimate));

  *pose_estimate = ceres_pose_estimate;
  *score = ceres_match_score;
  if (motion_filter_->IsSimilar(laser_data->timestamp(),
                                transform::Embed3D(*pose_estimate))) {
    *is_keyframe = false;
    return;
  }

  *is_keyframe = true;
  laser_data->set_pose(transform::Embed3D(*pose_estimate));
  active_submaps_->InsertLaserData(laser_data);
  // LOG(INFO) << "pose_estimate = [" << pose_estimate->translation().x() << ",
  // "
  //           << pose_estimate->translation().x() << ", "
  //           << pose_estimate->rotation().angle() << "], score = " << *score;
}

void LocalMapBuilder::AddPointCloud(std::vector<Eigen::Vector3d> point_cloud,
                                    const transform::Rigid2d& initial_pose,
                                    transform::Rigid2d* pose_estimate,
                                    float* score, bool* is_keyframe) {
  if (estimated_poses_.empty()) {
    // ndt_aligner_->AddPointCloud(point_cloud);
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
  if (motion_filter_->IsSimilar(0.0,
                                transform::Embed3D(*pose_estimate))) {  // Todo
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
}

const std::vector<std::shared_ptr<Submap>> LocalMapBuilder::GetLocalMap()
    const {
  if (active_submaps_ == nullptr) {
    return {};
  }
  return active_submaps_->submaps();
}

}  // namespace solex_robot::navigation::localization_2d