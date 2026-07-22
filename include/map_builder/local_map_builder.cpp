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

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kRadianToDegree = 180.0 / M_PI;
constexpr double kKeyframeMinDistance = 0.2;  // meter
constexpr double kKeyframeMinAngle = 5.0;     // degree
}  // namespace

bool LocalMapBuilder::IsKeyframe(const transform::Rigid2d& current_pose) {
  const transform::Rigid2d delta_pose =
      last_keyfframe_pose_.inverse() * current_pose;
  const double distance = delta_pose.translation().norm();  // meter
  const double angle =
      delta_pose.rotation().angle() * kRadianToDegree;  // degree

  return distance > kKeyframeMinDistance || angle > kKeyframeMinAngle;
}

void LocalMapBuilder::AddPointCloud(std::vector<Eigen::Vector3d> point_cloud,
                                    const transform::Rigid2d& initial_pose,
                                    transform::Rigid2d* pose_estimate,
                                    float* score, bool* is_keyframe) {
  if (estimated_poses_.empty()) {
    last_keyfframe_pose_ = initial_pose;
    // ndt_aligner_ = std::make_unique<NDTAligner>();
    // ndt_aligner_->AddPointCloud(point_cloud);

    icp_aligner_ = std::make_unique<ICPAligner>();
    icp_aligner_->AddPointCloud(point_cloud);
    estimated_poses_.emplace_back(initial_pose);
    map_points_.insert(map_points_.begin(), point_cloud.begin(),
                       point_cloud.end());

    *pose_estimate = initial_pose;
    *score = 1.0;
    *is_keyframe = true;
    return;
  }

  // 此时local map位于匹配器内部，直接配准即可
  // ndt_aligner_->Align(point_cloud, initial_pose, pose_estimate, score);
  icp_aligner_->Align(point_cloud, initial_pose, pose_estimate, score);
  estimated_poses_.emplace_back(*pose_estimate);
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
  last_keyfframe_pose_ = *pose_estimate;
  map_points_.insert(map_points_.begin(), transformed_points.begin(),
                     transformed_points.end());
}

}  // namespace solex_robot::navigation::localization_2d