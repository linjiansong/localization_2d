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
//  or all damages (including recovery of attorneys' fees) which may be //`
//  suffered and or incurred as a result of your infringement.                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "common/base_type.h"
#include "include/map_builder/local_map_builder.h"
#include "include/pose_estimator/pose_extrapolator.h"
#include "include/scan_match/ceres_scan_matcher_2d.h"
#include "include/scan_match/fast_correlative_scan_matcher_2d.h"
#include "include/scan_match/probability_grid.h"
#include "include/scan_match/real_time_correlative_scan_matcher_2d.h"

namespace solex_robot::navigation::localization_2d {

class Localization {
 public:
  Localization() = default;
  ~Localization() = default;

  void AddImuData();
  void AddOdometryData();
  void AddPointCloud(const PointCloud& point_cloud);
  void AddGridMap(std::shared_ptr<ProbabilityGrid> probability_grid);
  void AddInitialPose(const Eigen::Matrix4d& initial_pose);
  void Init();

  const std::deque<KeyframePtr>& keyframe_buffer() const {
    return keyframe_buffer_;
  }
  const Eigen::Matrix4d& curr_pose() const { return curr_pose_; }

 private:
  void DistordPointCloud(const PointCloud& point_cloud);

  std::pair<Eigen::Matrix4d, float> MatchGlobalMap(
      const std::vector<Eigen::Vector3d>& points,
      const Eigen::Matrix4d& initial_pose);

  void GlobalLocalization(const PointCloud& point_cloud);

  void Relocalization(const PointCloud& point_cloud);

  void Track(const PointCloud& point_cloud);

  void GlobalOptimize();

  void GlobalLocalization();

 private:
  std::shared_ptr<ProbabilityGrid> probability_grid_;
  std::deque<KeyframePtr> keyframe_buffer_;
  Eigen::Matrix4d initial_pose_ = Eigen::Matrix4d::Identity();
  std::mutex keyframe_buffer_mutex_;
  int keyframe_interval_ = 0;

  std::shared_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
  std::shared_ptr<FastCorrelativeScanMatcher2D> fast_correlative_scan_matcher_;
  std::shared_ptr<RealTimeCorrelativeScanMatcher2D>
      real_time_correlative_scan_matcher_;
  std::shared_ptr<LocalMapBuilder> local_map_builder_;
  std::shared_ptr<PoseExtrapolator> pose_extrapolator_;

  LocalizationStatus localization_status_ = LocalizationStatus::kUnknown;
  transform::Rigid2d last_local_pose_ = transform::Rigid2d::Identity();
  Eigen::Matrix4d curr_pose_ = Eigen::Matrix4d::Identity();
  int count_ = 0;
  // Todo: PoseExtractor
};
}  // namespace solex_robot::navigation::localization_2d
