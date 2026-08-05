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

#include <common/base_type.h>
#include <common/rigid_transform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "common/rigid_transform.h"
#include "include/map_builder/active_map.h"
#include "include/map_builder/icp_aligner.h"
#include "include/map_builder/motion_filter.h"
#include "include/map_builder/ndt_aligner.h"
#include "include/pose_estimator/pose_extrapolator.h"
#include "include/scan_match/ceres_scan_matcher_2d.h"
#include "include/scan_match/real_time_correlative_scan_matcher_2d.h"

namespace solex_robot::navigation::localization_2d {
class LocalMapBuilder {
 public:
  LocalMapBuilder();

  void AddPointCloud(std::vector<Eigen::Vector3d> point_cloud,
                     const transform::Rigid2d& initial_pose,
                     transform::Rigid2d* final_pose, float* score,
                     bool* is_keyframe);

  void AddLaserData(const sensor::LaserDataPtr& laser_data,
                    transform::Rigid2d* pose_estimate, float* score,
                    bool* is_keyframe);

  const std::vector<std::shared_ptr<Submap>> GetLocalMap() const;

 private:
  bool IsKeyframe(const transform::Rigid2d& current_pose);

 private:
  std::unique_ptr<MotionFilter> motion_filter_;
  std::unique_ptr<PoseExtrapolator> pose_extrapolator_;
  std::unique_ptr<NDTAligner> ndt_aligner_;
  std::unique_ptr<ICPAligner> icp_aligner_;
  std::unique_ptr<ActiveSubmap> active_submaps_;
  std::unique_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
  std::unique_ptr<RealTimeCorrelativeScanMatcher2D>
      real_time_correlative_scan_matcher_;
  std::vector<transform::Rigid2d> estimated_poses_;
};

}  // namespace solex_robot::navigation::localization_2d