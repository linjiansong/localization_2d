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
#include "include/map_builder/icp_aligner.h"
#include "include/map_builder/ndt_aligner.h"

namespace solex_robot::navigation::localization_2d {
class LocalMapBuilder {
 public:
  LocalMapBuilder() = default;

  void AddPointCloud(std::vector<Eigen::Vector3d> point_cloud,
                     const transform::Rigid2d& initial_pose,
                     transform::Rigid2d* final_pose, float* score,
                     bool* is_keyframe);

 private:
  bool IsKeyframe(const transform::Rigid2d& current_pose);

 private:
  transform::Rigid2d last_keyframe_pose_;
  std::unique_ptr<NDTAligner> ndt_aligner_;
  std::unique_ptr<ICPAligner> icp_aligner_;
  std::vector<transform::Rigid2d> estimated_poses_;
};

}  // namespace solex_robot::navigation::localization_2d