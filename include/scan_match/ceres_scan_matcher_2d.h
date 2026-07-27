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

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "common/base_type.h"
#include "common/rigid_transform.h"
#include "include/scan_match/probability_grid.h"

namespace solex_robot::navigation::localization_2d {

// Align scans with an existing map using Ceres.
class CeresScanMatcher2D {
 public:
  CeresScanMatcher2D() = default;
  ~CeresScanMatcher2D() = default;

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const transform::Rigid2d& initial_pose_estimate,
             const std::vector<Eigen::Vector3d>& point_cloud,
             const std::shared_ptr<ProbabilityGrid> probability_grid,
             transform::Rigid2d* const pose_estimate, float* const score);
};

}  // namespace solex_robot::navigation::localization_2d