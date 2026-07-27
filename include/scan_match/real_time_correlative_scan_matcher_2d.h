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

#include <iostream>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "common/rigid_transform.h"
#include "include/scan_match/probability_grid.h"
#include "include/scan_match/utility.h"

namespace solex_robot::navigation::localization_2d {

class RealTimeCorrelativeScanMatcher2D {
 public:
  explicit RealTimeCorrelativeScanMatcher2D(
      const std::shared_ptr<ProbabilityGrid> probability_grid,
      const RealTimeCorrelativeScanMatcherOptions2D& options)
      : probability_grid_(probability_grid), options_(options) {}

  RealTimeCorrelativeScanMatcher2D(const RealTimeCorrelativeScanMatcher2D&) =
      delete;
  RealTimeCorrelativeScanMatcher2D& operator=(
      const RealTimeCorrelativeScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate' then updates 'pose_estimate' with the result and
  // returns the score.
  void Match(const transform::Rigid2d& initial_pose_estimate,
             const std::vector<Eigen::Vector3d>& point_cloud,
             transform::Rigid2d* pose_estimate, float* score) const;

  // Computes the score for each Candidate2D in a collection. The cost is
  // computed as the sum of probabilities or normalized TSD values, different
  // from the Ceres CostFunctions: http://ceres-solver.org/modeling.html
  // Visible for testing.
  void ScoreCandidates(const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* candidates) const;

 private:
  float ComputeCandidateScore(const DiscreteScan2D& discrete_scan,
                              int x_index_offset, int y_index_offset) const;

  std::vector<Candidate2D> GenerateExhaustiveSearchCandidates(
      const SearchParameters& search_parameters) const;

  std::vector<DiscreteScan2D> DiscretizeScans(
      const MapLimits& map_limits,
      const std::vector<std::vector<Eigen::Vector3d>>& scans,
      const Eigen::Translation2f& initial_translation) const;

  std::vector<std::vector<Eigen::Vector3d>> GenerateRotatedScans(
      const std::vector<Eigen::Vector3d>& point_cloud,
      const SearchParameters& search_parameters) const;

 private:
  std::shared_ptr<ProbabilityGrid> probability_grid_;
  const RealTimeCorrelativeScanMatcherOptions2D options_;
};

}  // namespace solex_robot::navigation::localization_2d