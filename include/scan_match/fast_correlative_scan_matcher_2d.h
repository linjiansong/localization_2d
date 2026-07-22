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

#include <deque>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "common/base_type.h"
#include "common/rigid_transform.h"
#include "include/scan_match/precomputation_grid_2d.h"
#include "include/scan_match/probability_grid.h"
#include "include/scan_match/utility.h"

namespace solex_robot::navigation::localization_2d {

class PrecomputationGridStack2D {
 public:
  PrecomputationGridStack2D(
      const ProbabilityGrid& probability_grid,
      const FastCorrelativeScanMatcherOptions2D& options) {
    CHECK_GE(options.branch_and_bound_depth, 1);
    const int max_width = 1 << (options.branch_and_bound_depth - 1);
    precomputation_grids_.reserve(options.branch_and_bound_depth);
    std::vector<float> reusable_intermediate_grid;
    const CellLimits limits = probability_grid.map_limits().cell_limits();
    reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                       limits.num_y_cells);
    for (int i = 0; i != options.branch_and_bound_depth; ++i) {
      const int width = 1 << i;
      precomputation_grids_.emplace_back(probability_grid, limits, width,
                                         &reusable_intermediate_grid);
    }
  }

  const PrecomputationGrid2D& Get(int index) {
    return precomputation_grids_[index];
  }

  int max_depth() const { return precomputation_grids_.size() - 1; }

 private:
  std::vector<PrecomputationGrid2D> precomputation_grids_;
};

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class FastCorrelativeScanMatcher2D {
 public:
  FastCorrelativeScanMatcher2D(
      const ProbabilityGrid& probability_grid,
      const FastCorrelativeScanMatcherOptions2D& options);
  ~FastCorrelativeScanMatcher2D() = default;

  FastCorrelativeScanMatcher2D(const FastCorrelativeScanMatcher2D&) = delete;
  FastCorrelativeScanMatcher2D& operator=(const FastCorrelativeScanMatcher2D&) =
      delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result.
  // bool Match(const transform::Rigid2d& initial_pose_estimate,
  //            const std::vector<Eigen::Vector3d>& point_cloud, float
  //            min_score, float* score, transform::Rigid2d* pose_estimate)
  //            const;

  std::vector<DiscreteScan2D> DiscretizeScans(
      const MapLimits& map_limits,
      const std::vector<std::vector<Eigen::Vector3d>>& scans,
      const Eigen::Translation2f& initial_translation) const;

  std::vector<std::vector<Eigen::Vector3d>> GenerateRotatedScans(
      const std::vector<Eigen::Vector3d>& point_cloud,
      const SearchParameters& search_parameters) const;

  bool MatchLocalSubmap(const transform::Rigid2d& initial_pose_estimate,
                        const std::vector<Eigen::Vector3d>& point_cloud,
                        float min_score, float* score,
                        transform::Rigid2d* pose_estimate) const;

  // Aligns 'point_cloud' within the full 'probability_grid', i.e., not
  // restricted to the configured search window. If a score above 'min_score'
  // (excluding equality) is possible, true is returned, and 'score' and
  // 'pose_estimate' are updated with the result.
  bool MatchFullSubmap(const std::vector<Eigen::Vector3d>& point_cloud,
                       float min_score, float* score,
                       transform::Rigid2d* pose_estimate) const;

 private:
  // The actual implementation of the scan matcher, called by Match() and
  // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
  // 'search_parameters'.
  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const std::vector<Eigen::Vector3d>& point_cloud, float min_score,
      float* score, transform::Rigid2d* pose_estimate) const;
  std::vector<Candidate2D> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan2D>& discrete_scans,
      const SearchParameters& search_parameters) const;
  std::vector<Candidate2D> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;
  void ScoreCandidates(const PrecomputationGrid2D& precomputation_grid,
                       const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* const candidates) const;
  Candidate2D BranchAndBound(const std::vector<DiscreteScan2D>& discrete_scans,
                             const SearchParameters& search_parameters,
                             const std::vector<Candidate2D>& candidates,
                             int candidate_depth, float min_score) const;

  const FastCorrelativeScanMatcherOptions2D options_;
  MapLimits limits_;
  std::unique_ptr<PrecomputationGridStack2D> precomputation_grid_stack_;
};

}  // namespace solex_robot::navigation::localization_2d