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

#include "include/scan_match/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "glog/logging.h"

namespace solex_robot::navigation::localization_2d {

namespace {
std::vector<Eigen::Vector3d> TransformPointCloud(
    const std::vector<Eigen::Vector3d>& point_cloud,
    const transform::Rigid3f& transform) {
  std::vector<Eigen::Vector3d> result;
  result.reserve(point_cloud.size());
  for (const Eigen::Vector3d& point : point_cloud) {
    result.emplace_back((transform * point.cast<float>()).cast<double>());
  }
  return result;
}

float ComputeCandidateScore(const ProbabilityGrid& probability_grid,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    candidate_score += probability;
  }
  candidate_score /= static_cast<float>(discrete_scan.size());
  // CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const RealTimeCorrelativeScanMatcherOptions2D& options)
    : options_(options) {}

std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

std::vector<DiscreteScan2D> RealTimeCorrelativeScanMatcher2D::DiscretizeScans(
    const MapLimits& map_limits,
    const std::vector<std::vector<Eigen::Vector3d>>& scans,
    const Eigen::Translation2f& initial_translation) const {
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());
  for (const std::vector<Eigen::Vector3d>& scan : scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    for (const Eigen::Vector3d& point : scan) {
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.cast<float>().head<2>();
      discrete_scans.back().push_back(
          map_limits.GetCellIndex(translated_point));
    }
  }
  return discrete_scans;
}

std::vector<std::vector<Eigen::Vector3d>>
RealTimeCorrelativeScanMatcher2D::GenerateRotatedScans(
    const std::vector<Eigen::Vector3d>& point_cloud,
    const SearchParameters& search_parameters) const {
  std::vector<std::vector<Eigen::Vector3d>> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);

  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    rotated_scans.push_back(TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  return rotated_scans;
}

void RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const std::vector<Eigen::Vector3d>& point_cloud,
    const ProbabilityGrid& probability_grid, transform::Rigid2d* pose_estimate,
    float* score) const {
  CHECK(pose_estimate != nullptr);

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const std::vector<Eigen::Vector3d> rotated_point_cloud = TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  const SearchParameters search_parameters(
      options_.linear_search_window, options_.angular_search_window,
      rotated_point_cloud, probability_grid.map_limits().resolution());

  const std::vector<std::vector<Eigen::Vector3d>> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      probability_grid.map_limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                  &candidates);

  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  *score = best_candidate.score;
}

void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    switch (probability_grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
        candidate.score = ComputeCandidateScore(
            static_cast<const ProbabilityGrid&>(probability_grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        break;
        //   case GridType::TSDF:
        //     candidate.score = ComputeCandidateScore(
        //         static_cast<const TSDF2D&>(probability_grid),
        //         discrete_scans[candidate.scan_index],
        //         candidate.x_index_offset, candidate.y_index_offset);
        //     break;
    }
    candidate.score *=
        std::exp(-std::pow(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight,
                           2.0));
  }
}

}  // namespace solex_robot::navigation::localization_2d