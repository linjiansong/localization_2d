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

#include "include/scan_match/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <opencv2/opencv.hpp>

#include "Eigen/Geometry"
#include "glog/logging.h"
#include "include/scan_match/probability_grid.h"

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
}  // namespace

FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const ProbabilityGrid& probability_grid,
    const FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(probability_grid.map_limits()),
      precomputation_grid_stack_(std::make_unique<PrecomputationGridStack2D>(
          probability_grid, options)) {
  for (int depth = 0; depth < options.branch_and_bound_depth; ++depth) {
    const PrecomputationGrid2D& precomputation_grid =
        precomputation_grid_stack_->Get(depth);
    const CellLimits& wide_limits = precomputation_grid.wide_limits();
    const std::vector<uint8_t>& cells = precomputation_grid.cells();

    cv::Mat image(wide_limits.num_y_cells, wide_limits.num_x_cells, CV_8UC1);
    for (int y = 0; y < wide_limits.num_y_cells; ++y) {
      for (int x = 0; x < wide_limits.num_x_cells; ++x) {
        const int flat_index = y * wide_limits.num_x_cells + x;
        const uint8_t value = cells[flat_index];
        image.at<uchar>(y, x) = 255 - value;
      }
    }

    // const std::string image_path =
    //     "/home/linjs/图片/multimap_" + std::to_string(depth) + ".png";
    // LOG(INFO) << "image_path = " << image_path;
    // cv::imwrite(image_path, image);
  }
}

std::vector<DiscreteScan2D> FastCorrelativeScanMatcher2D::DiscretizeScans(
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
FastCorrelativeScanMatcher2D::GenerateRotatedScans(
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

// match_type = Initial
bool FastCorrelativeScanMatcher2D::MatchLocalSubmap(
    const transform::Rigid2d& initial_pose_estimate,
    const std::vector<Eigen::Vector3d>& point_cloud, const float min_score,
    float* score, transform::Rigid2d* pose_estimate) const {
  // LOG(INFO) << "initial_localization_linear_search_window = " <<
  // options_.initial_localization_linear_search_window(); LOG(INFO) <<
  // "initial_localization_angular_search_window = "  <<
  // options_.initial_localization_angular_search_window();

  const SearchParameters search_parameters(options_.linear_search_window,
                                           options_.angular_search_window,
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

// match_type = Global
bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const std::vector<Eigen::Vector3d>& point_cloud, float min_score,
    float* score, transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());

  const Eigen::Vector2d center =
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells);

  return MatchWithSearchParameters(
      search_parameters, transform::Rigid2d::Translation(center), point_cloud,
      min_score, score, pose_estimate);
}

bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const std::vector<Eigen::Vector3d>& point_cloud, float min_score,
    float* score, transform::Rigid2d* pose_estimate) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const std::vector<Eigen::Vector3d> rotated_point_cloud = TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));

  const std::vector<std::vector<Eigen::Vector3d>> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);

  *score = best_candidate.score;

  if (best_candidate.score > min_score) {
    // *score = best_candidate.score;
    // LOG(INFO) << "best_candidate.score = " << best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));

    {
      // Todo:
      transform::Rigid3f rigid3f(
          Eigen::Vector3f(pose_estimate->translation().x(),
                          pose_estimate->translation().y(), 0.),
          Eigen::AngleAxisf(pose_estimate->rotation().angle(),
                            Eigen::Vector3f::UnitZ()));

      const std::vector<Eigen::Vector3d> rotated_point_cloud =
          TransformPointCloud(point_cloud, rigid3f);

      const PrecomputationGrid2D& precomputation_grid =
          precomputation_grid_stack_->Get(0);

      float score = 0.0;
      for (const Eigen::Vector3d point : rotated_point_cloud) {
        const Eigen::Array2i index =
            limits_.GetCellIndex(point.cast<float>().head<2>());
        score += precomputation_grid.GetValue(index);
      }

      // LOG(INFO) << "score = "
      //           << precomputation_grid.ToScore(score /
      //                                          rotated_point_cloud.size());
    }

    return true;
  }

  return false;
}

std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);

  // LOG(INFO) << "discrete_scans = " << discrete_scans.size();
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);

  {
    const auto precomputation_grid = precomputation_grid_stack_->Get(
        precomputation_grid_stack_->max_depth());
    const CellLimits& wide_limits = precomputation_grid.wide_limits();
    const std::vector<uint8_t>& cells = precomputation_grid.cells();

    cv::Mat image(wide_limits.num_y_cells, wide_limits.num_x_cells, CV_8UC3);
    for (int y = 0; y < wide_limits.num_y_cells; ++y) {
      for (int x = 0; x < wide_limits.num_x_cells; ++x) {
        const int flat_index = y * wide_limits.num_x_cells + x;
        const uint8_t value = 255 - cells[flat_index];
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
      }
    }

    const Eigen::Vector2d center =
        limits_.max() - 0.5 * limits_.resolution() *
                            Eigen::Vector2d(limits_.cell_limits().num_x_cells,
                                            limits_.cell_limits().num_y_cells);
    LOG(INFO) << "center = " << center.x() << ", " << center.y();

    const Eigen::Array2i origin_index =
        limits_.GetCellIndex(center.cast<float>());
    LOG(INFO) << "origin_index = " << origin_index.x() << ", "
              << origin_index.y();
    for (const Candidate2D& candidate : lowest_resolution_candidates) {
      Eigen::Array2i index = origin_index;
      index.x() += candidate.x_index_offset;
      index.y() += candidate.y_index_offset;
      // LOG(INFO) << "index = " << index.x() << ", " << index.y();
      if (limits_.Contains(index)) {
        image.at<cv::Vec3b>(index.y(), index.x()) = cv::Vec3b(0, 255, 0);
      } else {
        // LOG(INFO) << "index = " << index.x() << ", " << index.y();
        continue;
      }
    }

    cv::imwrite("/home/linjs/图片/lowest_resolution_candidates.png", image);
  }

  // LOG(INFO) << "lowest_resolution_candidates = "
  //           << lowest_resolution_candidates.size();
  return lowest_resolution_candidates;
}

std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }

  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }

    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  for (const Candidate2D& candidate : candidates) {
    if (candidate.score <= min_score) {
      break;
    }
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace solex_robot::navigation::localization_2d