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

#include <Eigen/Core>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "common/base_type.h"
#include "common/rigid_transform.h"
#include "include/scan_match/probability_grid.h"

namespace solex_robot::navigation::localization_2d {

class TranslationDeltaCostFunctor2D {
 public:
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

  static ceres::CostFunction* Create(
      const double scaling_factor, const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;
  const double x_;
  const double y_;
};

class RotationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* Create(const double scaling_factor,
                                     const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
  RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
      delete;

  const double scaling_factor_;
  const double angle_;
};

class GridArrayAdapter {
 public:
  enum { DATA_DIMENSION = 1 };

  explicit GridArrayAdapter(const ProbabilityGrid& probability_grid)
      : probability_grid_(probability_grid) {}

  void GetValue(const int row, const int column, double* const value) const {
    if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
        column >= NumCols() - kPadding) {
      *value = kMaxCorrespondenceCost;
    } else {
      *value = static_cast<double>(probability_grid_.GetProbability(
          Eigen::Array2i(column - kPadding, row - kPadding)));
    }
  }

  int NumRows() const {
    return probability_grid_.map_limits().cell_limits().num_y_cells +
           2 * kPadding;
  }

  int NumCols() const {
    return probability_grid_.map_limits().cell_limits().num_x_cells +
           2 * kPadding;
  }

 private:
  const ProbabilityGrid& probability_grid_;
};

// Computes a cost for matching the 'point_cloud' to the 'probability_grid' with
// a 'pose'. The cost increases with poorer correspondence of the
// probability_grid and the point observation (e.g. points falling into less
// occupied space).
class OccupiedSpaceCostFunction2D {
 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const std::vector<Eigen::Vector3d>& point_cloud,
                              const ProbabilityGrid& probability_grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        probability_grid_(probability_grid) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(probability_grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& map_limits = probability_grid_.map_limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point = point_cloud_[i].cast<T>();
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
          (world[0] - map_limits.origin().x()) / map_limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (map_limits.origin().y() - world[1]) / map_limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

  static ceres::CostFunction* Create(
      const double scaling_factor,
      const std::vector<Eigen::Vector3d>& point_cloud,
      const ProbabilityGrid& probability_grid) {
    return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                           ceres::DYNAMIC /* residuals */,
                                           3 /* pose variables */>(
        new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud,
                                        probability_grid),
        point_cloud.size());
  }

 private:
  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const std::vector<Eigen::Vector3d>& point_cloud_;
  const ProbabilityGrid& probability_grid_;
};

}  // namespace solex_robot::navigation::localization_2d