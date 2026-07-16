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

#include "include/scan_match/precomputation_grid_2d.h"

namespace solex_robot::navigation::localization_2d {

PrecomputationGrid2D::PrecomputationGrid2D(
    const ProbabilityGrid& probability_grid, const CellLimits& limits,
    const int width, std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - kMaxCorrespondenceCost),
      max_score_(1.f - kMaxCorrespondenceCost),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  LOG(INFO) << "offset_ = " << offset_.x() << ", " << offset_.y();
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.

  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        1.f -
        std::abs(probability_grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));
    // LOG(INFO) << "max value = "
    //           << 1.f - std::abs(probability_grid.GetCorrespondenceCost(
    //                        Eigen::Array2i(0, y)));
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(1.f -
                                std::abs(probability_grid.GetCorrespondenceCost(
                                    Eigen::Array2i(x + width, y))));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(probability_grid.GetCorrespondenceCost(
                    Eigen::Array2i(x, y))));
      current_values.AddValue(1.f -
                              std::abs(probability_grid.GetCorrespondenceCost(
                                  Eigen::Array2i(x + width, y))));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(probability_grid.GetCorrespondenceCost(
                    Eigen::Array2i(x, y))));
    }
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }

  // for (const int value : cells_) {
  //   LOG(INFO) << "value = " << value;
  // }
}

int PrecomputationGrid2D::GetValue(const Eigen::Array2i& xy_index) const {
  const Eigen::Array2i local_xy_index = xy_index - offset_;
  // The static_cast<unsigned> is for performance to check with 2 comparisons
  // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
  // local_xy_index.x() >= wide_limits_.num_x_cells ||
  // local_xy_index.y() >= wide_limits_.num_y_cells
  // instead of using 4 comparisons.
  if (static_cast<unsigned>(local_xy_index.x()) >=
          static_cast<unsigned>(wide_limits_.num_x_cells) ||
      static_cast<unsigned>(local_xy_index.y()) >=
          static_cast<unsigned>(wide_limits_.num_y_cells)) {
    return 0;
  }
  const int stride = wide_limits_.num_x_cells;
  return cells_[local_xy_index.x() + local_xy_index.y() * stride];
}

// Maps values from [0, 255] to [min_score, max_score].
float PrecomputationGrid2D::ToScore(float value) const {
  return min_score_ + value * ((max_score_ - min_score_) / 255.f);
}

uint8_t PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = std::lround((probability - min_score_) *
                                     (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

}  // namespace solex_robot::navigation::localization_2d
