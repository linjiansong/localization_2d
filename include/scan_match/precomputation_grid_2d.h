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
#include <vector>

#include "Eigen/Core"
#include "common/base_type.h"
#include "common/rigid_transform.h"
#include "include/scan_match/probability_grid.h"

namespace solex_robot::navigation::localization_2d {

class SlidingWindowMaximum {
 public:
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation probability_grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation probability_grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

class PrecomputationGrid2D {
 public:
  PrecomputationGrid2D(const ProbabilityGrid& probability_grid,
                       const CellLimits& limits, int width,
                       std::vector<float>* reusable_intermediate_grid);

  // Returns a value between 0 and 255 to represent probabilities between
  // min_score and max_score.
  int GetValue(const Eigen::Array2i& xy_index) const {
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
  float ToScore(float value) const {
    return min_score_ + value * ((max_score_ - min_score_) / 255.f);
  }

  const CellLimits& wide_limits() const { return wide_limits_; }

  const std::vector<uint8_t>& cells() const { return cells_; }

 private:
  uint8_t ComputeCellValue(float probability) const;

  // Offset of the precomputation probability_grid in relation to the
  // 'probability_grid' including the additional 'width' - 1 cells.
  const Eigen::Array2i offset_;

  // Size of the precomputation probability_grid.
  const CellLimits wide_limits_;

  const float min_score_;
  const float max_score_;

  // Probabilites mapped to 0 to 255.
  std::vector<uint8_t> cells_;
};

}  // namespace solex_robot::navigation::localization_2d