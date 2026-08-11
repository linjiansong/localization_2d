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

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <vector>

#include "include/scan_match/utility.h"

namespace solex_robot::navigation::localization_2d {

enum class GridType { PROBABILITY_GRID = 0, TSDF = 1 };

// Represents a 2D grid of probabilities.
class ProbabilityGrid {
 public:
  ProbabilityGrid(const MapLimits& limits,
                  ValueConversionTables* conversion_tables);
  ProbabilityGrid(const MapLimits& limits,
                  const std::vector<uint16_t>& correspondence_cost_cells,
                  ValueConversionTables* conversion_tables);

  void FinishUpdate();
  
  void GrowLimits(const Eigen::Vector2f& point);

  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability);

  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const;

  std::unique_ptr<ProbabilityGrid> ComputeCroppedGrid() const;

  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16_t>& table);

  GridType GetGridType() const { return GridType::PROBABILITY_GRID; }

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i& cell_index) const;

  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const;

  // Returns the limits of this ProbabilityGrid.
  const MapLimits& map_limits() const { return map_limits_; }

 private:

  int ToFlatIndex(const Eigen::Array2i& cell_index) const;

  bool IsKnown(const Eigen::Array2i& cell_index) const;

  void PrecomputeValueToBoundedFloat();

  inline float ValueToCorrespondenceCost(const uint16_t value) const {
    return value_to_correspondence_cost_[value];
  }

 private:
  MapLimits map_limits_;
  std::vector<uint16_t> correspondence_cost_cells_;
  std::vector<int> update_indices_;

  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;
  ValueConversionTables* conversion_tables_;
  std::vector<float> value_to_correspondence_cost_;
};

}  // namespace solex_robot::navigation::localization_2d