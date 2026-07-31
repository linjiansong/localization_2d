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

#include <vector>

#include "Eigen/Core"
#include "common/sensor_type.h"
#include "include/scan_match/probability_grid.h"

namespace solex_robot::navigation::localization_2d {
class LaserDataInserter {
 public:
  explicit LaserDataInserter();

  LaserDataInserter(const LaserDataInserter&) = delete;
  LaserDataInserter& operator=(const LaserDataInserter&) = delete;
  void Insert(const sensor::LaserDataPtr& laser_data,
              ProbabilityGrid* probability_grid) const;

 private:
  std::vector<float> PrecomputeValueToBoundedFloat() const;

  std::vector<uint16_t> ComputeLookupTableToApplyCorrespondenceCostOdds(
      float odds) const;

  void GrowAsNeeded(const sensor::LaserDataPtr& laser_data,
                    ProbabilityGrid* const probability_grid) const;

  std::vector<Eigen::Array2i> RayToPixelMask(const Eigen::Array2i& scaled_begin,
                                             const Eigen::Array2i& scaled_end,
                                             int subpixel_scale) const;
  void CastRays(const sensor::LaserDataPtr& laser_data,
                const std::vector<uint16_t>& hit_table,
                const std::vector<uint16_t>& miss_table,
                ProbabilityGrid* probability_grid) const;

 private:
  const std::vector<float> value_to_correspondence_cost_;
  const std::vector<uint16_t> hitting_table_;
  const std::vector<uint16_t> missing_table_;
};

}  // namespace solex_robot::navigation::localization_2d