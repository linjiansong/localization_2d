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

#include "include/map_builder/active_map.h"

#include <glog/logging.h>

namespace solex_robot::navigation::localization_2d {
namespace {
constexpr int kInitialSubmapSize = 100;
constexpr int kNumberLaserData = 50;
constexpr float kResolution = 0.05;
}  // namespace

std::unique_ptr<ProbabilityGrid> ActiveSubmap::CreateGrid(
    const Eigen::Vector2f& origin) {
  return std::make_unique<ProbabilityGrid>(
      MapLimits(kResolution, origin.cast<double>(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
      &conversion_tables_);
}

void ActiveSubmap::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }

  submaps_.push_back(std::make_unique<Submap>(origin, CreateGrid(origin),
                                              &conversion_tables_));
}

void ActiveSubmap::InsertLaserData(const sensor::LaserDataPtr& laser_data) {
  if (submaps_.empty() ||
      submaps_.back()->num_laser_data() == kNumberLaserData) {
    AddSubmap(laser_data->pose().translation().cast<float>().head<2>());
  }

  for (auto& submap : submaps_) {
    submap->InsertLaserData(laser_data, laser_data_inserter_.get());
  }

  if (submaps_.front()->num_laser_data() == 2 * kNumberLaserData) {
    submaps_.front()->Finish();
  }
}

}  // namespace solex_robot::navigation::localization_2d
