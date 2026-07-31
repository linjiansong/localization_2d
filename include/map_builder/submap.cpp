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

#include "include/map_builder/submap.h"

#include <glog/logging.h>

namespace solex_robot::navigation::localization_2d {

Submap::Submap(const Eigen::Vector2f& origin,
               std::unique_ptr<ProbabilityGrid> probability_grid,
               ValueConversionTables* conversion_tables)
    : local_pose_(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables),
      probability_grid_(std::move(probability_grid)) {}

void Submap::InsertLaserData(const sensor::LaserDataPtr& laser_data,
                             const LaserDataInserter* laser_data_inserter) {
  CHECK(probability_grid_);
  CHECK(!insertion_finished());
  laser_data_inserter->Insert(laser_data, probability_grid_.get());
  ++num_laser_data_;
}

void Submap::Finish() {
  CHECK(probability_grid_);
  CHECK(!insertion_finished());
  probability_grid_ = probability_grid_->ComputeCroppedGrid();
  insertion_finished_ = true;
}

}  // namespace solex_robot::navigation::localization_2d
