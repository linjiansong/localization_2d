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
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "common/sensor_type.h"
#include "include/map_builder/submap.h"
#include "include/scan_match/laser_data_inserter.h"
#include "include/scan_match/utility.h"

namespace solex_robot::navigation::localization_2d {
class ActiveSubmap {
 public:
  explicit ActiveSubmap()
      : laser_data_inserter_(std::make_unique<LaserDataInserter>()) {}

  ActiveSubmap(const ActiveSubmap&) = delete;
  ActiveSubmap& operator=(const ActiveSubmap&) = delete;

  // Inserts 'laser_data' into the Submap collection.
  void InsertLaserData(const sensor::LaserDataPtr& laser_data);

  const std::vector<std::shared_ptr<Submap>>& submaps() const {
    return submaps_;
  }

 private:
  std::unique_ptr<ProbabilityGrid> CreateGrid(const Eigen::Vector2f& origin);
  void FinishSubmap();
  void AddSubmap(const Eigen::Vector2f& origin);

 private:
  const std::unique_ptr<LaserDataInserter> laser_data_inserter_;
  std::vector<std::shared_ptr<Submap>> submaps_;
  ValueConversionTables conversion_tables_;
};

}  // namespace solex_robot::navigation::localization_2d
