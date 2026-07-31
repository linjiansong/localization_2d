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
#include "common/rigid_transform.h"
#include "common/sensor_type.h"
#include "include/scan_match/laser_data_inserter.h"
#include "include/scan_match/probability_grid.h"
#include "include/scan_match/utility.h"

namespace solex_robot::navigation::localization_2d {

inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8_t ProbabilityToLogOddsInteger(const float probability) {
  const int value = std::lround((Logit(probability) - kMinLogOdds) * 254.f /
                                (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

class Submap {
 public:
  Submap(const Eigen::Vector2f& origin,
         std::unique_ptr<ProbabilityGrid> probability_grid,
         ValueConversionTables* conversion_tables);

  transform::Rigid3d local_pose() const { return local_pose_; }

  int num_laser_data() const { return num_laser_data_; }
  void set_num_laser_data(const int num_laser_data) {
    num_laser_data_ = num_laser_data;
  }

  bool insertion_finished() const { return insertion_finished_; }
  void set_insertion_finished(bool insertion_finished) {
    insertion_finished_ = insertion_finished;
  }

  const ProbabilityGrid* probability_grid() const {
    return probability_grid_.get();
  }

  // Insert 'laser_data' into this submap using 'laser_data_inserter'. The
  // submap must not be finished yet.
  void InsertLaserData(const sensor::LaserDataPtr& laser_data,
                       const LaserDataInserter* laser_data_inserter);
  void Finish();

 private:
  const transform::Rigid3d local_pose_;
  int num_laser_data_ = 0;
  bool insertion_finished_ = false;
  std::unique_ptr<ProbabilityGrid> probability_grid_;
  ValueConversionTables* conversion_tables_;
};

}  // namespace solex_robot::navigation::localization_2d
