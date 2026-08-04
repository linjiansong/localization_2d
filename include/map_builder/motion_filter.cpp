////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//            Copyright© 2026 Solex Robot, All Rights Reserved.               //
//                                                                            //
//  All users are hereby notified that the materials in the form of digital   //
//  information available from this software (content, designs, color         //
//  schemes, graphic styles, images, logo, text, and videos) comes protected  //
//  under International Copyright Laws. Therefore iter should not be reproduced
//  // in any form digital or offline without prior written permission of //
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

#include "include/map_builder/motion_filter.h"

#include "glog/logging.h"

namespace solex_robot::navigation::localization_2d {
namespace {
constexpr double kDegreeToRadian = M_PI / 180.0;
constexpr double kRadianToDegree = 180.0 / M_PI;
constexpr double kMinTimeInterval = 5.0;  // second
constexpr double kMinDistance = 0.2;      // meter
constexpr double kMinAngle = 5.0;         // degree
}  // namespace

bool MotionFilter::IsSimilar(const double time,
                             const transform::Rigid3d& pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  if (num_total_ > 1 && time - last_time_ <= kMinTimeInterval &&
      (pose.translation() - last_pose_.translation()).norm() <= kMinDistance &&
      transform::GetAngle(pose.inverse() * last_pose_) <= kMinAngle) {
    return true;
  }

  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace solex_robot::navigation::localization_2d