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

#pragma once

#include "common/rigid_transform.h"
#include "Eigen/Geometry"

namespace solex_robot::navigation::localization_2d {

class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, double timestamp);

  // Advances to the given 'timestamp' and updates the orientation to reflect this.
  void Advance(double timestamp);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);

  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current timestamp.
  double timestamp() const { return timestamp_; }

  // Query the current orientation estimate.
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;
  double timestamp_;

  double last_linear_acceleration_time_ = std::numeric_limits<double>::lowest();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gravity_vector_ = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d imu_angular_velocity_ = Eigen::Vector3d::Zero();
};

}  // namespace solex_robot::navigation::localization_2d