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

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "common/rigid_transform.h"

namespace solex_robot::navigation::localization_2d {
namespace sensor {

enum class DataType { kLaserData = 0, kImuData = 1, kOdometryData = 2 };

struct TimedPoint {
  Eigen::Vector3d position;
  double timestamp = 0.0;
};
using TimedPointPtr = std::shared_ptr<TimedPoint>;

class SensorData {
 public:
  SensorData(const double timestamp, const DataType data_type)
      : timestamp_(timestamp), data_type_(data_type) {}

  double timestamp() const { return timestamp_; }
  DataType data_type() const { return data_type_; }

  const transform::Rigid3d& pose() const { return pose_; }
  void set_pose(const transform::Rigid3d& pose) { pose_ = pose; }

 private:
  const double timestamp_;
  const DataType data_type_;
  transform::Rigid3d pose_ = transform::Rigid3d::Identity();
};

class LaserData final : public SensorData {
 public:
  LaserData(const double timestamp)
      : SensorData(timestamp, DataType::kLaserData) {}

  const std::vector<TimedPointPtr>& missing_points() const {
    return missing_points_;
  }
  std::vector<TimedPointPtr>* mutable_missing_points() {
    return &missing_points_;
  }

  const std::vector<TimedPointPtr>& hitting_points() const {
    return hitting_points_;
  }
  std::vector<TimedPointPtr>* mutable_hitting_points() {
    return &hitting_points_;
  }

 private:
  std::vector<TimedPointPtr> missing_points_;
  std::vector<TimedPointPtr> hitting_points_;
};
using LaserDataPtr = std::shared_ptr<LaserData>;

// Imu data
class ImuData final : public SensorData {
 public:
  ImuData(const double timestamp) : SensorData(timestamp, DataType::kImuData) {}

  const Eigen::Vector3d& angular_velocity() const { return angular_velocity_; }
  void set_angular_velocity(const Eigen::Vector3d& angular_velocity) {
    angular_velocity_ = angular_velocity;
  }

  const Eigen::Vector3d& linear_acceleration() const {
    return linear_acceleration_;
  }
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) {
    linear_acceleration_ = linear_acceleration;
  }

 private:
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d linear_acceleration_;
};
using ImuDataPtr = std::shared_ptr<ImuData>;

// Odometry SensorData
class OdometryData final : public SensorData {
 public:
  OdometryData(const double timestamp)
      : SensorData(timestamp, DataType::kOdometryData) {}
};
using OdometryDataPtr = std::shared_ptr<OdometryData>;
}  // namespace sensor
}  // namespace solex_robot::navigation::localization_2d