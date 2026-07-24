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

struct TimedPointCloud {
  Eigen::Vector3d position;
  double timestamp = 0.0;
};
using TimedPointCloudPtr = std::shared_ptr<TimedPointCloud>;

class Data {
 public:
  Data(const double timestamp, const DataType data_type)
      : timestamp_(timestamp), data_type_(data_type) {}

  double timestamp() const { return timestamp_; }
  DataType data_type() const { return data_type_; }

 private:
  const double timestamp_;
  const DataType data_type_;
};

class LaserData final : public Data {
 public:
  LaserData(const double timestamp) : Data(timestamp, DataType::kLaserData) {}

  const std::vector<TimedPointCloudPtr>& points() const { return points_; }
  std::vector<TimedPointCloudPtr>* mutable_points() { return &points_; }

 private:
  std::vector<TimedPointCloudPtr> points_;
};

// Imu data
class ImuData final : public Data {
 public:
  ImuData(const double timestamp) : Data(timestamp, DataType::kImuData) {}

  const transform::Rigid3d& pose() const { return pose_; }
  void set_pose(const transform::Rigid3d& pose) { pose_ = pose; }

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
  transform::Rigid3d pose_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d linear_acceleration_;
};

// Odometry Data
class OdometryData final : public Data {
 public:
  OdometryData(const double timestamp)
      : Data(timestamp, DataType::kOdometryData) {}

  const transform::Rigid3d& pose() const { return pose_; }
  void set_pose(const transform::Rigid3d& pose) { pose_ = pose; }

 private:
  transform::Rigid3d pose_;
};

}  // namespace sensor
}  // namespace solex_robot::navigation::localization_2d