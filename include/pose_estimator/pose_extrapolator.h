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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <memory>

#include "common/rigid_transform.h"
#include "common/sensor_type.h"
#include "include/pose_estimator/imu_tracker.h"

namespace solex_robot::navigation::localization_2d {

struct TimedPose {
  double timestamp = std::numeric_limits<double>::lowest();
  transform::Rigid3d pose = transform::Rigid3d::Identity();
};

class PoseExtrapolator {
 public:
  PoseExtrapolator() = default;

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      double pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  double GetLastPoseTime() const;
  double GetLastExtrapolatedTime() const;

  void AddPose(double time, const transform::Rigid3d& pose);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(double time);

  // ExtrapolationResult ExtrapolatePosesWithGravity(
  //     const std::vector<double>& times);

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(double time);

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(double time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(double time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(double time);

 private:
  const double pose_queue_duration_ = 1.e-3;
  const double gravity_time_constant_ = 1.0;

  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  std::deque<sensor::ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace solex_robot::navigation::localization_2d