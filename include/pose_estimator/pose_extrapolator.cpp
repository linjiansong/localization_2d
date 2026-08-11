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

#include "include/pose_estimator/pose_extrapolator.h"

#include <algorithm>
#include <iomanip>

#include "glog/logging.h"

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kPoseQueueDuration = 0.001;   // second
constexpr double kGravityTimeConstant = 10.0;  // second
}  // namespace

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const sensor::ImuData& imu_data) {
  auto extrapolator = std::make_unique<PoseExtrapolator>();
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      std::make_unique<ImuTracker>(kGravityTimeConstant, imu_data.timestamp());
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration());
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity());
  extrapolator->imu_tracker_->Advance(imu_data.timestamp());
  extrapolator->AddPose(
      imu_data.timestamp(),
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

double PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return std::numeric_limits<double>::lowest();
  }
  return timed_pose_queue_.back().timestamp;
}

double PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return std::numeric_limits<double>::lowest();
  }
  return extrapolation_imu_tracker_->timestamp();
}

void PoseExtrapolator::AddPose(double timestamp,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    double tracker_start = timestamp;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().timestamp());
    }
    imu_tracker_ =
        std::make_unique<ImuTracker>(kGravityTimeConstant, tracker_start);
  }

  timed_pose_queue_.push_back(TimedPose{timestamp, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].timestamp <= timestamp - kPoseQueueDuration) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  AdvanceImuTracker(timestamp, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = std::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = std::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.timestamp() >= timed_pose_queue_.back().timestamp);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.timestamp() >= timed_pose_queue_.back().timestamp);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }

  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      odometry_data_oldest.timestamp() - odometry_data_newest.timestamp();
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose().inverse() * odometry_data_oldest.pose();
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.timestamp(),
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(double timestamp) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(timestamp, newest_timed_pose.timestamp);

  if (cached_extrapolated_pose_.timestamp != timestamp) {
    const Eigen::Vector3d translation = ExtrapolateTranslation(timestamp) +
                                        newest_timed_pose.pose.translation();
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(timestamp, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{timestamp, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    double timestamp) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(timestamp, &imu_tracker);
  return imu_tracker.orientation();
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.timestamp;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.timestamp;
  const double queue_delta = newest_time - oldest_time;
  if (queue_delta < kPoseQueueDuration) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].timestamp() <= timed_pose_queue_.back().timestamp) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].timestamp() <= timed_pose_queue_.back().timestamp) {
    odometry_data_.pop_front();
  }
}

void PoseExtrapolator::AdvanceImuTracker(double timestamp,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(timestamp, imu_tracker->timestamp());
  if (imu_data_.empty() || timestamp < imu_data_.front().timestamp()) {
    // There is no IMU data until 'timestamp', so we advance the ImuTracker and
    // use the angular velocities from poses and fake gravity to help 2D
    // stability.
    imu_tracker->Advance(timestamp);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->timestamp() < imu_data_.front().timestamp()) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().timestamp());
  }
  auto imu_data_iter = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->timestamp(),
      [](const sensor::ImuData& imu_data, const double timestamp) {
        return imu_data.timestamp() < timestamp;
      });
  while (imu_data_iter != imu_data_.end() &&
         imu_data_iter->timestamp() < timestamp) {
    imu_tracker->Advance(imu_data_iter->timestamp());
    imu_tracker->AddImuLinearAccelerationObservation(
        imu_data_iter->linear_acceleration());
    imu_tracker->AddImuAngularVelocityObservation(
        imu_data_iter->angular_velocity());
    ++imu_data_iter;
  }
  imu_tracker->Advance(timestamp);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    double timestamp, ImuTracker* const imu_tracker) const {
  CHECK_GE(timestamp, imu_tracker->timestamp());
  AdvanceImuTracker(timestamp, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(double timestamp) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta = timestamp - newest_timed_pose.timestamp;
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace solex_robot::navigation::localization_2d
