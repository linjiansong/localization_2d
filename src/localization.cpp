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

#include "src/localization.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <execution>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <vector>

#include "common/ceres_helper.h"
#include "include/scan_match/utility.h"

namespace solex_robot::navigation::localization_2d {
namespace {
constexpr double kMillisecondToSecond = 1.e-3;
constexpr double kRadianToDegree = 180.0 / M_PI;
constexpr double kDegreeToRadian = M_PI / 180.0;
constexpr double kKeyframeMinDistance = 0.2;  // meter
constexpr double kKeyframeMinAngle = 5.0;     // degree
constexpr double kICPMinDistance = 1e-4;      // meter
constexpr double kICPMinAngle = 0.01;         // degree

constexpr double kOccupiedSpaceWeight = 10.0;
constexpr double kTranslationWeight = 10.0;
constexpr double kRotationWeight = 40.0;

constexpr int kMaxKeyframeBufferLength = 1;
constexpr int kOptimizeKeyframeInterval = 5;

constexpr int kMaxIterationsNum = 50;
constexpr int kThreadNum = 8;

constexpr std::array<double, 6> kInterFrameWeight = {1.e3, 1.e3, 1.e3,
                                                     1.e2, 1.e2, 1.e2};
constexpr std::array<double, 6> kFixedPoseWeiht = {1.e3, 1.e3, 1.e3,
                                                   1.e2, 1.e2, 1.e2};

constexpr int kFixedPoseHuberLoss = 20.0;

constexpr int kMinGlobalLocalizationScore = 0.5;

Eigen::Matrix4d ToMatrix4d(const transform::Rigid2d& rigid_pose) {
  Eigen::Matrix4d eigen_pose = Eigen::Matrix4d::Identity();
  eigen_pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(rigid_pose.rotation().angle(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  eigen_pose(0, 3) = rigid_pose.translation().x();
  eigen_pose(1, 3) = rigid_pose.translation().y();
  return eigen_pose;
}

transform::Rigid2d ToRigid2d(const Eigen::Matrix4d& eigen_pose) {
  const Eigen::Vector2d translation(eigen_pose(0, 3), eigen_pose(1, 3));
  const double rotation_angle = std::atan2(eigen_pose(1, 0), eigen_pose(0, 0));
  return transform::Rigid2d(translation, rotation_angle);
}

Eigen::Vector3d TransformPoint(const Eigen::Matrix4d& transform,
                               const Eigen::Vector3d& point) {
  return transform.block<3, 3>(0, 0) * point + transform.block<3, 1>(0, 3);
}

std::vector<Eigen::Vector3d> ConvertPoint(const sensor::LaserData& laser_data) {
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.reserve(laser_data.points().size());
  for (const sensor::TimedPointCloudPtr& timed_point : laser_data.points()) {
    eigen_points.emplace_back(timed_point->position);
  }

  return eigen_points;
}
}  // namespace

std::pair<Eigen::Matrix4d, float> Localization::MatchGlobalMap(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Matrix4d& initial_pose) {
  // LOG(INFO) << "Match global map";
  transform::Rigid2d real_time_pose_estimate;
  float real_time_score = 0.0;
  real_time_correlative_scan_matcher_->Match(
      ToRigid2d(initial_pose), points, *probability_grid_,
      &real_time_pose_estimate, &real_time_score);
  // LOG(INFO) << "++++++++++++++++++++++++++ real time = " << real_time_score;

  float ceres_match_score = real_time_score;
  transform::Rigid2d ceres_pose_estimate = real_time_pose_estimate;
  // float ceres_match_score = 0.0;
  // transform::Rigid2d ceres_pose_estimate;
  // ceres_scan_matcher_->Match(real_time_pose_estimate, points,
  // probability_grid_,
  //                            &ceres_pose_estimate, &ceres_match_score);
  // LOG(INFO) << "-------------------------- ceres = " << ceres_match_score;

  // {
  //   const MapLimits& map_limits = probability_grid_->map_limits();
  //   const int width = map_limits.cell_limits().num_x_cells;
  //   const int height = map_limits.cell_limits().num_y_cells;

  //   cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
  //   for (int y = 0; y < height; ++y) {
  //     for (int x = 0; x < width; ++x) {
  //       const Eigen::Array2i index(x, y);
  //       const double probability = probability_grid_->GetProbability(index);
  //       const uint8_t value = 255 * (1.0 - probability);
  //       image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
  //     }
  //   }

  //   for (const Eigen::Vector3d& point : points) {
  //     const Eigen::Vector2d new_point = ceres_pose_estimate * point.head(2);
  //     const Eigen::Array2i index =
  //         map_limits.GetCellIndex(new_point.cast<float>());
  //     image.at<cv::Vec3b>(index.y(), index.x()) = cv::Vec3b(0, 0, 255);
  //   }

  //   cv::imwrite("/home/linjs/图片/global_match/match_" +
  //                   std::to_string(count_++) + ".png",
  //               image);
  // }

  // return std::make_pair(global_pose, match_score);
  return std::make_pair(ToMatrix4d(ceres_pose_estimate), ceres_match_score);
}

void Localization::GlobalOptimize() {
  const auto problem = std::make_shared<ceres::Problem>();

  // Add parameter
  std::unordered_map<KeyframePtr, PosePtr> keyframe_pose6ds;
  for (const auto& keyframe : keyframe_buffer_) {
    PosePtr p6d_ptr(new double[6], std::default_delete<double[]>());
    Eigen::Map<Eigen::Matrix<double, 6, 1>> p6d(p6d_ptr.get());
    p6d.topRows(3) = keyframe->state.rotation.cast<double>();
    p6d.bottomRows(3) = keyframe->state.position.cast<double>();
    keyframe_pose6ds.insert(std::make_pair(keyframe, p6d_ptr));
    problem->AddParameterBlock(p6d_ptr.get(), 6);
    problem->SetParameterization(p6d_ptr.get(), SE3SeprateLieAlgo::Create());
  }

  // Add inter-frame constraint
  for (std::size_t curr_idx = 1; curr_idx < keyframe_buffer_.size();
       ++curr_idx) {
    const auto& curr_keyframe = keyframe_buffer_.at(curr_idx);
    const auto& prev_keyframe = keyframe_buffer_.at(curr_idx - 1);
    auto& curr_p6d_ptr = keyframe_pose6ds.at(curr_keyframe);
    auto& prev_p6d_ptr = keyframe_pose6ds.at(prev_keyframe);

    const Eigen::Matrix4d& prev_local_pose = prev_keyframe->local_pose;
    const Eigen::Matrix4d& curr_local_pose = curr_keyframe->local_pose;
    const Eigen::Matrix4d delta_transform =
        prev_local_pose.inverse() * curr_local_pose;

    const Eigen::Matrix<double, 6, 6> sqrt_info =
        Eigen::Matrix<double, 6, 1>(kInterFrameWeight.data()).asDiagonal() *
        curr_keyframe->local_pose_score;
    const auto cost = RelativePoseCost::Create(delta_transform, sqrt_info);
    problem->AddResidualBlock(cost, nullptr, prev_p6d_ptr.get(),
                              curr_p6d_ptr.get());
  }

  // Add fixed pose constraint
  for (const auto& keyframe : keyframe_buffer_) {
    auto& p6d_ptr = keyframe_pose6ds.at(keyframe);
    const Eigen::Matrix<double, 6, 6> sqrt_info =
        Eigen::Matrix<double, 6, 1>(kFixedPoseWeiht.data()).asDiagonal() *
        keyframe->global_pose_score;
    const auto cost = FixedPoseCost::Create(keyframe->global_pose, sqrt_info);
    const auto huber_loss = new ceres::HuberLoss(kFixedPoseHuberLoss);
    problem->AddResidualBlock(cost, huber_loss, p6d_ptr.get());
  }

  // solve
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = kMaxIterationsNum;
  options.num_threads = kThreadNum;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);

  // update
  for (const auto& [keyframe, p6d_ptr] : keyframe_pose6ds) {
    if (keyframe == nullptr) {
      continue;
    }

    Eigen::Map<Eigen::Matrix<double, 6, 1>> se3(p6d_ptr.get());
    keyframe->state.position = se3.bottomRows(3);
    keyframe->state.rotation = se3.topRows(3);

    Eigen::Matrix4d new_pose = Eigen::Matrix4d::Identity();
    new_pose.block<3, 3>(0, 0) = ExpSo3<double>(se3.topRows(3));
    new_pose.block<3, 1>(0, 3) = se3.bottomRows(3);
    keyframe->optimized_pose = new_pose;
  }
}

void Localization::GlobalLocalization(const sensor::LaserData& laser_data) {
  CHECK_NOTNULL(fast_correlative_scan_matcher_);

  const std::vector<Eigen::Vector3d>& eigen_points = ConvertPoint(laser_data);

  float fast_match_score = 0.f;
  transform::Rigid2d fast_pose_estimate;
  fast_correlative_scan_matcher_->MatchFullSubmap(
      eigen_points, kMinGlobalLocalizationScore, &fast_match_score,
      &fast_pose_estimate);
  if (fast_match_score < kMinGlobalLocalizationScore) {
    return;
  }
  LOG(INFO) << "fast_match_score = " << fast_match_score;

  float ceres_match_score = ceres_match_score;
  transform::Rigid2d ceres_pose_estimate = fast_pose_estimate;
  // float ceres_match_score = 0.0;
  // transform::Rigid2d ceres_pose_estimate;
  // ceres_scan_matcher_->Match(fast_pose_estimate, eigen_points,
  //                            probability_grid_, &ceres_pose_estimate,
  //                            &ceres_match_score);

  keyframe_buffer_.clear();

  float local_pose_score = 0.f;
  transform::Rigid2d local_pose_estimate;
  bool is_keyframe = false;
  local_map_builder_->AddPointCloud(
      eigen_points, transform::Rigid2d::Identity(), &local_pose_estimate,
      &local_pose_score, &is_keyframe);
  last_local_pose_ = local_pose_estimate;

  if (!is_keyframe) {
    return;
  }

  const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  new_keyframe->timestamp = laser_data.timestamp();
  new_keyframe->global_pose = ToMatrix4d(ceres_pose_estimate);
  new_keyframe->local_pose = Eigen::Matrix4d::Identity();
  new_keyframe->optimized_pose = ToMatrix4d(ceres_pose_estimate);
  new_keyframe->global_pose_score = ceres_match_score;
  new_keyframe->local_pose_score = local_pose_score;
  keyframe_buffer_.emplace_back(new_keyframe);
  ++keyframe_interval_;

  localization_status_ = LocalizationStatus::kSuccess;
}

void Localization::Relocalization(const sensor::LaserData& laser_data) {
  CHECK_NOTNULL(fast_correlative_scan_matcher_);
}

void Localization::Track(const sensor::LaserData& laser_data) {
  transform::Rigid2d local_pose_estimate;
  float local_pose_score = 0.0;
  bool is_keyframe = false;
  local_map_builder_->AddPointCloud(
      ConvertPoint(laser_data),
      transform::Project2D(
          pose_extrapolator_->ExtrapolatePose(laser_data.timestamp())),
      &local_pose_estimate, &local_pose_score, &is_keyframe);

  // local_map_builder_->AddPointCloud(ConvertPoint(laser_data),
  // last_local_pose_,
  //                                   &local_pose_estimate, &local_pose_score,
  //                                   &is_keyframe);

  transform::Rigid2d delta_pose =
      last_local_pose_.inverse() * local_pose_estimate;
  // LOG(INFO) << count_++ << ". delta = " << delta_pose.translation().x() << ",
  // "
  //           << delta_pose.translation().x() << ", "
  //           << delta_pose.rotation().angle() * 180 / M_PI;
  last_local_pose_ = local_pose_estimate;
  curr_pose_ = ToMatrix4d(local_pose_estimate);

  pose_extrapolator_->AddPose(
      laser_data.timestamp(),
      transform::Rigid3d(
          Eigen::Vector3d(local_pose_estimate.translation().x(),
                          local_pose_estimate.translation().y(), 0.),
          Eigen::AngleAxisd(local_pose_estimate.rotation().angle(),
                            Eigen::Vector3d::UnitZ())));

  // {
  //   static auto debug_node = rclcpp::Node::make_shared("submap_debug_node");
  //   static auto submap_pub =
  //       debug_node->create_publisher<sensor_msgs::msg::PointCloud2>(
  //           "map_points", 10);
  //   auto convert_to_ros = [](const std::vector<Eigen::Vector3d>& points) {
  //     pcl::PointCloud<pcl::PointXYZ> cloud;
  //     cloud.reserve(points.size());
  //     for (const Eigen::Vector3d& point : points) {
  //       cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
  //     }

  //     sensor_msgs::msg::PointCloud2 msg;
  //     pcl::toROSMsg(cloud, msg);
  //     msg.header.stamp = debug_node->now();
  //     msg.header.frame_id = "map";  // 必须和你的 Rviz Global Frame 保持一致
  //     return msg;
  //   };

  //   submap_pub->publish(
  //       convert_to_ros(local_map_builder_->map_points()));  // 地图上的匹配点
  // }
  // return;

  if (!is_keyframe) {
    return;
  }

  const Eigen::Matrix4d global_pose_guess =
      keyframe_buffer_.back()->optimized_pose *
      keyframe_buffer_.back()->local_pose.inverse() *
      ToMatrix4d(local_pose_estimate);

  const auto [global_pose, global_pose_score] =
      MatchGlobalMap(ConvertPoint(laser_data), global_pose_guess);

  const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  new_keyframe->timestamp = laser_data.timestamp();
  new_keyframe->global_pose = global_pose;
  new_keyframe->local_pose = ToMatrix4d(local_pose_estimate);
  new_keyframe->optimized_pose = global_pose;
  new_keyframe->global_pose_score = global_pose_score;
  new_keyframe->local_pose_score = local_pose_score;
  keyframe_buffer_.emplace_back(new_keyframe);
  ++keyframe_interval_;

  // {
  //   static auto debug_node = rclcpp::Node::make_shared("submap_debug_node");
  //   static auto submap_pub =
  //       debug_node->create_publisher<sensor_msgs::msg::PointCloud2>(
  //           "map_points", 10);
  //   auto convert_to_ros = [](const std::vector<Eigen::Vector3d>& points) {
  //     pcl::PointCloud<pcl::PointXYZ> cloud;
  //     cloud.reserve(points.size());
  //     for (const Eigen::Vector3d& point : points) {
  //       cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
  //     }

  //     sensor_msgs::msg::PointCloud2 msg;
  //     pcl::toROSMsg(cloud, msg);
  //     msg.header.stamp = debug_node->now();
  //     msg.header.frame_id = "map";  // 必须和你的 Rviz Global Frame 保持一致
  //     return msg;
  //   };

  //   submap_pub->publish(
  //       convert_to_ros(local_map_builder_->map_points()));  // 地图上的匹配点
  // }
}

void Localization::DistordPointCloud(const sensor::LaserData& laser_data) {
  const transform::Rigid3d start_pose =
      pose_extrapolator_->ExtrapolatePose(laser_data.timestamp());

  for (const sensor::TimedPointCloudPtr timed_point : laser_data.points()) {
    const transform::Rigid3d curr_pose =
        pose_extrapolator_->ExtrapolatePose(timed_point->timestamp);
    timed_point->position =
        start_pose.inverse() * curr_pose * timed_point->position;
  }
}

void Localization::AddLaserData(const sensor::LaserData& laser_data) {
  // std::unique_lock<std::mutex> lock(keyframe_buffer_mutex_);
  // GlobalLocalization(laser_data);
  // return;

  if (pose_extrapolator_ == nullptr) {
    pose_extrapolator_ = std::make_shared<PoseExtrapolator>();
    pose_extrapolator_->AddPose(laser_data.timestamp(),
                                transform::Rigid3d::Identity());
  }

  // DistordPointCloud(laser_data);

  // Track(laser_data);
  // return;

  switch (localization_status_) {
    case LocalizationStatus::kInitialization:
    case LocalizationStatus::kGlobalLocalization: {
      GlobalLocalization(laser_data);
      break;
    }

    case LocalizationStatus::kRelocalization: {
      Relocalization(laser_data);
      break;
    }

    case LocalizationStatus::kSuccess: {
      Track(laser_data);
      break;
    }

    default:
      break;
  }

  // if (keyframe_interval_ >= kOptimizeKeyframeInterval) {
  //   const auto t0 = std::chrono::steady_clock::now();
  //   GlobalOptimize();
  //   keyframe_interval_ = 0;
  //   const auto t1 = std::chrono::steady_clock::now();
  //   LOG(INFO) << "GlobalOptimize takes "
  //             << std::chrono::duration_cast<std::chrono::milliseconds>(t1 -
  //             t0)
  //                    .count()
  //             << "ms";
  // }

  if (keyframe_buffer_.size() > kMaxKeyframeBufferLength) {
    keyframe_buffer_.pop_front();
  }

  if (!keyframe_buffer_.empty()) {
    curr_pose_ = keyframe_buffer_.back()->optimized_pose *
                 keyframe_buffer_.back()->local_pose.inverse() *
                 ToMatrix4d(last_local_pose_);
  }
}

void Localization::AddGridMap(
    std::shared_ptr<ProbabilityGrid> probability_grid) {
  probability_grid_ = probability_grid;
  FastCorrelativeScanMatcherOptions2D options;
  options.linear_search_window = 8.0;
  options.angular_search_window = M_PI / 6.0;
  options.branch_and_bound_depth = 7;

  const auto t0 = std::chrono::steady_clock::now();
  fast_correlative_scan_matcher_ =
      std::make_shared<FastCorrelativeScanMatcher2D>(*probability_grid,
                                                     options);
  const auto t1 = std::chrono::steady_clock::now();
  LOG(INFO)
      << "FastCorrelativeScanMatcher2D takes "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << "ms";
}

void Localization::AddInitialPose(const Eigen::Matrix4d& initial_pose) {
  std::unique_lock<std::mutex> lock(keyframe_buffer_mutex_);
  initial_pose_ = initial_pose;
  localization_status_ = LocalizationStatus::kGlobalLocalization;

  local_map_builder_ = std::make_shared<LocalMapBuilder>();
  pose_extrapolator_ = nullptr;
}

void Localization::AddImuData(const sensor::ImuData& imu_data) {}

void Localization::AddOdometryData(const sensor::OdometryData& odometry_data) {}

void Localization::Init() {
  ceres_scan_matcher_ = std::make_shared<CeresScanMatcher2D>(
      kOccupiedSpaceWeight, kTranslationWeight, kRotationWeight);

  local_map_builder_ = std::make_shared<LocalMapBuilder>();

  RealTimeCorrelativeScanMatcherOptions2D options;
  options.linear_search_window = 0.2;
  options.angular_search_window = 5.0 * kDegreeToRadian;
  options.translation_delta_cost_weight = 0.1;
  options.rotation_delta_cost_weight = 0.1;
  real_time_correlative_scan_matcher_ =
      std::make_shared<RealTimeCorrelativeScanMatcher2D>(options);
}

}  // namespace solex_robot::navigation::localization_2d
