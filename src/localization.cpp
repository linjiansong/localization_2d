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

Eigen::Vector3d TransformPoint(const Eigen::Matrix4d& transform,
                               const Eigen::Vector3d& point) {
  return transform.block<3, 3>(0, 0) * point + transform.block<3, 1>(0, 3);
}

std::vector<Eigen::Vector3d> ConvertPoint(const PointCloud& point_cloud) {
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.reserve(point_cloud.points.size());
  for (const TimedPointCloud& timed_point : point_cloud.points) {
    eigen_points.emplace_back(timed_point.position);
  }

  return eigen_points;
}
}  // namespace

std::pair<Eigen::Matrix4d, double> Localization::MatchGlobalMap(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Matrix4d& initial_pose) {
  // LOG(INFO) << "Match global map";
  if (probability_grid_ == nullptr) {
    LOG(ERROR) << "Not probability grid map found!";
    return std::make_pair(initial_pose, 0.0);
  }

  if (ceres_scan_matcher_ == nullptr) {
    LOG(ERROR) << "Ceres scan matcher has not been initialized!";
    return std::make_pair(initial_pose, 0.0);
  }

  Eigen::Matrix4d global_pose = initial_pose;
  double match_score = 0.0;
  ceres_scan_matcher_->Match(initial_pose, points, probability_grid_,
                             &global_pose, &match_score);

  // return std::make_pair(global_pose, match_score);
  return std::make_pair(global_pose, match_score);
}

std::pair<Eigen::Matrix4d, double> Localization::MatchLocalMap(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Matrix4d& initial_pose) {
  CHECK_NOTNULL(search_tree_);
  const auto t0 = std::chrono::steady_clock::now();
  // const Eigen::Matrix4d local_pose = ICP(points, initial_pose);
  const Eigen::Matrix4d local_pose = Eigen::Matrix4d::Identity();


  // calculate score
  int inlier_count = 0;
  double mean_error = 0.0;
  for (const Eigen::Vector3d& point : points) {
    const Eigen::Vector2d query_point =
        TransformPoint(local_pose, point).head(2);
    std::vector<size_t> indices(1);
    std::vector<double> sqr_distances(1, std::numeric_limits<double>::max());
    search_tree_->Query(query_point.data(), 1, indices.data(),
                        sqr_distances.data());
    const double distance = std::sqrt(sqr_distances[0]);
    if (distance < 0.1) {
      mean_error += distance;
      ++inlier_count;
    }
  }

  mean_error /= (inlier_count + kEpsilon);

  const double score = inlier_count / (points.size() + kEpsilon);
  const auto t1 = std::chrono::steady_clock::now();
  LOG(INFO)
      << "MatchLocalMap takes "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << "ms";

  return std::make_pair(local_pose, score);
}

void Localization::UpdateKeyframeBuffer() {
  if (!update_keyframe_buffer_) {
    return;
  }

  std::unique_lock<std::mutex> lock(keyframe_buffer_mutex_);
  if (keyframe_buffer_.size() > kMaxKeyframeBufferLength) {
    keyframe_buffer_.pop_front();
    // update local map origin
    const Eigen::Matrix4d transform =
        keyframe_buffer_.front()->local_pose.inverse();
    for (const auto& keyframe : keyframe_buffer_) {
      keyframe->local_pose = transform * keyframe->local_pose;
    }
  }

  std::vector<Eigen::Vector2d> points_2d;
  for (const auto& keyframe : keyframe_buffer_) {
    for (const TimedPointCloud& timed_point : keyframe->point_cloud.points) {
      const Eigen::Vector3d transformed_point =
          TransformPoint(keyframe->local_pose, timed_point.position);
      points_2d.emplace_back(transformed_point.head(2));
    }
  }

  if (!points_2d.empty()) {
    search_tree_ = std::make_unique<KDTree2D>(2, points_2d);
  }

  update_keyframe_buffer_ = false;
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

  update_keyframe_buffer_ = true;
  keyframe_interval_ = 0;
}

void Localization::GlobalLocalization(const PointCloud& point_cloud) {
  CHECK_NOTNULL(fast_correlative_scan_matcher_);

  std::vector<Eigen::Vector3f> eigen_points;
  eigen_points.reserve(point_cloud.points.size());
  for (const TimedPointCloud& timed_point : point_cloud.points) {
    eigen_points.emplace_back(timed_point.position.cast<float>());
  }

  float match_score = 0.f;
  transform::Rigid2d pose_estimate;
  fast_correlative_scan_matcher_->MatchFullSubmap(
      eigen_points, kMinGlobalLocalizationScore, &match_score, &pose_estimate);
  if (match_score < kMinGlobalLocalizationScore) {
    return;
  }

  LOG(INFO) << "match_score = " << match_score;
  Eigen::Matrix4d global_pose = Eigen::Matrix4d::Identity();
  global_pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(pose_estimate.rotation().angle(),
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  global_pose(0, 3) = pose_estimate.translation().x();
  global_pose(1, 3) = pose_estimate.translation().y();

  keyframe_buffer_.clear();
  search_tree_ = nullptr;

  const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  new_keyframe->timestamp = point_cloud.timestamp;
  new_keyframe->point_cloud = point_cloud;
  new_keyframe->global_pose = global_pose;
  new_keyframe->local_pose = Eigen::Matrix4d::Identity();
  new_keyframe->optimized_pose = global_pose;
  new_keyframe->global_pose_score = match_score;
  new_keyframe->local_pose_score = 1.0;
  keyframe_buffer_.emplace_back(new_keyframe);

  // const auto [global_pose2, global_pose_score] =
  //     MatchGlobalMap(ConvertPoint(point_cloud), global_pose);

  ++keyframe_interval_;
  update_keyframe_buffer_ = true;
  localization_status_ = LocalizationStatus::kSuccess;
  curr_pose_ = global_pose;
  LOG(INFO) << "global_pose = " << global_pose;
}

void Localization::Relocalization(const PointCloud& point_cloud) {
  CHECK_NOTNULL(fast_correlative_scan_matcher_);

  const std::vector<Eigen::Vector3d> eigen_points = ConvertPoint(point_cloud);
}

void Localization::Track(const PointCloud& point_cloud) {
  // match to local map
  const Eigen::Vector2d translation(curr_pose_(0, 3), curr_pose_(1, 3));
  const double rotation_angle = std::atan2(curr_pose_(1, 0), curr_pose_(0, 0));
  const transform::Rigid2d initial_pose(translation, rotation_angle);

  transform::Rigid2d pose_estimate;
  double score = 0.0;
  local_map_builder_->AddPointCloud(ConvertPoint(point_cloud), initial_pose,
                                    &pose_estimate, &score);

  Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
  curr_pose_.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(pose_estimate.rotation().angle(),
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  curr_pose_(0, 3) = pose_estimate.translation().x();
  curr_pose_(1, 3) = pose_estimate.translation().y();

  {
    static auto debug_node = rclcpp::Node::make_shared("submap_debug_node");
    static auto submap_pub =
        debug_node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "map_points", 10);
    auto convert_to_ros = [](const std::vector<Eigen::Vector3d>& points) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.reserve(points.size());
      for (const Eigen::Vector3d& point : points) {
        cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
      }

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.stamp = debug_node->now();
      msg.header.frame_id = "map";  // 必须和你的 Rviz Global Frame 保持一致
      return msg;
    };

    submap_pub->publish(
        convert_to_ros(local_map_builder_->map_points()));  // 地图上的匹配点
  }

  // if (!keyframe_buffer_.empty()) {
  //   const auto [curr_local_pose, local_pose_score] = MatchLocalMap(
  //       ConvertPoint(point_cloud),
  //       keyframe_buffer_.front()->optimized_pose.inverse() * curr_pose_);
  //   const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  //   curr_pose_ = keyframe_buffer_.front()->optimized_pose * curr_local_pose;
  //   new_keyframe->timestamp = point_cloud.timestamp;
  //   new_keyframe->point_cloud = point_cloud;
  //   new_keyframe->global_pose = curr_pose_;
  //   new_keyframe->local_pose = curr_local_pose;
  //   new_keyframe->optimized_pose = curr_pose_;
  //   new_keyframe->global_pose_score = local_pose_score;
  //   new_keyframe->local_pose_score = local_pose_score;
  //   keyframe_buffer_.emplace_back(new_keyframe);
  //   ++keyframe_interval_;
  //   update_keyframe_buffer_ = true;
  // } else {
  //   const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  //   curr_pose_ = Eigen::Matrix4d::Identity();
  //   new_keyframe->timestamp = point_cloud.timestamp;
  //   new_keyframe->point_cloud = point_cloud;
  //   new_keyframe->global_pose = Eigen::Matrix4d::Identity();
  //   new_keyframe->local_pose = Eigen::Matrix4d::Identity();
  //   new_keyframe->optimized_pose = Eigen::Matrix4d::Identity();
  //   new_keyframe->global_pose_score = 0;
  //   new_keyframe->local_pose_score = 0;
  //   keyframe_buffer_.emplace_back(new_keyframe);
  //   ++keyframe_interval_;
  //   update_keyframe_buffer_ = true;
  // }
  // return;

  // Check keyframe
  // const Eigen::Matrix4d prev_local_pose =
  // keyframe_buffer_.back()->local_pose; const auto [curr_local_pose,
  // local_pose_score] = MatchLocalMap(
  //     ConvertPoint(point_cloud),
  //     keyframe_buffer_.front()->optimized_pose.inverse() * curr_pose_);
  // curr_pose_ = keyframe_buffer_.front()->optimized_pose * curr_local_pose;

  // const Eigen::Matrix4d delta_pose =
  //     prev_local_pose.inverse() * curr_local_pose;
  // const double distance =
  //     delta_pose.block<3, 1>(0, 3).norm();  // meterinitial_pose
  // const Eigen::AngleAxisd axisd_angle(delta_pose.block<3, 3>(0, 0));
  // const double angle =
  //     std::abs(axisd_angle.angle() * kRadianToDegree);  // degree
  // if (distance < kKeyframeMinDistance && angle < kKeyframeMinAngle) {
  //   return;
  // }

  // // calculate initial pose
  // const Eigen::Matrix4d initial_pose =
  //     keyframe_buffer_.front()->optimized_pose * curr_local_pose;
  // const Eigen::Vector2d translation(initial_pose(0, 3), initial_pose(1, 3));
  // const double rotation_angle =
  //     std::atan2(initial_pose(1, 0), initial_pose(0, 0));
  // const transform::Rigid2d initial_pose_estimate(translation,
  // rotation_angle);

  // std::vector<Eigen::Vector3f> eigen_points;
  // eigen_points.reserve(point_cloud.points.size());
  // for (const TimedPointCloud& timed_point : point_cloud.points) {
  //   eigen_points.emplace_back(timed_point.position.cast<float>());
  // }

  // transform::Rigid2d pose_estimate;
  // float score;
  // real_time_correlative_scan_matcher_->Match(initial_pose_estimate,
  //                                            eigen_points,
  //                                            *probability_grid_,
  //                                            &pose_estimate, &score);
  // LOG(INFO) << "++++++++++++++++++++++++++ real time = " << score;
  // Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
  // final_pose.block<3, 3>(0, 0) =
  //     Eigen::AngleAxisd(pose_estimate.rotation().angle(),
  //                       Eigen::Vector3d::UnitZ())
  //         .toRotationMatrix();
  // final_pose(0, 3) = pose_estimate.translation().x();
  // final_pose(1, 3) = pose_estimate.translation().y();

  // const auto [global_pose, global_pose_score] =
  //     MatchGlobalMap(ConvertPoint(point_cloud), final_pose);
  // const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  // new_keyframe->timestamp = point_cloud.timestamp;
  // new_keyframe->point_cloud = point_cloud;
  // new_keyframe->global_pose = global_pose;
  // new_keyframe->local_pose = curr_local_pose;
  // new_keyframe->optimized_pose = global_pose;
  // new_keyframe->global_pose_score = global_pose_score;
  // new_keyframe->local_pose_score = local_pose_score;
  // keyframe_buffer_.emplace_back(new_keyframe);
  // ++keyframe_interval_;
  // update_keyframe_buffer_ = true;
}

void Localization::AddPointCloud(const PointCloud& point_cloud) {
  const auto t0 = std::chrono::steady_clock::now();
  Track(point_cloud);
  UpdateKeyframeBuffer();
  const auto t1 = std::chrono::steady_clock::now();
  LOG(INFO)
      << "AddPointCloud takes "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << "ms";
  return;

  switch (localization_status_) {
    case LocalizationStatus::kInitialization:
    case LocalizationStatus::kGlobalLocalization: {
      GlobalLocalization(point_cloud);
      break;
    }

    case LocalizationStatus::kRelocalization: {
      Relocalization(point_cloud);
      break;
    }

    case LocalizationStatus::kSuccess: {
      Track(point_cloud);
      break;
    }

    default:
      break;
  }

  if (keyframe_interval_ >= kOptimizeKeyframeInterval) {
    const auto t0 = std::chrono::steady_clock::now();
    GlobalOptimize();
    const auto t1 = std::chrono::steady_clock::now();
    LOG(INFO) << "GlobalOptimize takes "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                     .count()
              << "ms";
  }

  UpdateKeyframeBuffer();
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
}

void Localization::AddImuData() {}

void Localization::AddOdometryData() {}

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
