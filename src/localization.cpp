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
constexpr double kKeyframeMinDistance = 0.2;  // meter
constexpr double kKeyframeMinAngle = 5.0;     // degree
constexpr double kICPMinDistance = 1e-3;      // meter
constexpr double kICPMinAngle = 0.1;          // degree

constexpr double kOccupiedSpaceWeight = 10.0;
constexpr double kTranslationWeight = 10.0;
constexpr double kRotationWeight = 40.0;

constexpr int kMaxKeyframeBufferLength = 2;
constexpr int kOptimizeKeyframeInterval = 5;

constexpr int kMaxIterationsNum = 50;
constexpr int kThreadNum = 8;

constexpr std::array<double, 6> kInterFrameWeight = {1.e3, 1.e3, 1.e3,
                                                     1.e2, 1.e2, 1.e2};
constexpr std::array<double, 6> kFixedPoseWeiht = {1.e3, 1.e3, 1.e3,
                                                   1.e2, 1.e2, 1.e2};

constexpr int kFixedPoseHuberLoss = 20.0;

Eigen::Vector3d TransformPoint(const Eigen::Matrix4d& transform,
                               const Eigen::Vector3d& point) {
  return transform.block<3, 3>(0, 0) * point + transform.block<3, 1>(0, 3);
}
}  // namespace

// SVD计算两个点云之间的最佳旋转平移矩阵(Kabsch 算法)
Eigen::Matrix3d Localization::ComputeTransformation2D(
    const std::vector<Eigen::Vector2d>& source_points,
    const std::vector<Eigen::Vector2d>& target_points) {
  // 1. 计算质心
  Eigen::Vector2d source_mean = Eigen::Vector2d::Zero();
  Eigen::Vector2d target_mean = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < source_points.size(); ++i) {
    source_mean += source_points[i];
    target_mean += target_points[i];
  }
  source_mean /= source_points.size();
  target_mean /= target_points.size();

  // 2. 去质心坐标
  Eigen::MatrixXd new_source_points(2, source_points.size());
  Eigen::MatrixXd new_target_points(2, target_points.size());
  for (size_t i = 0; i < source_points.size(); ++i) {
    new_source_points.col(i) = source_points[i] - source_mean;
    new_target_points.col(i) = target_points[i] - target_mean;
  }

  // 3. 计算协方差矩阵 matrix_h
  const Eigen::Matrix2d matrix_h =
      new_source_points * new_target_points.transpose();

  // 4. SVD 分解
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      matrix_h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();

  // 5. 处理反射情况 (2D 中行列式必须为 1，否则就是镜像)
  if (rotation.determinant() < 0) {
    Eigen::Matrix2d matrix_v = svd.matrixV();
    matrix_v.col(1) *= -1;  // 翻转最后一列
    rotation = matrix_v * svd.matrixU().transpose();
  }

  // 6. 计算平移
  const Eigen::Vector2d translation = target_mean - rotation * source_mean;

  // 7. 构建 3x3 齐次矩阵
  Eigen::Matrix3d final_transform = Eigen::Matrix3d::Identity();
  final_transform.block<2, 2>(0, 0) = rotation;
  final_transform.block<2, 1>(0, 2) = translation;
  return final_transform;
}

// Eigen::Matrix3d Localization::ComputeTransformation2D(
//     const std::vector<Eigen::Vector2d>& source_points,
//     const std::vector<Eigen::Vector2d>& target_points) {
//   if (source_points.empty()) return Eigen::Matrix3d::Identity();

//   Eigen::Vector2d source_mean = Eigen::Vector2d::Zero();
//   Eigen::Vector2d target_mean = Eigen::Vector2d::Zero();
//   for (size_t i = 0; i < source_points.size(); ++i) {
//     source_mean += source_points[i];
//     target_mean += target_points[i];
//   }
//   source_mean /= source_points.size();
//   target_mean /= target_points.size();

//   double Sxx = 0.0, Sxy = 0.0, Syx = 0.0, Syy = 0.0;
//   for (size_t i = 0; i < source_points.size(); ++i) {
//     const double sx = source_points[i].x() - source_mean.x();
//     const double sy = source_points[i].y() - source_mean.y();
//     const double tx = target_points[i].x() - target_mean.x();
//     const double ty = target_points[i].y() - target_mean.y();
//     Sxx += sx * tx;  Sxy += sx * ty;
//     Syx += sy * tx;  Syy += sy * ty;
//   }

//   // 闭式解直接计算旋转角，完美解决 SVD 滞后问题
//   const double theta = std::atan2(Sxy - Syx, Sxx + Syy);
//   Eigen::Matrix2d rotation;
//   rotation << std::cos(theta), -std::sin(theta),
//               std::sin(theta),  std::cos(theta);

//   const Eigen::Vector2d translation = target_mean - rotation * source_mean;

//   Eigen::Matrix3d final_transform = Eigen::Matrix3d::Identity();
//   final_transform.block<2, 2>(0, 0) = rotation;
//   final_transform.block<2, 1>(0, 2) = translation;

//   return final_transform;
// }

Eigen::Matrix3d Localization::ICP(const std::vector<Eigen::Vector3d>& points,
                                  const int max_iterations) {
  // static auto debug_node = rclcpp::Node::make_shared("icp_debug_node");
  // static auto source_pub =
  //     debug_node->create_publisher<sensor_msgs::msg::PointCloud2>("icp_source",
  //                                                                 10);
  // static auto target_pub =
  //     debug_node->create_publisher<sensor_msgs::msg::PointCloud2>("icp_target",
  //                                                                 10);
  // static auto final_pub =
  //     debug_node->create_publisher<sensor_msgs::msg::PointCloud2>("icp_final",
  //                                                                 10);

  Eigen::Matrix3d final_transform = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d prev_transform = final_transform;
  int final_iter = 0;
  std::vector<Eigen::Vector2d> source_points;
  std::vector<Eigen::Vector2d> target_points;
  for (int iter = 0; iter < max_iterations; ++iter) {
    final_iter = iter;
    // std::vector<Eigen::Vector2d> source_points;
    // std::vector<Eigen::Vector2d> target_points;
    source_points.clear();
    target_points.clear();

    source_points.reserve(points.size());
    target_points.reserve(points.size());
    for (const Eigen::Vector3d& point : points) {
      const Eigen::Vector2d query_point =
          final_transform.block<2, 2>(0, 0) * point.head(2) +
          final_transform.block<2, 1>(0, 2);

      std::vector<size_t> indices(1);
      std::vector<double> sqr_distances(1, std::numeric_limits<double>::max());
      search_tree_->Query(query_point.data(), 1, indices.data(),
                          sqr_distances.data());
      const double distance = std::sqrt(sqr_distances[0]);
      if (distance < 1.0) {
        source_points.emplace_back(point.head(2));
        target_points.emplace_back(search_tree_->get_data(indices[0]));
      }
    }

    if (source_points.size() < 0.15 * points.size()) {
      break;
    }

    final_transform = ComputeTransformation2D(source_points, target_points);
    const Eigen::Matrix3d delta_transform =
        prev_transform.inverse() * final_transform;
    const double delta_distance =
        delta_transform.block<2, 1>(0, 2).norm();  // meter
    const double delta_angle =
        std::abs(std::atan2(delta_transform(1, 0), delta_transform(0, 0)));
    if (delta_distance < kICPMinDistance && delta_angle < kICPMinAngle) {
      break;
    }

    prev_transform = final_transform;
  }

  // LOG(INFO) << "final_iter = " << final_iter;

  // [临时调试区]: 发布最后一次迭代的点云
  // if (!source_points.empty() && !target_points.empty()) {
  //   auto convert_to_ros = [](const std::vector<Eigen::Vector2d>& pts) {
  //     pcl::PointCloud<pcl::PointXYZ> cloud;
  //     cloud.reserve(pts.size());
  //     for (const auto& p : pts) {
  //       cloud.push_back(pcl::PointXYZ(p.x(), p.y(), 0.0));
  //     }
  //     sensor_msgs::msg::PointCloud2 msg;
  //     pcl::toROSMsg(cloud, msg);
  //     msg.header.stamp = debug_node->now();
  //     msg.header.frame_id = "map";  // 必须和你的 Rviz Global Frame 保持一致
  //     return msg;
  //   };

  //   source_pub->publish(convert_to_ros(source_points));  // 当前帧变换后的点
  //   target_pub->publish(convert_to_ros(target_points));  // 地图上的匹配点

  //   std::vector<Eigen::Vector2d> final_points;
  //   for (const Eigen::Vector2d& point : source_points) {
  //     const Eigen::Vector2d query_point =
  //         final_transform.block<2, 2>(0, 0) * point +
  //         final_transform.block<2, 1>(0, 2);
  //     final_points.emplace_back(query_point);
  //   }
  //   final_pub->publish(convert_to_ros(final_points));  // 地图上的匹配点
  // }
  // =========================================================================

  // LOG(INFO) << "final_iter = " << final_iter;

  return final_transform;
}

std::pair<Eigen::Matrix4d, double> Localization::MatchGlobalMap(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Matrix4d& initial_pose) {
  LOG(INFO) << "Match global map";
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

  LOG(INFO) << "match_score = " << match_score;

  // return std::make_pair(global_pose, match_score);
  return std::make_pair(global_pose, match_score);
}

std::pair<Eigen::Matrix4d, double> Localization::MatchLobalMap(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Matrix4d& initial_pose) {
  CHECK_NOTNULL(search_tree_);
  const auto t0 = std::chrono::steady_clock::now();
  const Eigen::Matrix3d delta_transform = ICP(points);

  // calculate score
  int inlier_count = 0;
  double mean_error = 0.0;
  for (const Eigen::Vector3d& point : points) {
    const Eigen::Vector2d query_point =
        delta_transform.block<2, 2>(0, 0) * point.head(2) +
        delta_transform.block<2, 1>(0, 2);

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

  // LOG(INFO) << "mean_error = " << mean_error
  //           << ", inlier_count = " << inlier_count;

  const double score = inlier_count / (points.size() + kEpsilon);

  Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
  final_transform.block<2, 2>(0, 0) = delta_transform.block<2, 2>(0, 0);
  final_transform.block<2, 1>(0, 3) = delta_transform.block<2, 1>(0, 2);

  const auto t1 = std::chrono::steady_clock::now();
  // LOG(INFO)
  //     << "MatchLobalMap takes "
  //     << std::chrono::duration_cast<std::chrono::milliseconds>(t1 -
  //     t0).count()
  //     << "ms";

  return std::make_pair(final_transform * initial_pose, score);
}

void Localization::UpdateKeyframeBuffer() {
  if (!update_keyframe_buffer_) {
    return;
  }

  std::unique_lock<std::mutex> lock(keyframe_buffer_mutex_);
  if (keyframe_buffer_.size() > kMaxKeyframeBufferLength) {
    keyframe_buffer_.pop_front();
  }

  std::vector<Eigen::Vector2d> points_2d;
  for (const auto& keyframe : keyframe_buffer_) {
    for (const TimedPointCloud& timed_point : keyframe->point_cloud.points) {
      const Eigen::Vector3d transformed_point =
          TransformPoint(keyframe->optimized_pose, timed_point.position);
      points_2d.emplace_back(transformed_point.head(2));
    }
  }

  LOG(INFO) << "points_2d = " << points_2d.size();

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
    p6d.topRows(3) = keyframe->state.rotation;
    p6d.bottomRows(3) = keyframe->state.position;
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

void Localization::AddPointCloud(const PointCloud& point_cloud) {
  // LOG(INFO) << "Add new point cloud";
  // transform points
  // const Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d initial_pose =
      keyframe_buffer_.empty() ? initial_pose_
                               : keyframe_buffer_.back()->optimized_pose;
  std::vector<Eigen::Vector3d> transformed_points;
  transformed_points.reserve(point_cloud.points.size());
  for (const TimedPointCloud& timed_point : point_cloud.points) {
    const Eigen::Vector3d transformed_point =
        TransformPoint(initial_pose, timed_point.position);
    transformed_points.emplace_back(transformed_point);
  }

  const Eigen::Matrix4d global_pose = initial_pose_;
  const float global_pose_score = 0.0;

  const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  new_keyframe->timestamp = point_cloud.timestamp;
  new_keyframe->point_cloud = point_cloud;
  new_keyframe->global_pose = global_pose;
  new_keyframe->local_pose = global_pose;
  new_keyframe->optimized_pose = global_pose;
  new_keyframe->global_pose_score = global_pose_score;
  new_keyframe->local_pose_score = global_pose_score;
  keyframe_buffer_.emplace_back(new_keyframe);
  ++keyframe_interval_;
  update_keyframe_buffer_ = true;

  // if (keyframe_buffer_.empty()) {
  //   // match to global map
  //   const auto [global_pose, global_pose_score] =
  //       MatchGlobalMap(transformed_points, initial_pose);

  //   const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  //   new_keyframe->timestamp = point_cloud.timestamp;
  //   new_keyframe->point_cloud = point_cloud;
  //   new_keyframe->global_pose = global_pose;
  //   new_keyframe->local_pose = global_pose;
  //   new_keyframe->optimized_pose = global_pose;
  //   new_keyframe->global_pose_score = global_pose_score;
  //   new_keyframe->local_pose_score = global_pose_score;
  //   keyframe_buffer_.emplace_back(new_keyframe);
  //   ++keyframe_interval_;
  //   update_keyframe_buffer_ = true;
  // } else {
  //   // match to local map
  //   const auto [curr_local_pose, local_pose_score] =
  //       MatchLobalMap(transformed_points, initial_pose);

  //   {
  //     static auto debug_node = rclcpp::Node::make_shared("icp_debug_node");
  //     static auto source_pub =
  //         debug_node->create_publisher<sensor_msgs::msg::PointCloud2>(
  //             "icp_source", 10);
  //     static auto target_pub =
  //         debug_node->create_publisher<sensor_msgs::msg::PointCloud2>(
  //             "icp_target", 10);
  //     static auto final_pub =
  //         debug_node->create_publisher<sensor_msgs::msg::PointCloud2>(
  //             "icp_final", 10);

  //     auto convert_to_ros = [](const std::vector<Eigen::Vector3d>& pts) {
  //       pcl::PointCloud<pcl::PointXYZ> cloud;
  //       cloud.reserve(pts.size());
  //       for (const auto& p : pts) {
  //         cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  //       }
  //       sensor_msgs::msg::PointCloud2 msg;
  //       pcl::toROSMsg(cloud, msg);
  //       msg.header.stamp = debug_node->now();
  //       msg.header.frame_id = "map";  // 必须和你的 Rviz Global Frame
  //       保持一致 return msg;
  //     };

  //     // 当前帧变换后的点
  //     source_pub->publish(convert_to_ros(transformed_points));

  //     std::vector<Eigen::Vector3d> target_points;
  //     for (const auto& keyframe : keyframe_buffer_) {
  //       for (const TimedPointCloud& timed_point :
  //            keyframe->point_cloud.points) {
  //         const Eigen::Vector3d transformed_point =
  //             TransformPoint(keyframe->optimized_pose, timed_point.position);
  //         target_points.emplace_back(transformed_point);
  //       }
  //     }
  //     target_pub->publish(convert_to_ros(target_points));  // 地图上的匹配点

  //     std::vector<Eigen::Vector3d> final_points;
  //     final_points.reserve(point_cloud.points.size());
  //     for (const TimedPointCloud& timed_point : point_cloud.points) {
  //       const Eigen::Vector3d transformed_point =
  //           TransformPoint(curr_local_pose, timed_point.position);
  //       final_points.emplace_back(transformed_point);
  //     }

  //     final_pub->publish(convert_to_ros(final_points));  // Z
  //   }

  //   // LOG(INFO) << "curr_local_pose = " << curr_local_pose;
  //   // LOG(INFO) << "local_pose_score = " << local_pose_score;

  //   const Eigen::Matrix4d prev_local_pose =
  //   keyframe_buffer_.back()->local_pose; const Eigen::Matrix4d delta_pose =
  //       prev_local_pose.inverse() * curr_local_pose;
  //   const double distance = delta_pose.block<3, 1>(0, 3).norm();  // meter
  //   const Eigen::AngleAxisd axisd_angle(delta_pose.block<3, 3>(0, 0));
  //   const double angle =
  //       std::abs(axisd_angle.angle() * kRadianToDegree);  // degree
  //   // LOG(INFO) << "angle = " << angle << ", distance = " << distance;
  //   if (distance > kKeyframeMinDistance || angle > kKeyframeMinAngle) {
  //     // if (true) {
  //     const auto [global_pose, global_pose_score] =
  //         MatchGlobalMap(transformed_points, curr_local_pose);
  //     const KeyframePtr new_keyframe = std::make_shared<Keyframe>();
  //     new_keyframe->timestamp = point_cloud.timestamp;
  //     new_keyframe->point_cloud = point_cloud;
  //     new_keyframe->global_pose = global_pose;
  //     new_keyframe->local_pose = curr_local_pose;
  //     new_keyframe->optimized_pose = curr_local_pose;
  //     new_keyframe->global_pose_score = global_pose_score;
  //     new_keyframe->local_pose_score = local_pose_score;
  //     keyframe_buffer_.emplace_back(new_keyframe);
  //     ++keyframe_interval_;
  //     update_keyframe_buffer_ = true;
  //   }
  // }

  // if (keyframe_interval_ >= kOptimizeKeyframeInterval) {
  //   GlobalOptimize();
  //   update_keyframe_buffer_ = true;
  //   keyframe_interval_ = 0;
  // }

  // if (update_keyframe_buffer_) {
  //   UpdateKeyframeBuffer();
  // }
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
  if (keyframe_buffer_.empty()) {
    LOG(ERROR) << "Not laser scan data, please try again later";
    return;
  }

  std::vector<Eigen::Vector3f> point_cloud;
  for (const TimedPointCloud& timed_point :
       keyframe_buffer_.back()->point_cloud.points) {
    point_cloud.emplace_back(timed_point.position.cast<float>());
  }

  const float min_score = 0.4;
  float match_score = 0.f;
  transform::Rigid2d pose_estimate;
  fast_correlative_scan_matcher_->MatchFullSubmap(point_cloud, min_score,
                                                  &match_score, &pose_estimate);
  LOG(INFO) << "match_score = " << match_score;

  // {
  //   {
  //     const int width =
  //         probability_grid_->map_limits().cell_limits().num_x_cells;
  //     const int height =
  //         probability_grid_->map_limits().cell_limits().num_y_cells;
  //     cv::Mat image(height, width, CV_8UC3);
  //     for (int y = 0; y < height; ++y) {
  //       for (int x = 0; x < width; ++x) {
  //         const Eigen::Array2i index(x, y);
  //         const float probability = probability_grid_->GetProbability(index);
  //         const u_char val = static_cast<u_char>((1.0f - probability) * 255.0f);
  //         image.at<cv::Vec3b>(y, x) = cv::Vec3b(val, val, val);
  //       }
  //     }

  //     transform::Rigid3f rigid3f(
  //         Eigen::Vector3f(pose_estimate.translation().x(),
  //                         pose_estimate.translation().y(), 0.),
  //         Eigen::AngleAxisf(pose_estimate.rotation().angle(),
  //                           Eigen::Vector3f::UnitZ()));

  //     std::vector<Eigen::Vector3f> rotated_point_cloud;
  //     rotated_point_cloud.reserve(point_cloud.size());
  //     for (const Eigen::Vector3f& point : point_cloud) {
  //       rotated_point_cloud.emplace_back(rigid3f * point);
  //     }

  //     for (const Eigen::Vector3f point : rotated_point_cloud) {
  //       const Eigen::Array2i index =
  //           probability_grid_->map_limits().GetCellIndex(point.head(2));
  //       image.at<cv::Vec3b>(index.y(), index.x()) = cv::Vec3b(0, 0, 255);
  //     }

  //     cv::imwrite("/home/linjs/图片/match.png", image);
  //   }

  //   // Todo:
  //   transform::Rigid3f rigid3f(
  //       Eigen::Vector3f(pose_estimate.translation().x(),
  //                       pose_estimate.translation().y(), 0.),
  //       Eigen::AngleAxisf(pose_estimate.rotation().angle(),
  //                         Eigen::Vector3f::UnitZ()));

  //   std::vector<Eigen::Vector3f> rotated_point_cloud;
  //   rotated_point_cloud.reserve(point_cloud.size());
  //   for (const Eigen::Vector3f& point : point_cloud) {
  //     rotated_point_cloud.emplace_back(rigid3f * point);
  //   }

  //   float score = 0.0;
  //   for (const Eigen::Vector3f point : rotated_point_cloud) {
  //     const Eigen::Array2i index =
  //         probability_grid_->map_limits().GetCellIndex(point.head(2));
  //     score += probability_grid_->GetProbability(index);
  //   }

  //   LOG(INFO) << "---------------------score = "
  //             << score / rotated_point_cloud.size();
  // }

  initial_pose_.block<2, 1>(0, 3) = pose_estimate.translation();
  initial_pose_.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(pose_estimate.rotation().angle(),
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();

  LOG(INFO) << "initial_pose = " << initial_pose_;

  keyframe_buffer_.clear();
  search_tree_ = nullptr;
}

void Localization::AddImuData() {}

void Localization::AddOdometryData() {}

void Localization::Init() {
  ceres_scan_matcher_ = std::make_shared<CeresScanMatcher2D>(
      kOccupiedSpaceWeight, kTranslationWeight, kRotationWeight);
}

}  // namespace solex_robot::navigation::localization_2d
