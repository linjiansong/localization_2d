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

#include "include/pose_graph/pose_graph.h"

#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "common/base_type.h"
#include "common/ceres_helper.h"
#include "common/rigid_transform.h"
#include "include/scan_match/probability_grid.h"

namespace solex_robot::navigation::localization_2d {
namespace {
constexpr double kDegreeToRadian = M_PI / 180.0;

constexpr int kOptimizeNodeInterval = 10;
constexpr int kMaxNodeBufferLength = 40;

constexpr int kMaxIterationsNum = 50;
constexpr int kThreadNum = 8;

constexpr std::array<double, 6> kInterFrameWeight = {1.e3, 1.e3, 1.e3,
                                                     1.e2, 1.e2, 1.e2};
constexpr std::array<double, 6> kFixedPoseWeiht = {1.e2, 1.e2, 1.e2,
                                                   10.0, 10.0, 10.0};

constexpr int kFixedPoseHuberLoss = 20.0;
constexpr int kMinTrackConstraintScore = 0.3;
constexpr int kMinGlobalLocalizationScore = 0.3;
constexpr int kMinRelocalizationScore = 0.4;

Eigen::Matrix4d ToMatrix4d(const transform::Rigid2d& rigid_pose) {
  Eigen::Matrix4d eigen_pose = Eigen::Matrix4d::Identity();
  eigen_pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(rigid_pose.rotation().angle(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  eigen_pose(0, 3) = rigid_pose.translation().x();
  eigen_pose(1, 3) = rigid_pose.translation().y();
  return eigen_pose;
}

}  // namespace

void PoseGraph::ComputeTrackConstraint(
    const ConstraintDataPtr& constraint_data) {
  transform::Rigid2d initial_pose_estimate = transform::Rigid2d::Identity();
  {
    // calculate initial guess
    std::unique_lock<std::mutex> lock(node_buffer_mutex_);
    if (!node_buffer_.empty()) {
      const NodePtr latest_node = node_buffer_.back();
      initial_pose_estimate =
          latest_node->optimized_pose *
          latest_node->constraint_data->local_pose.inverse() *
          constraint_data->local_pose;
    }
  }

  transform::Rigid2d real_time_pose_estimate;
  float real_time_score = 0.0;
  real_time_correlative_scan_matcher_->Match(
      initial_pose_estimate, constraint_data->points, *probability_grid_,
      &real_time_pose_estimate, &real_time_score);

  float ceres_match_score = 0.0;
  transform::Rigid2d ceres_pose_estimate;
  ceres_scan_matcher_->Match(real_time_pose_estimate, constraint_data->points,
                             *probability_grid_, &ceres_pose_estimate,
                             &ceres_match_score);
  if (ceres_match_score < kMinTrackConstraintScore) {
    return;
  }

  // LOG(INFO) << "real_time_score = " << real_time_score
  //           << ", ceres_match_score = " << ceres_match_score;

  constraint_data->global_pose = ceres_pose_estimate;
  constraint_data->global_pose_score = ceres_match_score;

  const NodePtr new_node = std::make_shared<Node>();
  new_node->timestamp = constraint_data->timestamp;
  new_node->constraint_data = constraint_data;
  new_node->optimized_pose = ceres_pose_estimate;

  AddNode(new_node);

  // {
  //   static int count = 0;
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

  //   for (const Eigen::Vector3d& point : constraint_data->points) {
  //     const Eigen::Vector2d new_point = ceres_pose_estimate *
  //     point.head<2>(); const Eigen::Array2i index =
  //         map_limits.GetCellIndex(new_point.cast<float>());
  //     image.at<cv::Vec3b>(index.y(), index.x()) = cv::Vec3b(0, 0, 255);
  //   }

  //   cv::imwrite("/home/linjs/图片/global_match/match_" +
  //                   std::to_string(count++) + ".png",
  //               image);
  // }
}

void PoseGraph::ComputeGlobalConstraint(
    const ConstraintDataPtr& constraint_data) {
  CHECK_NOTNULL(fast_correlative_scan_matcher_);

  float fast_match_score = 0.f;
  transform::Rigid2d fast_pose_estimate;
  fast_correlative_scan_matcher_->MatchFullSubmap(
      constraint_data->points, kMinGlobalLocalizationScore, &fast_match_score,
      &fast_pose_estimate);

  float ceres_match_score = 0.0;
  transform::Rigid2d ceres_pose_estimate;
  ceres_scan_matcher_->Match(fast_pose_estimate, constraint_data->points,
                             *probability_grid_, &ceres_pose_estimate,
                             &ceres_match_score);
  LOG(INFO) << "fast_pose_estimate = [" << fast_pose_estimate.translation().x()
            << ", " << fast_pose_estimate.translation().y() << ", "
            << fast_pose_estimate.rotation().angle()
            << "], fast_match_score = " << fast_match_score
            << ", ceres_match_score = " << ceres_match_score;
  if (ceres_match_score < kMinGlobalLocalizationScore) {
    return;
  }

  constraint_data->global_pose = ceres_pose_estimate;
  constraint_data->global_pose_score = ceres_match_score;

  const NodePtr new_node = std::make_shared<Node>();
  new_node->timestamp = constraint_data->timestamp;
  new_node->constraint_data = constraint_data;
  new_node->optimized_pose = ceres_pose_estimate;

  AddNode(new_node);
}

void PoseGraph::ComputeLocalConstraint(
    const ConstraintDataPtr& constraint_data) {
  CHECK_NOTNULL(fast_correlative_scan_matcher_);

  float fast_match_score = 0.f;
  transform::Rigid2d fast_pose_estimate;
  fast_correlative_scan_matcher_->MatchLocalSubmap(
      constraint_data->initial_pose_estimate, constraint_data->points,
      kMinRelocalizationScore, &fast_match_score, &fast_pose_estimate);

  float ceres_match_score = 0.0;
  transform::Rigid2d ceres_pose_estimate;
  ceres_scan_matcher_->Match(fast_pose_estimate, constraint_data->points,
                             *probability_grid_, &ceres_pose_estimate,
                             &ceres_match_score);
  LOG(INFO) << "fast_pose_estimate = [" << fast_pose_estimate.translation().x()
            << ", " << fast_pose_estimate.translation().y() << ", "
            << fast_pose_estimate.rotation().angle()
            << "], fast_match_score = " << fast_match_score
            << ", ceres_match_score = " << ceres_match_score;
  if (ceres_match_score < kMinRelocalizationScore) {
    return;
  }

  constraint_data->global_pose = ceres_pose_estimate;
  constraint_data->global_pose_score = ceres_match_score;

  const NodePtr new_node = std::make_shared<Node>();
  new_node->timestamp = constraint_data->timestamp;
  new_node->constraint_data = constraint_data;
  new_node->optimized_pose = ceres_pose_estimate;

  AddNode(new_node);
}

void PoseGraph::TrimNodeBuffer() {
  while (node_buffer_.size() > kMaxNodeBufferLength) {
    node_buffer_.pop_front();
  }
}

void PoseGraph::GlobalOptimize() {
  // const auto t0 = std::chrono::steady_clock::now();

  std::vector<NodePtr> nodes;
  {
    std::unique_lock<std::mutex> lock(node_buffer_mutex_);
    TrimNodeBuffer();
    nodes = std::vector<NodePtr>(node_buffer_.begin(), node_buffer_.end());
  }

  const auto problem = std::make_shared<ceres::Problem>();

  // Add parameter
  std::unordered_map<NodePtr, PosePtr> node_pose6ds;
  for (const auto& node : nodes) {
    PosePtr p6d_ptr(new double[6], std::default_delete<double[]>());
    Eigen::Map<Eigen::Matrix<double, 6, 1>> p6d(p6d_ptr.get());
    p6d.setZero();
    p6d(2, 0) = node->optimized_pose.rotation().angle();
    p6d(3, 0) = node->optimized_pose.translation().x();
    p6d(4, 0) = node->optimized_pose.translation().y();

    node_pose6ds.insert(std::make_pair(node, p6d_ptr));
    problem->AddParameterBlock(p6d_ptr.get(), 6);
    problem->SetParameterization(p6d_ptr.get(), SE3SeprateLieAlgo::Create());
  }

  // Add inter-frame constraint
  for (std::size_t curr_idx = 1; curr_idx < nodes.size(); ++curr_idx) {
    const auto& curr_node = nodes.at(curr_idx);
    const auto& prev_node = nodes.at(curr_idx - 1);
    auto& curr_p6d_ptr = node_pose6ds.at(curr_node);
    auto& prev_p6d_ptr = node_pose6ds.at(prev_node);

    const transform::Rigid2d& prev_local_pose =
        prev_node->constraint_data->local_pose;
    const transform::Rigid2d& curr_local_pose =
        curr_node->constraint_data->local_pose;
    const transform::Rigid2d delta_transform =
        prev_local_pose.inverse() * curr_local_pose;

    const Eigen::Matrix<double, 6, 6> sqrt_info =
        Eigen::Matrix<double, 6, 1>(kInterFrameWeight.data()).asDiagonal() *
        curr_node->constraint_data->local_pose_score;
    const auto cost =
        RelativePoseCost::Create(ToMatrix4d(delta_transform), sqrt_info);
    problem->AddResidualBlock(cost, nullptr, prev_p6d_ptr.get(),
                              curr_p6d_ptr.get());
  }

  // Add fixed pose constraint
  for (const auto& node : nodes) {
    auto& p6d_ptr = node_pose6ds.at(node);
    const Eigen::Matrix<double, 6, 6> sqrt_info =
        Eigen::Matrix<double, 6, 1>(kFixedPoseWeiht.data()).asDiagonal() *
        node->constraint_data->global_pose_score;
    const auto cost = FixedPoseCost::Create(
        ToMatrix4d(node->constraint_data->global_pose), sqrt_info);
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
  for (const auto& [node, p6d_ptr] : node_pose6ds) {
    if (node == nullptr) {
      continue;
    }

    Eigen::Map<Eigen::Matrix<double, 6, 1>> se3(p6d_ptr.get());
    node->optimized_pose = transform::Rigid2d(
        Eigen::Vector2d(se3(3, 0), se3(4, 0)), Eigen::Rotation2Dd(se3(2, 0)));
  }

  {
    std::unique_lock<std::mutex> buffer_lock(optimized_node_mutex_);
    optimized_node_ = nodes.back();
    const transform::Rigid2d delta_pose =
        optimized_node_->optimized_pose.inverse() *
        optimized_node_->constraint_data->global_pose;

    // LOG(INFO) << "delta_pose = " << delta_pose.translation().norm()
    //           << ", angle = " << delta_pose.rotation().angle();
  }

  // const auto t1 = std::chrono::steady_clock::now();
  // LOG(INFO)
  //     << "Global optimization takes "
  //     << std::chrono::duration_cast<std::chrono::milliseconds>(t1 -
  //     t0).count()
  //     << "ms";
}

void PoseGraph::Init() {
  LOG(INFO) << "Pose graph initialization...";
  CHECK_NOTNULL(probability_grid_);

  ceres_scan_matcher_ = std::make_shared<CeresScanMatcher2D>();

  RealTimeCorrelativeScanMatcherOptions2D real_time_options;
  real_time_options.linear_search_window = 0.2;
  real_time_options.angular_search_window = 2.5 * kDegreeToRadian;
  real_time_options.translation_delta_cost_weight = 10;
  real_time_options.rotation_delta_cost_weight = 0.1;
  real_time_correlative_scan_matcher_ =
      std::make_shared<RealTimeCorrelativeScanMatcher2D>(real_time_options);

  FastCorrelativeScanMatcherOptions2D fast_options;
  fast_options.linear_search_window = 8.0;
  fast_options.angular_search_window = M_PI / 6.0;
  fast_options.branch_and_bound_depth = 7;

  const auto t0 = std::chrono::steady_clock::now();
  fast_correlative_scan_matcher_ =
      std::make_shared<FastCorrelativeScanMatcher2D>(*probability_grid_,
                                                     fast_options);
  const auto t1 = std::chrono::steady_clock::now();
  LOG(INFO)
      << "FastCorrelativeScanMatcher2D takes "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << "ms";

  pose_graph_running_ = true;
  optimization_thread_ = std::thread(&PoseGraph::OptimizationLoop, this);
  constraint_thread_ = std::thread(&PoseGraph::ConstraintLoop, this);
}

void PoseGraph::Reset() {
  Finish();

  node_interval_ = 0;
  pose_graph_running_ = true;
  optimization_thread_ = std::thread(&PoseGraph::OptimizationLoop, this);
  constraint_thread_ = std::thread(&PoseGraph::ConstraintLoop, this);
}

void PoseGraph::Finish() {
  pose_graph_running_ = false;
  optimization_condition_.notify_all();
  constraint_condition_.notify_all();

  if (optimization_thread_.joinable()) {
    optimization_thread_.join();
  }

  if (constraint_thread_.joinable()) {
    constraint_thread_.join();
  }

  constraint_data_buffer_.clear();
  caidate_node_buffer_.clear();
  node_buffer_.clear();
}

// 后台优化线程的主循环
void PoseGraph::OptimizationLoop() {
  while (pose_graph_running_) {
    std::unique_lock<std::mutex> lock(optimization_mutex_);

    // 让当前线程释放锁并进入休眠，直到Lambda表达式返回true时才被允许醒来并继续向下执行
    optimization_condition_.wait(lock, [this] {
      std::unique_lock<std::mutex> buffer_lock(caidate_node_buffer_mutex_);
      return !pose_graph_running_ || !caidate_node_buffer_.empty();
    });

    if (!pose_graph_running_) {
      break;
    }

    std::deque<NodePtr> temporary_caidate_node_buffer;
    {
      std::unique_lock<std::mutex> buffer_lock(caidate_node_buffer_mutex_);
      temporary_caidate_node_buffer = std::move(caidate_node_buffer_);
    }

    while (!temporary_caidate_node_buffer.empty()) {
      const auto candidate_node = temporary_caidate_node_buffer.front();
      temporary_caidate_node_buffer.pop_front();
      node_buffer_.emplace_back(candidate_node);
      ++node_interval_;
      if (candidate_node->constraint_data->constraint_type ==
              ConstraintType::kGlobal ||
          candidate_node->constraint_data->constraint_type ==
              ConstraintType::kLocal ||
          node_interval_ > kOptimizeNodeInterval) {
        // 执行全局优化

        GlobalOptimize();
        node_interval_ = 0;
      }
    }
  }
}

// 后台优化线程的主循环
void PoseGraph::ConstraintLoop() {
  while (pose_graph_running_) {
    std::unique_lock<std::mutex> lock(constraint_mutex_);

    // 让当前线程释放锁并进入休眠，直到Lambda表达式返回true时才被允许醒来并继续向下执行
    constraint_condition_.wait(lock, [this] {
      std::unique_lock<std::mutex> buffer_lock(constraint_data_buffer_mutex_);
      return !pose_graph_running_ || !constraint_data_buffer_.empty();
    });

    if (!pose_graph_running_) {
      break;
    }

    ConstraintDataPtr constraint_data = nullptr;
    {
      std::unique_lock<std::mutex> buffer_lock(constraint_data_buffer_mutex_);
      if (!constraint_data_buffer_.empty()) {
        constraint_data = constraint_data_buffer_.front();
        constraint_data_buffer_.pop_front();
      }
    }

    switch (constraint_data->constraint_type) {
      case ConstraintType::kTrack: {
        ComputeTrackConstraint(constraint_data);
        break;
      }
      case ConstraintType::kLocal: {
        ComputeLocalConstraint(constraint_data);
        break;
      }
      case ConstraintType::kGlobal: {
        ComputeGlobalConstraint(constraint_data);
        break;
      }
      default:
        break;
    }
  }
}

void PoseGraph::AddNode(const NodePtr node) {
  {
    std::unique_lock<std::mutex> lock(caidate_node_buffer_mutex_);
    caidate_node_buffer_.emplace_back(node);
  }

  // 唤醒后台优化线程
  optimization_condition_.notify_one();
}

void PoseGraph::AddTrackingConstraint(
    const double timestamp, const std::vector<Eigen::Vector3d>& points,
    const transform::Rigid2d& local_pose, float local_pose_score) {
  ConstraintDataPtr constraint_data = std::make_shared<ConstraintData>();
  constraint_data->timestamp = timestamp;
  constraint_data->constraint_type = ConstraintType::kTrack;
  constraint_data->points = points;
  constraint_data->local_pose = local_pose;
  constraint_data->local_pose_score = local_pose_score;

  {
    std::unique_lock<std::mutex> lock(constraint_data_buffer_mutex_);
    constraint_data_buffer_.emplace_back(constraint_data);
  }

  // 唤醒后台优化线程
  constraint_condition_.notify_one();
}

void PoseGraph::AddLocalMatchConstraint(
    const double timestamp, const std::vector<Eigen::Vector3d>& points,
    const transform::Rigid2d& initial_pose_estimate,
    const transform::Rigid2d& local_pose, float local_pose_score) {
  ConstraintDataPtr constraint_data = std::make_shared<ConstraintData>();
  constraint_data->timestamp = timestamp;
  constraint_data->constraint_type = ConstraintType::kLocal;
  constraint_data->initial_pose_estimate = initial_pose_estimate;
  constraint_data->points = points;
  constraint_data->local_pose = local_pose;
  constraint_data->local_pose_score = local_pose_score;

  {
    std::unique_lock<std::mutex> lock(constraint_data_buffer_mutex_);
    constraint_data_buffer_.emplace_back(constraint_data);
  }

  // 唤醒后台优化线程
  constraint_condition_.notify_one();
}

void PoseGraph::AddGlobalMatchConstraint(
    const double timestamp, const std::vector<Eigen::Vector3d>& points) {
  ConstraintDataPtr constraint_data = std::make_shared<ConstraintData>();
  constraint_data->timestamp = timestamp;
  constraint_data->constraint_type = ConstraintType::kGlobal;
  constraint_data->points = points;
  constraint_data->local_pose = transform::Rigid2d::Identity();
  constraint_data->local_pose_score = 1.0;

  {
    std::unique_lock<std::mutex> lock(constraint_data_buffer_mutex_);
    constraint_data_buffer_.emplace_back(constraint_data);
  }

  // 唤醒后台优化线程
  constraint_condition_.notify_one();
}

}  // namespace solex_robot::navigation::localization_2d