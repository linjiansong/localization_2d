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

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "Eigen/Core"
#include "common/base_type.h"
#include "common/rigid_transform.h"
#include "include/scan_match/ceres_scan_matcher_2d.h"
#include "include/scan_match/fast_correlative_scan_matcher_2d.h"
#include "include/scan_match/probability_grid.h"
#include "include/scan_match/real_time_correlative_scan_matcher_2d.h"

namespace solex_robot::navigation::localization_2d {

enum class ConstraintType { kUnknown = 0, kGlobal = 1, kTrack = 2 };

using PosePtr = std::shared_ptr<double[]>;

struct ConstraintData {
  double timestamp = 0.0;
  ConstraintType constraint_type = ConstraintType::kUnknown;
  std::vector<Eigen::Vector3d> points;
  transform::Rigid2d local_pose;  // scan to submap
  float local_pose_score;
  transform::Rigid2d global_pose;  // scan to submap
  float global_pose_score;
};
using ConstraintDataPtr = std::shared_ptr<ConstraintData>;

struct Node {
  double timestamp = 0.0;
  transform::Rigid2d optimized_pose;  // optimized_pose
  ConstraintDataPtr constraint_data;
};
using NodePtr = std::shared_ptr<Node>;

// using ConstraintCallback = std::function<void(
//     double time, transform::Rigid3d, std::unique_ptr<const
//     InsertionResult>)>;

// Align scans with an existing map using Ceres.
class PoseGraph {
 public:
  explicit PoseGraph(const std::shared_ptr<ProbabilityGrid> probability_grid)
      : probability_grid_(probability_grid) {}

  PoseGraph() = delete;

  void Init();
  void Finish();
  void Reset();

  void AddGlobalConstraint(const double timestamp,
                           const std::vector<Eigen::Vector3d>& points);
  void AddLocalConstraint(const double timestamp,
                          const std::vector<Eigen::Vector3d>& points,
                          const transform::Rigid2d& local_pose,
                          float local_pose_score);

  NodePtr optimized_node() {
    std::unique_lock<std::mutex> buffer_lock(optimized_node_mutex_);
    return optimized_node_;
  }

 private:
  void ComputeGlobalConstraint(const ConstraintDataPtr& constraint_data);
  void ComputeLocalConstraint(const ConstraintDataPtr& constraint_data);

  void AddNode(const NodePtr node);

  void TrimNodeBuffer();

  void MatchGlobalMap(const ConstraintDataPtr& constraint_data);
  void GlobalOptimize();

  void OptimizationLoop();
  void ConstraintLoop();

 private:
  std::shared_ptr<ProbabilityGrid> probability_grid_;
  std::shared_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
  std::shared_ptr<RealTimeCorrelativeScanMatcher2D>
      real_time_correlative_scan_matcher_;
  std::shared_ptr<FastCorrelativeScanMatcher2D> fast_correlative_scan_matcher_;

  std::atomic<bool> pose_graph_running_ = false;

  // 图优化线程相关成员
  std::deque<NodePtr> caidate_node_buffer_;
  std::mutex caidate_node_buffer_mutex_;

  std::deque<NodePtr> node_buffer_;
  std::mutex node_buffer_mutex_;
  int node_interval_ = 0;

  std::thread optimization_thread_;
  std::mutex optimization_mutex_;
  std::condition_variable optimization_condition_;

  // 约束计算线程相关成员
  std::deque<ConstraintDataPtr> constraint_data_buffer_;
  std::mutex constraint_data_buffer_mutex_;

  std::thread constraint_thread_;
  std::mutex constraint_mutex_;
  std::condition_variable constraint_condition_;

  // optimized result
  std::mutex optimized_node_mutex_;
  NodePtr optimized_node_;
};

}  // namespace solex_robot::navigation::localization_2d