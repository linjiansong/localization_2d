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

#include "include/scan_match/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "include/scan_match/occupied_space_cost_function_2d.h"

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kOccupiedSpaceWeight = 10.0;
constexpr double kTranslationWeight = 10.0;
constexpr double kRotationWeight = 40.0;
}  // namespace

void CeresScanMatcher2D::Match(const transform::Rigid2d& initial_pose_estimate,
                               const std::vector<Eigen::Vector3d>& point_cloud,
                               const ProbabilityGrid& probability_grid,
                               transform::Rigid2d* const pose_estimate,
                               float* const score) {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};

  ceres::Problem problem;
  CHECK_GT(kOccupiedSpaceWeight, 0.);
  CHECK_GT(kTranslationWeight, 0.);
  CHECK_GT(kRotationWeight, 0.);

  switch (probability_grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      problem.AddResidualBlock(
          OccupiedSpaceCostFunction2D::Create(
              kOccupiedSpaceWeight /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, probability_grid),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
      // case GridType::TSDF:
      //   problem.AddResidualBlock(
      //       CreateTSDFMatchCostFunction2D(
      //           kOccupiedSpaceWeight/
      //               std::sqrt(static_cast<double>(point_cloud.size())),
      //           point_cloud, static_cast<const TSDF2D&>(grid)),
      //       nullptr /* loss function */, ceres_pose_estimate);
      //   break;
  }

  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::Create(
          kTranslationWeight, initial_pose_estimate.translation()),
      nullptr /* loss function */, ceres_pose_estimate);
  problem.AddResidualBlock(RotationDeltaCostFunctor2D::Create(
                               kRotationWeight, ceres_pose_estimate[2]),
                           nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = false;
  options.max_num_iterations = 50;
  options.num_threads = 1;

  options.gradient_tolerance = 1.e-6;   // 梯度极小时停止，避免无效迭代
  options.function_tolerance = 1.e-6;   // 代价函数变化极小时停止
  options.parameter_tolerance = 1.e-6;  // 参数变化极小时停止

  // 3. 线性求解器配置 (对于2D问题，DenseQR最快且最稳定)
  options.linear_solver_type = ceres::DENSE_QR;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // LOG(INFO) << summary.BriefReport();

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);

  // 计算最终的匹配得分
  {
    const MapLimits& map_limits = probability_grid.map_limits();

    *score = 0.0;
    for (const Eigen::Vector3d& point : point_cloud) {
      const Eigen::Vector2d world_point = (*pose_estimate) * point.head(2);

      const Eigen::Array2i proposed_xy_index =
          map_limits.GetCellIndex(world_point.cast<float>());
      const float probability =
          probability_grid.GetProbability(proposed_xy_index);
      *score += probability;
    }

    *score /= static_cast<float>(point_cloud.size());
  }
}

}  // namespace solex_robot::navigation::localization_2d