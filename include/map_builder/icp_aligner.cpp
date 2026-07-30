// ////////////////////////////////////////////////////////////////////////////////
// // //
// //            Copyright© 2026 Solex Robot, All Rights Reserved. //
// // //
// //  All users are hereby notified that the materials in the form of digital
// //
// //  information available from this software (content, designs, color //
// //  schemes, graphic styles, images, logo, text, and videos) comes protected
// //
// //  under International Copyright Laws. Therefore it should not be reproduced
// //
// //  in any form digital or offline without prior written permission of //
// //  Solex Robot. //
// // //
// //  Any unauthorized reprint or material usage (Solex Robot) either manually
// //
// //  or digitally, is strictly prohibited. //
// // //
// //  Any further unauthorized digital copying of this material via copying, //
// //  publication, reproduction or distribution of copyrighted works is an //
// //  infringement of the copyright owners' rights may be the subject of the //
// //  copyright of performers' protection under the Copyright Act. For such //
// //  illegal activities you will be strictly liable to Solox Robot for any and
// //
// //  or all damages (including recovery of attorneys' fees) which may be //
// //  suffered and or incurred as a result of your infringement. //
// // //
// ////////////////////////////////////////////////////////////////////////////////

// #include "include/map_builder/icp_aligner.h"

// namespace solex_robot::navigation::localization_2d {

// namespace {
// constexpr double kRadianToDegree = 180.0 / M_PI;
// constexpr int kICPMaxIterations = 50;
// constexpr double kICPMaxInlierDistance = 10.0;  // meter
// constexpr double kICPMinInlierRatio = 0.5;
// constexpr double kICPMinDistance = 1e-4;  // meter
// constexpr double kICPMinAngle = 0.01;     // degree

// constexpr int kMaxPointCloudSize = 1;
// }  // namespace

// // SVD计算两个点云之间的最佳旋转平移矩阵(Kabsch 算法)
// transform::Rigid2d ICPAligner::ComputeTransformation2D(
//     const std::vector<Eigen::Vector2d>& source_points,
//     const std::vector<Eigen::Vector2d>& target_points) const {
//   // 1. 计算质心
//   Eigen::Vector2d source_mean = Eigen::Vector2d::Zero();
//   Eigen::Vector2d target_mean = Eigen::Vector2d::Zero();
//   for (size_t i = 0; i < source_points.size(); ++i) {
//     source_mean += source_points[i];
//     target_mean += target_points[i];
//   }
//   source_mean /= source_points.size();
//   target_mean /= target_points.size();

//   // 2. 去质心坐标
//   Eigen::MatrixXd new_source_points(2, source_points.size());
//   Eigen::MatrixXd new_target_points(2, target_points.size());
//   for (size_t i = 0; i < source_points.size(); ++i) {
//     new_source_points.col(i) = source_points[i] - source_mean;
//     new_target_points.col(i) = target_points[i] - target_mean;
//   }

//   // 3. 计算协方差矩阵 matrix_h
//   const Eigen::Matrix2d matrix_h =
//       new_source_points * new_target_points.transpose();

//   // 4. SVD分解
//   Eigen::JacobiSVD<Eigen::Matrix2d> svd(
//       matrix_h, Eigen::ComputeFullU | Eigen::ComputeFullV);
//   Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();

//   // 5. 处理反射情况(2D 中行列式必须为1，否则就是镜像)
//   if (rotation.determinant() < 0) {
//     Eigen::Matrix2d matrix_v = svd.matrixV();
//     matrix_v.col(1) *= -1;  // 翻转最后一列
//     rotation = matrix_v * svd.matrixU().transpose();
//   }

//   // 6. 计算平移
//   const Eigen::Vector2d translation = target_mean - rotation * source_mean;

//   return transform::Rigid2d(translation, Eigen::Rotation2Dd(rotation));
// }

// void ICPAligner::Align(const std::vector<Eigen::Vector3d>& point_cloud,
//                        const transform::Rigid2d& initial_pose,
//                        transform::Rigid2d* pose_estimate, double* score) {
//   *pose_estimate = initial_pose;
//   transform::Rigid2d prev_estimate = initial_pose;

//   int final_iter = 0;
//   std::vector<Eigen::Vector2d> source_points;
//   std::vector<Eigen::Vector2d> target_points;
//   for (int iter = 0; iter < kICPMaxIterations; ++iter) {
//     final_iter = iter;
//     source_points.clear();
//     target_points.clear();

//     source_points.reserve(point_cloud.size());
//     target_points.reserve(point_cloud.size());
//     for (const Eigen::Vector3d& point : point_cloud) {
//       const Eigen::Vector2d query_point = (*pose_estimate) * point.head(2);
//       std::vector<size_t> indices(1);
//       std::vector<double> sqr_distances(1,
//       std::numeric_limits<double>::max());
//       search_tree_->Query(query_point.data(), 1, indices.data(),
//                           sqr_distances.data());
//       const double distance = std::sqrt(sqr_distances[0]);
//       if (distance < kICPMaxInlierDistance) {
//         source_points.emplace_back(point.head(2));
//         target_points.emplace_back(search_tree_->get_data(indices[0]));
//       }
//     }

//     if (source_points.size() < kICPMinInlierRatio * point_cloud.size()) {
//       break;
//     }

//     *pose_estimate = ComputeTransformation2D(source_points, target_points);
//     const transform::Rigid2d delta_transform =
//         prev_estimate.inverse() * (*pose_estimate);
//     const double delta_distance = delta_transform.translation().norm();  //
//     meter const double delta_angle =
//         delta_transform.rotation().angle() * kRadianToDegree;
//     if (delta_distance < kICPMinDistance && delta_angle < kICPMinAngle) {
//       // LOG(INFO) << "delta_distance = " << delta_distance
//       //           << ", delta_angle = " << delta_angle << ", iter = " <<
//       iter; break;
//     }

//     prev_estimate = *pose_estimate;
//   }
// }

// void ICPAligner::AddPointCloud(const std::vector<Eigen::Vector3d>&
// point_cloud) {
//   if (point_cloud_list_.size() > kMaxPointCloudSize) {
//     point_cloud_list_.pop_front();
//   }

//   point_cloud_list_.emplace_back(point_cloud);

//   std::vector<Eigen::Vector2d> points_2d;
//   for (const auto& point_cloud : point_cloud_list_) {
//     for (const Eigen::Vector3d& point : point_cloud) {
//       points_2d.emplace_back(point.head(2));
//     }
//   }

//   if (!points_2d.empty()) {
//     search_tree_ = std::make_unique<KDTree2D>(2, points_2d);
//   }
// }

// }  // namespace solex_robot::navigation::localization_2d

// //////////////////////////////////////////////////////////////////////////////
// // //
// //            Copyright© 2026 Solex Robot, All Rights Reserved. //
// // //
// //  All users are hereby notified that the materials in the form of digital
// //
// //  information available from this software (content, designs, color //
// //  schemes, graphic styles, images, logo, text, and videos) comes protected
// //
// //  under International Copyright Laws. Therefore it should not be reproduced
// //
// //  in any form digital or offline without prior written permission of //
// //  Solex Robot. //
// // //
// //  Any unauthorized reprint or material usage (Solex Robot) either manually
// //
// //  or digitally, is strictly prohibited. //
// // //
// //  Any further unauthorized digital copying of this material via copying, //
// //  publication, reproduction or distribution of copyrighted works is an //
// //  infringement of the copyright owners' rights may be the subject of the //
// //  copyright of performers' protection under the Copyright Act. For such //
// //  illegal activities you will be strictly liable to Solex Robot for any and
// //
// //  or all damages (including recovery of attorneys' fees) which may be //
// //  suffered and or incurred as a result of your infringement. //
// // //
// //////////////////////////////////////////////////////////////////////////////

#include "include/map_builder/icp_aligner.h"

#include <opencv2/opencv.hpp>

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kRadianToDegree = 180.0 / M_PI;
constexpr int kICPMaxIterations = 50;
constexpr double kICPMaxInlierDistance = 1.0;  // meter
constexpr double kICPMinInlierRatio = 0.3;
constexpr double kICPMinDistance = 1.e-4;  // meter
constexpr double kICPMinAngle = 0.01;      // degree

constexpr int kMaxPointCloudSize = 1;
}  // namespace

// transform::Rigid2d ICPAligner::ComputeTransformation2D(
//     const std::vector<Eigen::Vector2d>& source_points,
//     const std::vector<Eigen::Vector2d>& target_points) const {
//   if (source_points.empty() || target_points.empty() ||
//       source_points.size() != target_points.size()) {
//     return transform::Rigid2d::Identity();
//   }

//   // 1. 初始化优化变量：平移 (tx, ty) 和 旋转角度 (yaw)
//   double translation_array[2] = {0.0, 0.0};
//   double yaw_angle = 0.0;

//   // 如果有上一轮的粗略估计或者可以先通过质心对齐给个初始平移
//   Eigen::Vector2d source_mean = Eigen::Vector2d::Zero();
//   Eigen::Vector2d target_mean = Eigen::Vector2d::Zero();
//   for (size_t i = 0; i < source_points.size(); ++i) {
//     source_mean += source_points[i];
//     target_mean += target_points[i];
//   }
//   source_mean /= source_points.size();
//   target_mean /= source_points.size();

//   // 用质心差作为平移的初始猜测，有助于加速收敛
//   Eigen::Vector2d init_trans = target_mean - source_mean;
//   translation_array[0] = init_trans.x();
//   translation_array[1] = init_trans.y();

//   // 2. 构建 Ceres 优化问题
//   ceres::Problem problem;

//   // 可选：引入 Huber 鲁棒核函数，防止个别错配点拉偏整体位姿
//   ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);

//   for (size_t i = 0; i < source_points.size(); ++i) {
//     ceres::CostFunction* cost_function =
//         ICP2DCostFunction::Create(source_points[i], target_points[i]);

//     problem.AddResidualBlock(cost_function, loss_function, translation_array,
//                              &yaw_angle);
//   }

//   // 3. 配置 Ceres 求解器参数
//   ceres::Solver::Options options;
//   options.linear_solver_type = ceres::DENSE_QR;
//   options.max_num_iterations = 30;
//   options.minimizer_progress_to_stdout = false;
//   options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

//   // 4. 执行优化
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);

//   // 5. 将优化结果转换为 transform::Rigid2d
//   Eigen::Vector2d final_translation(translation_array[0],
//   translation_array[1]); Eigen::Rotation2Dd final_rotation(yaw_angle);

//   return transform::Rigid2d(final_translation, final_rotation);
// }

// SVD计算两个点云之间的最佳旋转平移矩阵(Kabsch 算法)
transform::Rigid2d ICPAligner::ComputeTransformation2D(
    const std::vector<Eigen::Vector2d>& source_points,
    const std::vector<Eigen::Vector2d>& target_points) const {
  if (source_points.empty() || target_points.empty() ||
      source_points.size() != target_points.size()) {
    return transform::Rigid2d::Identity();
  }

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

  // 4. SVD分解
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      matrix_h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();

  // 5. 处理反射情况(2D 中行列式必须为1，否则就是镜像)
  if (rotation.determinant() < 0) {
    Eigen::Matrix2d matrix_v = svd.matrixV();
    matrix_v.col(1) *= -1;  // 翻转最后一列
    rotation = matrix_v * svd.matrixU().transpose();
  }

  // 6. 计算平移
  const Eigen::Vector2d translation = target_mean - rotation * source_mean;

  return transform::Rigid2d(translation, Eigen::Rotation2Dd(rotation));
}

void ICPAligner::Align(const std::vector<Eigen::Vector3d>& point_cloud,
                       const transform::Rigid2d& initial_pose,
                       transform::Rigid2d* pose_estimate, float* score) {
  CHECK_NOTNULL(pose_estimate);
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(search_tree_);

  if (point_cloud.empty()) {
    *pose_estimate = initial_pose;
    *score = 0.0;
    return;
  }

  *pose_estimate = initial_pose;
  transform::Rigid2d current_pose = initial_pose;

  std::vector<Eigen::Vector2d> source_points;
  std::vector<Eigen::Vector2d> target_points;

  for (int iter = 0; iter < kICPMaxIterations; ++iter) {
    source_points.clear();
    target_points.clear();

    source_points.reserve(point_cloud.size());
    target_points.reserve(point_cloud.size());
    // 核心修改：利用当前的 pose_estimate 将雷达点投影到全局坐标系下，再找最近邻
    for (const Eigen::Vector3d& point : point_cloud) {
      const Eigen::Vector2d query_point = current_pose * point.head(2);
      std::vector<size_t> indices(1);
      std::vector<double> sqr_distances(1, std::numeric_limits<double>::max());

      search_tree_->Query(query_point.data(), 1, indices.data(),
                          sqr_distances.data());
      const double distance = std::sqrt(sqr_distances[0]);
      if (distance < kICPMaxInlierDistance) {
        // source 记录局部原始点，target 记录对应全局地图中的点
        source_points.emplace_back(query_point);
        target_points.emplace_back(search_tree_->get_data(indices[0]));
      }
    }

    if (source_points.size() < kICPMinInlierRatio * point_cloud.size()) {
      break;
    }

    // 计算当前局部的增量变换 (Delta Transform)
    transform::Rigid2d delta_transform =
        ComputeTransformation2D(source_points, target_points);

    // 累加到整体估计位姿上: pose = pose * delta
    current_pose = delta_transform * current_pose;

    const double delta_distance =
        delta_transform.translation().norm();  // meter
    const double delta_angle =
        std::abs(delta_transform.rotation().angle() * kRadianToDegree);

    if (delta_distance < kICPMinDistance && delta_angle < kICPMinAngle) {
      break;
    }
  }

  *pose_estimate = current_pose;
  if (score) {
    // 可选：根据最终匹配上的内点比例计算一个简单的匹配得分
    *score = static_cast<double>(source_points.size()) / point_cloud.size();
  }

  // {
  //   double max_x = std::numeric_limits<double>::lowest();
  //   double max_y = std::numeric_limits<double>::lowest();
  //   double min_x = std::numeric_limits<double>::max();
  //   double min_y = std::numeric_limits<double>::max();

  //   for (const auto& cloud : point_cloud_list_) {
  //     for (const Eigen::Vector3d& point : cloud) {
  //       max_x = std::max(point.x(), max_x);
  //       max_y = std::max(point.y(), max_y);
  //       min_x = std::min(point.x(), min_x);
  //       min_y = std::min(point.y(), min_y);
  //     }
  //   }

  //   for (const Eigen::Vector3d& point : point_cloud) {
  //     const Eigen::Vector2d query_point = initial_pose * point.head(2);
  //     max_x = std::max(query_point.x(), max_x);
  //     max_y = std::max(query_point.y(), max_y);
  //     min_x = std::min(query_point.x(), min_x);
  //     min_y = std::min(query_point.y(), min_y);
  //   }

  //   for (const Eigen::Vector3d& point : point_cloud) {
  //     const Eigen::Vector2d query_point = (*pose_estimate) * point.head(2);
  //     max_x = std::max(query_point.x(), max_x);
  //     max_y = std::max(query_point.y(), max_y);
  //     min_x = std::min(query_point.x(), min_x);
  //     min_y = std::min(query_point.y(), min_y);
  //   }

  //   const double resolution = 0.02;

  //   const int width = (max_x - min_x) / resolution;
  //   const int height = (max_y - min_y) / resolution;

  //   cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
  //   for (const auto& cloud : point_cloud_list_) {
  //     for (const Eigen::Vector3d& point : cloud) {
  //       int x_index = std::clamp(
  //           static_cast<int>((point.x() - min_x) / resolution), 0, width -
  //           1);
  //       int y_index = std::clamp(
  //           static_cast<int>((point.y() - min_y) / resolution), 0, height -
  //           1);
  //       image.at<cv::Vec3b>(y_index, x_index) = cv::Vec3b(0, 0, 255);
  //     }
  //   }

  //   for (const Eigen::Vector3d& point : point_cloud) {
  //     const Eigen::Vector2d query_point = initial_pose * point.head(2);
  //     int x_index =
  //         std::clamp(static_cast<int>((query_point.x() - min_x) /
  //         resolution),
  //                    0, width - 1);
  //     int y_index =
  //         std::clamp(static_cast<int>((query_point.y() - min_y) /
  //         resolution),
  //                    0, height - 1);
  //     image.at<cv::Vec3b>(y_index, x_index) = cv::Vec3b(0, 255, 0);
  //   }

  //   for (const Eigen::Vector3d& point : point_cloud) {
  //     const Eigen::Vector2d query_point = (*pose_estimate) * point.head(2);
  //     int x_index =
  //         std::clamp(static_cast<int>((query_point.x() - min_x) /
  //         resolution),
  //                    0, width - 1);
  //     int y_index =
  //         std::clamp(static_cast<int>((query_point.y() - min_y) /
  //         resolution),
  //                    0, height - 1);
  //     image.at<cv::Vec3b>(y_index, x_index) = cv::Vec3b(255, 0, 0);
  //   }

  //   cv::imwrite("/home/linjs/图片/match/match_" + std::to_string(count_++) +
  //   ".png",
  //               image);
  // }
}

void ICPAligner::AddPointCloud(
    const std::vector<Eigen::Vector3d>& point_cloud) {
  if (point_cloud.empty()) {
    return;
  }

  if (point_cloud_list_.size() >= kMaxPointCloudSize) {
    point_cloud_list_.pop_front();
  }

  point_cloud_list_.emplace_back(point_cloud);

  // 重新收集所有历史子图的点云构建全局 KDTree 检索树
  std::vector<Eigen::Vector2d> points_2d;
  size_t total_points = 0;
  for (const auto& cloud : point_cloud_list_) {
    total_points += cloud.size();
  }
  points_2d.reserve(total_points);

  for (const auto& cloud : point_cloud_list_) {
    for (const Eigen::Vector3d& point : cloud) {
      if (!point.allFinite()) {
        LOG(INFO) << "point = " << point.transpose();
      }

      points_2d.emplace_back(point.head(2));
    }
  }

  if (!points_2d.empty()) {
    search_tree_ = std::make_unique<KDTree2D>(2, points_2d);
  }
}

}  // namespace solex_robot::navigation::localization_2d