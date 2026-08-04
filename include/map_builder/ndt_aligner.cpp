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

#include "include/map_builder/ndt_aligner.h"

#include <glog/logging.h>

#include <cmath>
#include <execution>
#include <set>

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kEpsilon = 1.e-6;
GridIndex ConvertToGridIndex(const Eigen::Vector2d& point) {
  return GridIndex(std::floor(point.x()), std::floor(point.y()));
}
}  // namespace

void NDTAligner::ComputeMeanAndCovarance(
    const std::vector<Eigen::Vector2d>& points, Eigen::Vector2d* mean,
    Eigen::Matrix2d* covarance_matrix) const {
  assert(points.size() > 1);

  *mean = std::accumulate(points.begin(), points.end(),
                          Eigen::Vector2d::Zero().eval(),
                          [](const Eigen::Vector2d& sum,
                             const Eigen::Vector2d& point) -> Eigen::Vector2d {
                            return sum + point;
                          }) /
          points.size();

  *covarance_matrix =
      std::accumulate(points.begin(), points.end(),
                      Eigen::Matrix2d::Zero().eval(),
                      [&mean](const Eigen::Matrix2d& sum,
                              const Eigen::Vector2d& point) -> Eigen::Matrix2d {
                        const Eigen::Vector2d diff = point - (*mean);
                        return sum + diff * diff.transpose();
                      }) /
      (points.size() - 1);
}

void NDTAligner::UpdateMeanAndCovarance(
    int raw_size, int new_size, const Eigen::Vector2d& raw_center,
    const Eigen::Matrix2d& raw_covarance_matrix,
    const Eigen::Vector2d& add_center,
    const Eigen::Matrix2d& add_covarance_matrix, Eigen::Vector2d* new_center,
    Eigen::Matrix2d* new_covarance_matrix) const {
  assert(raw_size > 0);
  assert(new_size > 0);
  *new_center =
      (raw_size * raw_center + new_size * add_center) / (raw_size + new_size);

  const Eigen::Vector2d raw_diff = raw_center - (*new_center);
  const Eigen::Vector2d add_diff = add_center - (*new_center);

  *new_covarance_matrix =
      (raw_size * (raw_covarance_matrix + raw_diff * raw_diff.transpose()) +
       new_size * (add_covarance_matrix + add_diff * add_diff.transpose())) /
      (raw_size + new_size);
}

void NDTAligner::GenerateNearbyGrids() {
  if (options_.nearby_type == NearbyType::kCenter) {
    nearby_grids_.emplace_back(GridIndex(0, 0));
  } else if (options_.nearby_type == NearbyType::kNeighbors) {
    nearby_grids_ = {GridIndex(0, 0), GridIndex(0, 1), GridIndex(0, -1),
                     GridIndex(-1, 0), GridIndex(1, 0)};
  }
}

void NDTAligner::AddPointCloud(const std::vector<Eigen::Vector3d>& points) {
  std::set<GridIndex> active_voxels;
  for (const Eigen::Vector3d& point : points) {
    const GridIndex grid_index =
        ConvertToGridIndex(point.head<2>() * options_.inverse_voxel_size);
    auto voxel_data_iter = voxel_data_map_.find(grid_index);
    if (voxel_data_iter == voxel_data_map_.end()) {
      // 栅格不存在：在链表头部插入新体素，并带上当前第一个点
      VoxelDataPtr voxel_data = std::make_shared<VoxelData>();
      voxel_data->points.emplace_back(point.head<2>());
      voxel_data->grid_index = grid_index;
      voxel_data_list_.push_front(voxel_data);
      voxel_data_map_.insert(std::make_pair(grid_index, voxel_data));

      // 检查容量是否溢出
      if (voxel_data_list_.size() >= options_.voxel_capacity) {
        // 删除哈希表中的尾部数据对应的索引
        voxel_data_map_.erase(voxel_data_list_.back()->grid_index);
        voxel_data_list_.pop_back();
      }
    } else {
      // 栅格存在：添加点到该体素中
      voxel_data_iter->second->points.emplace_back(point.head<2>());

      // 将当前体素节点移动到链表最前方(表示最近刚被使用过)
      voxel_data_list_.remove(voxel_data_iter->second);
      voxel_data_list_.push_front(voxel_data_iter->second);
    }

    // 记录这个体素在这一帧被更新了
    active_voxels.insert(grid_index);
  }

  // 并行更新高斯分布
  std::for_each(std::execution::par_unseq, active_voxels.begin(),
                active_voxels.end(), [this](const auto& active_voxel) {
                  UpdateVoxel(voxel_data_map_[active_voxel]);
                });
}

void NDTAligner::UpdateVoxel(const VoxelDataPtr& voxel_data) {
  // 处理第一帧
  // if (flag_first_scan_) {
  //   if (voxel_data->points.size() > 1) {
  //     math::ComputeMeanAndCovarance(
  //         voxel_data->points, &(voxel_data->center),
  //         &(voxel_data->covarance_matrix));
  //     voxel_data->information_matrix =
  //         (voxel_data->covarance_matrix + Eigen::Matrix3d::Identity() * 1e-3)
  //             .inverse();
  //   } else {
  //     // 如果点数只有1个，无法构成协方差，则给一个极大的初始信息矩阵
  //     voxel_data->center = voxel_data->points[0];
  //     voxel_data->information_matrix = Eigen::Matrix3d::Identity() * 1e2;
  //   }

  //   voxel_data->ndt_estimated = true;
  //   // 清空暂存点
  //   voxel_data->points.clear();
  //   return;
  // }

  // 如果该体素已经完成了高斯估计，并且历史累计总点数已经超过了配置的最大阈值，说明这个体素的几何特征已经足够饱满和稳定了。
  if (voxel_data->ndt_estimated &&
      voxel_data->total_points > options_.max_points_per_voxel) {
    voxel_data->points.clear();
    return;
  }

  // 新体素首次估计
  if (!voxel_data->ndt_estimated &&
      voxel_data->points.size() > options_.min_points_per_voxel) {
    // 如果某个体素之前还没建立高斯分布，但随着时间推移，新帧往里面塞了足够多的点，触发首次高斯建模
    ComputeMeanAndCovarance(voxel_data->points, &(voxel_data->center),
                            &(voxel_data->covarance_matrix));
    voxel_data->information_matrix =
        (voxel_data->covarance_matrix + Eigen::Matrix2d::Identity() * kEpsilon)
            .inverse();
    voxel_data->ndt_estimated = true;
    voxel_data->total_points = voxel_data->points.size();
    voxel_data->points.clear();
  } else if (voxel_data->ndt_estimated &&
             voxel_data->points.size() > options_.min_points_per_voxel) {
    // 当体素已经有历史高斯分布，且新帧又扫到了它，需要把新点云融合进旧的高斯分布中

    // 计算当前新来这一小批点云的临时均值和协方差
    Eigen::Vector2d curr_center;
    Eigen::Matrix2d curr_covariance_matrix;
    ComputeMeanAndCovarance(voxel_data->points, &curr_center,
                            &curr_covariance_matrix);

    // 增量融合公式：将旧的统计量和新的统计量合并，算出新均值和新协方差
    Eigen::Vector2d new_center;
    Eigen::Matrix2d new_covarance_matrix;
    UpdateMeanAndCovarance(voxel_data->total_points, voxel_data->points.size(),
                           voxel_data->center, voxel_data->covarance_matrix,
                           curr_center, curr_covariance_matrix, &new_center,
                           &new_covarance_matrix);

    voxel_data->center = new_center;
    voxel_data->covarance_matrix = new_covarance_matrix;
    voxel_data->total_points += voxel_data->points.size();
    voxel_data->points.clear();

    // 防退化与正则化处理
    Eigen::JacobiSVD svd(voxel_data->covarance_matrix,
                         Eigen::ComputeFullU | Eigen::ComputeFullV);

    // 获取特征值
    Eigen::Vector2d lambda = svd.singularValues();

    // 限制最小特征值，防止为0或负数（防止退化）
    if (lambda.y() < lambda.x() * kEpsilon) {
      lambda.y() = lambda.x() * kEpsilon;
    }

    const Eigen::Matrix2d inverse_lambda =
        Eigen::Vector2d(1.0 / lambda.x(), 1.0 / lambda.y()).asDiagonal();
    voxel_data->information_matrix =
        svd.matrixV() * inverse_lambda * svd.matrixU().transpose();
  }
}

bool NDTAligner::Align(const std::vector<Eigen::Vector3d>& point_cloud,
                       const transform::Rigid2d& initial_pose,
                       transform::Rigid2d* pose_estimate, double* score) {
  LOG(INFO) << "aligning with inc ndt, pts: " << point_cloud.size()
            << ", grids: " << voxel_data_map_.size();
  assert(voxel_data_map_.empty() == false);

  std::vector<int> indices(point_cloud.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    indices[i] = i;
  }

  *pose_estimate = initial_pose;
  const int total_size = indices.size() * nearby_grids_.size();
  for (int iter = 0; iter < options_.max_iterations; ++iter) {
    std::vector<bool> effect_point_mask(total_size, false);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(total_size);
    std::vector<Eigen::Vector2d> errors(total_size);
    std::vector<Eigen::Matrix2d> information_matrixs(total_size);

    // 提前缓存当前迭代的旋转矩阵，避免在并行循环中重复计算
    const double cos_theta = std::cos(pose_estimate->rotation().angle());
    const double sin_theta = std::sin(pose_estimate->rotation().angle());

    // gauss-newton迭代(最近邻，可以并发)
    std::for_each(
        std::execution::par_unseq, indices.begin(), indices.end(),
        [&](int idx) {
          const Eigen::Vector2d source_point = point_cloud[idx].head<2>();
          const Eigen::Vector2d transformed_point =
              (*pose_estimate) * source_point;

          // 计算transformed_point所在的栅格以及它的最近邻栅格
          const GridIndex grid_index = ConvertToGridIndex(
              transformed_point * options_.inverse_voxel_size);
          for (std::size_t nearby_grid_index = 0;
               nearby_grid_index < nearby_grids_.size(); ++nearby_grid_index) {
            const GridIndex neighbor_index =
                grid_index + nearby_grids_[nearby_grid_index];
            auto iter = voxel_data_map_.find(neighbor_index);
            const int final_index =
                idx * nearby_grids_.size() + nearby_grid_index;
            /// 这里要检查高斯分布是否已经估计
            if (iter != voxel_data_map_.end() && iter->second->ndt_estimated) {
              const VoxelDataPtr& voxel_data = iter->second;  // voxel
              // 残差 e = 变换后的点 - 高斯分布中心
              const Eigen::Vector2d error =
                  transformed_point - voxel_data->center;

              // 卡方检验（Chi-square Test）进行异常值剔除
              const double chi2 =
                  error.transpose() * voxel_data->information_matrix * error;
              if (std::isnan(chi2) || chi2 > options_.outlier_threshold) {
                effect_point_mask[final_index] = false;
                continue;
              }

              // build residual(对dx, dy, dθ的偏导)
              Eigen::Matrix<double, 2, 3> jacobian_matrix;
              jacobian_matrix.setZero();
              // 平移对状态的导数
              jacobian_matrix.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();

              jacobian_matrix(0, 2) =
                  -sin_theta * source_point.x() - cos_theta * source_point.y();
              jacobian_matrix(1, 2) =
                  cos_theta * source_point.x() - sin_theta * source_point.y();

              jacobians[final_index] = jacobian_matrix;
              errors[final_index] = error;
              information_matrixs[final_index] = voxel_data->information_matrix;
              effect_point_mask[final_index] = true;
            } else {
              effect_point_mask[final_index] = false;
            }
          }
        });

    // 累加Hessian和error,计算dx
    double total_res = 0.0;
    int effective_num = 0;

    // 2D状态量为3维(dx, dy, dθ)，因此Hessian为3x3，误差梯度向量为3x1
    Eigen::Matrix<double, 3, 3> hessian_matrix =
        Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, 3, 1> error_vector =
        Eigen::Matrix<double, 3, 1>::Zero();

    for (std::size_t idx = 0; idx < effect_point_mask.size(); ++idx) {
      if (!effect_point_mask[idx]) {
        continue;
      }

      total_res +=
          errors[idx].transpose() * information_matrixs[idx] * errors[idx];
      ++effective_num;

      // 累加2D的Hessian和梯度项(高斯牛顿公式)
      hessian_matrix += jacobians[idx].transpose() * information_matrixs[idx] *
                        jacobians[idx];
      error_vector +=
          -jacobians[idx].transpose() * information_matrixs[idx] * errors[idx];
    }

    if (effective_num < options_.min_effective_points) {
      LOG(WARNING) << "effective num too small: " << effective_num;
      return false;
    }

    // 使用更稳健的LDLT分解求解3x3线性方程组
    Eigen::LDLT<Eigen::Matrix<double, 3, 3>> ldlt_solver(hessian_matrix);
    if (ldlt_solver.info() != Eigen::Success) {
      LOG(WARNING) << "Hessian matrix LDLT decomposition failed!";
      return false;
    }

    // H * delta = -g
    const Eigen::Vector3d delta = ldlt_solver.solve(error_vector);

    // 更新2D位姿 (前2维为平移增量，第3维为旋转角度增量)
    *pose_estimate = transform::Rigid2d(
        Eigen::Vector2d(pose_estimate->translation().x() + delta.x(),
                        pose_estimate->translation().y() + delta.y()),
        pose_estimate->rotation() * Eigen::Rotation2Dd(delta.z()));

    // 更新
    // LOG(INFO) << "iter " << iter << " total res: " << total_res
    //           << ", eff: " << effective_num
    //           << ", center res: " << total_res / effective_num
    //           << ", delta.norm: " << delta.norm()
    //           << ", delta: " << delta.transpose();

    if (delta.norm() < options_.convergence_eps) {
      // LOG(INFO) << "converged, dx = " << delta.transpose();
      break;
    }
  }

  return true;
}

}  // namespace solex_robot::navigation::localization_2d