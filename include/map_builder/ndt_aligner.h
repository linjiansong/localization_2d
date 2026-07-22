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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <map>
#include <memory>
#include <vector>

#include "common/rigid_transform.h"

namespace solex_robot::navigation::localization_2d {
enum class NearbyType {
  kCenter = 0,
  kNeighbors = 1,
};

struct GridIndex {
  GridIndex() = default;
  GridIndex(int x_in, int y_in) : x(x_in), y(y_in) {}

  bool operator==(const GridIndex& other) const {
    return x == other.x && y == other.y;
  }

  bool operator<(const GridIndex& other) const {
    if (x != other.x) {
      return x < other.x;
    }
    return y < other.y;
  }

  GridIndex operator+(const GridIndex& other) const {
    return {x + other.x, y + other.y};
  }

  int x = 0;
  int y = 0;
};

struct Options {
  int max_iterations = 50;          // 最大迭代次数
  double voxel_size = 1.0;          // 体素大小
  double inverse_voxel_size = 1.0;  // 体素大小的逆 (1.0 / voxel_size)
  int min_effective_points = 10;    // 有效匹配点的最小数量阈值
  int min_points_per_voxel = 5;     // 每个栅格中容纳的最小点数
  int max_points_per_voxel = 50;    // 每个栅格中容纳的最大点数
  double convergence_eps = 1e-3;    // 收敛判定条件（两次迭代的步长阈值）
  double outlier_threshold = 5.0;   // 残差异常值拒绝阈值
  size_t voxel_capacity = 300;   // 哈希表缓存的体素最大数量
  NearbyType nearby_type = NearbyType::kNeighbors;
};

struct VoxelData {
  GridIndex grid_index;
  std::vector<Eigen::Vector2d> points;
  int total_points = 0;
  bool ndt_estimated = false;
  Eigen::Vector2d center;
  Eigen::Matrix2d covarance_matrix;
  Eigen::Matrix2d information_matrix;
};
using VoxelDataPtr = std::shared_ptr<VoxelData>;

class NDTAligner {
 public:
  NDTAligner() { GenerateNearbyGrids(); }

  void AddPointCloud(const std::vector<Eigen::Vector3d>& points);

  bool Align(const std::vector<Eigen::Vector3d>& point_cloud,
             const transform::Rigid2d& initial_pose,
             transform::Rigid2d* final_pose, double* score);

 private:
  void ComputeMeanAndCovarance(const std::vector<Eigen::Vector2d>& points,
                               Eigen::Vector2d* mean,
                               Eigen::Matrix2d* covarance_matrix) const;

  void UpdateMeanAndCovarance(int raw_size, int new_size,
                              const Eigen::Vector2d& raw_center,
                              const Eigen::Matrix2d& raw_covarance_matrix,
                              const Eigen::Vector2d& add_center,
                              const Eigen::Matrix2d& add_covarance_matrix,
                              Eigen::Vector2d* new_center,
                              Eigen::Matrix2d* new_covarance_matrix) const;

  void GenerateNearbyGrids();

  void UpdateVoxel(VoxelData& voxel_data);

  void UpdateVoxel(const VoxelDataPtr& voxel_data);

 private:
  const Options options_;

  // 真实数据，会缓存，也会清理
  std::list<VoxelDataPtr> voxel_data_list_;

  // 栅格数据，存储真实数据的迭代器
  std::map<GridIndex, VoxelDataPtr> voxel_data_map_;

  // 附近的栅格
  std::vector<GridIndex> nearby_grids_;

  // bool flag_first_scan_ = true;  // 首帧点云特殊处理
};

}  // namespace solex_robot::navigation::localization_2d