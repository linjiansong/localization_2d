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
#include <deque>
#include <memory>
#include <vector>

#include "ceres/ceres.h"
#include "common/kdtree_adaptor.h"
#include "common/rigid_transform.h"

namespace solex_robot::navigation::localization_2d {

struct ICP2DCostFunction {
  ICP2DCostFunction(const Eigen::Vector2d& source_point, const Eigen::Vector2d& target_point)
      : source_point_(source_point), target_point_(target_point) {}

  template <typename T>
  bool operator()(const T* const translation, const T* const yaw, T* residual) const {
    // 1. 将常规的 double 坐标显式转换为模板类型 T (支持 ceres::Jet)
    T px = T(source_point_.x());
    T py = T(source_point_.y());
    T tx = translation[0];
    T ty = translation[1];
    T theta = yaw[0];

    // 2. 构造 2D 旋转矩阵
    T cos_theta = ceres::cos(theta);
    T sin_theta = ceres::sin(theta);

    // 3. 将 source_point 变换到目标坐标系下：p_trans = R * p_src + t
    T transformed_x = cos_theta * px - sin_theta * py + tx;
    T transformed_y = sin_theta * px + cos_theta * py + ty;

    // 4. 计算与 target_point 的残差（同样需要将 target_point 转为 T）
    residual[0] = transformed_x - T(target_point_.x());
    residual[1] = transformed_y - T(target_point_.y());

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d& source_point,
                                     const Eigen::Vector2d& target_point) {
    return new ceres::AutoDiffCostFunction<ICP2DCostFunction, 2, 2, 1>(
        new ICP2DCostFunction(source_point, target_point));
  }

  const Eigen::Vector2d source_point_;
  const Eigen::Vector2d target_point_;
};

class ICPAligner {
 public:
  ICPAligner() = default;
  ~ICPAligner() = default;

  void AddPointCloud(const std::vector<Eigen::Vector3d>& points);

  void Align(const std::vector<Eigen::Vector3d>& point_cloud,
             const transform::Rigid2d& initial_pose,
             transform::Rigid2d* final_pose, double* score);

 private:
  transform::Rigid2d ComputeTransformation2D(
      const std::vector<Eigen::Vector2d>& source_points,
      const std::vector<Eigen::Vector2d>& target_points) const;

 private:
  std::shared_ptr<KDTree2D> search_tree_;
  std::deque<std::vector<Eigen::Vector3d>> point_cloud_list_;
};

}  // namespace solex_robot::navigation::localization_2d