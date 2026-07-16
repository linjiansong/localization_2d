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

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <vector>

namespace solex_robot::navigation::localization_2d {
namespace {
// constexpr float kMinProbability = 0.1f;
constexpr float kMinProbability = 0.0f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;
constexpr float kUnknownCorrespondenceValue = 0.5;

constexpr int kValueCount = 32768;
constexpr int kUnknownProbabilityValue = 0;
constexpr int kPadding = INT_MAX / 4;

constexpr int kRepetitionCount = 2;
constexpr double kSafetyMargin = 1. - 1e-3;
}  // namespace

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

struct FastCorrelativeScanMatcherOptions2D {
  // 线性搜索窗口的大小（以米为单位）。
  // 这个参数决定了在多大的范围内进行平移搜索。
  double linear_search_window = 0.0;

  // 角度搜索窗口的大小（以弧度为单位）。
  // 这个参数决定了在多大的范围内进行旋转搜索。
  double angular_search_window = 0.0;

  // 分支定界算法（Branch-and-Bound）所使用的查找表层数。
  // Cartographer 会为地图构建多分辨率的查找表（类似图像金字塔），
  // 该值决定了层级深度。层数越多，搜索范围越广，计算量也越大。
  int32_t branch_and_bound_depth = 0;
};

struct CellLimits {
  CellLimits(int init_num_x_cells, int init_num_y_cells)
      : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& origin,
            const CellLimits& cell_limits)
      : resolution_(resolution), origin_(origin), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0);
    CHECK_GT(cell_limits.num_x_cells, 0);
    CHECK_GT(cell_limits.num_y_cells, 0);
  }

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d& origin() const { return origin_; }

  // Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        std::lround((point.x() - origin_.x()) / resolution_ - 0.5),
        std::lround((origin_.y() - point.y()) / resolution_ - 0.5));
  }

  // Returns the center of the cell at 'cell_index'.
  Eigen::Vector2d GetCellCenter(const Eigen::Array2i cell_index) const {
    return Eigen::Vector2d(origin_.x() + resolution_ * (cell_index[0] + 0.5),
                           origin_.y() - resolution_ * (cell_index[1] + 0.5));
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;
  Eigen::Vector2d origin_;
  CellLimits cell_limits_;
};

}  // namespace solex_robot::navigation::localization_2d