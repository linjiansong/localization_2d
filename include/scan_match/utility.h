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
constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;
constexpr uint16_t kUnknownCorrespondenceValue = 0;
constexpr uint16_t kUnknownProbabilityValue = 0;
constexpr uint16_t kValueCount = 32768;

constexpr int kRepetitionCount = 2;
constexpr double kSafetyMargin = 1. - 1e-3;
// 小于kUpdateMarker的值：代表正常的Correspondence
// Cost。这个数值可以直接作为查找表（table）的下标去查询更新后的代价。
// 大于等于kUpdateMarker的值：代表“本帧已被更新过”的标记
constexpr uint16_t kUpdateMarker = 1u << 15;
}  // namespace

inline float SlowValueToBoundedFloat(const uint16_t value,
                                     const uint16_t unknown_value,
                                     const float unknown_result,
                                     const float lower_bound,
                                     const float upper_bound) {
  CHECK_LT(value, kValueCount);
  if (value == unknown_value) {
    return unknown_result;
  }

  // 将有效的值范围[1, kValueCount - 1]线性映射到[lower_bound, upper_bound]
  const float scale = (upper_bound - lower_bound) / (kValueCount - 2.f);

  // (lower_bound - scale) 为偏移量, 保证了当value为1时，输出正好是lower_bound
  return value * scale + (lower_bound - scale);
}

inline uint16_t BoundedFloatToValue(const float float_value,
                                    const float lower_bound,
                                    const float upper_bound) {
  const int value =
      std::lround(
          (std::clamp(float_value, lower_bound, upper_bound) - lower_bound) *
          (32766.f / (upper_bound - lower_bound))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

inline uint16_t CorrespondenceCostToValue(const float correspondence_cost) {
  return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost,
                             kMaxCorrespondenceCost);
}

inline float ProbabilityToCorrespondenceCost(const float probability) {
  return 1.f - probability;
}

inline float CorrespondenceCostToProbability(const float correspondence_cost) {
  return 1.f - correspondence_cost;
}

inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

inline float Odds(float probability) {
  return probability / (1.f - probability);
}

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

struct RealTimeCorrelativeScanMatcherOptions2D {
  // 线性搜索窗口的大小（单位：米），用于限定在哪个范围内寻找最佳平移对齐位置
  double linear_search_window = 0.0;

  // 角度搜索窗口的大小（单位：弧度），用于限定在哪个范围内寻找最佳旋转对齐角度
  double angular_search_window = 0.0;

  // 平移变化惩罚权重（用于对偏离初始估计的平移施加惩罚得分）
  double translation_delta_cost_weight = 0.0;

  // 旋转变化惩罚权重（用于对偏离初始估计的旋转施加惩罚得分）
  double rotation_delta_cost_weight = 0.0;
};

struct CellLimits {
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
      : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

// Describes the search space.
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(const double linear_search_window,
                   const double angular_search_window,
                   const std::vector<Eigen::Vector3d>& point_cloud,
                   const double resolution)
      : resolution(resolution) {
    // We set this value to something on the order of resolution to make sure
    // that the std::acos() below is defined. float max_scan_range = 3.f *
    // resolution; for (const Eigen::Vector3f& point : point_cloud) {
    //   const float range = point.position.head<2>().norm();
    //   max_scan_range = std::max(range, max_scan_range);
    // }

    // angular_perturbation_step_size =
    //     kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
    //                                       (2. *
    //                                       common::Pow2(max_scan_range)));
    angular_perturbation_step_size = 0.25 * M_PI / 180.;
    num_angular_perturbations =
        std::ceil(angular_search_window / angular_perturbation_step_size);

    num_scans = 2 * num_angular_perturbations + 1;

    const int num_linear_perturbations =
        std::ceil(linear_search_window / resolution);

    linear_bounds.reserve(num_scans);
    for (int i = 0; i != num_scans; ++i) {
      linear_bounds.push_back(
          LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                       -num_linear_perturbations, num_linear_perturbations});
    }
  }

  // Tightens the search window as much as possible.
  // Make sure at least one scan point fall within the map
  void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                   const CellLimits& cell_limits) {
    CHECK_EQ(scans.size(), num_scans);
    CHECK_EQ(linear_bounds.size(), num_scans);
    for (int i = 0; i != num_scans; ++i) {
      Eigen::Array2i min_bound = Eigen::Array2i::Zero();
      Eigen::Array2i max_bound = Eigen::Array2i::Zero();
      for (const Eigen::Array2i& xy_index : scans[i]) {
        // xy_index + offset >= 0 --> offset > -xy_index
        min_bound = min_bound.min(-xy_index);
        // xy_index + offset <= cell_limits --> offset <= cell_limits - xy_index
        max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                                 cell_limits.num_y_cells - 1) -
                                  xy_index);
      }
      linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
      linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
      linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
      linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
    }
  }

  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        // x(-y_index_offset * search_parameters.resolution),
        // y(-x_index_offset * search_parameters.resolution),
        x(x_index_offset * search_parameters.resolution),
        y(-y_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate2D relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

class ValueConversionTables {
 public:
  const std::vector<float>* GetConversionTable(float unknown_result,
                                               float lower_bound,
                                               float upper_bound);

 private:
  std::map<const std::tuple<float /* unknown_result */, float /* lower_bound */,
                            float /* upper_bound */>,
           std::unique_ptr<const std::vector<float>>>
      bounds_to_lookup_table_;
};

class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0);
    CHECK_GT(cell_limits.num_x_cells, 0);
    CHECK_GT(cell_limits.num_y_cells, 0);
  }

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d& max() const { return max_; }

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
        std::lround((max_.x() - point.x()) / resolution_ - 0.5),
        std::lround((max_.y() - point.y()) / resolution_ - 0.5));
    // return Eigen::Array2i(
    //     std::lround((max_.y() - point.y()) / resolution_ - 0.5),
    //     std::lround((max_.x() - point.x()) / resolution_ - 0.5));
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
  Eigen::Vector2d max_;
  CellLimits cell_limits_;
};

class XYIndexRangeIterator
    : public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
 public:
  // Constructs a new iterator for the specified range.
  XYIndexRangeIterator(const Eigen::Array2i& min_xy_index,
                       const Eigen::Array2i& max_xy_index)
      : min_xy_index_(min_xy_index),
        max_xy_index_(max_xy_index),
        xy_index_(min_xy_index) {}

  // Constructs a new iterator for everything contained in 'cell_limits'.
  explicit XYIndexRangeIterator(const CellLimits& cell_limits)
      : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                             Eigen::Array2i(cell_limits.num_x_cells - 1,
                                            cell_limits.num_y_cells - 1)) {}

  XYIndexRangeIterator& operator++() {
    // This is a necessary evil. Bounds checking is very expensive and needs to
    // be avoided in production. We have unit tests that exercise this check
    // in debug mode.
    DCHECK(*this != end());
    if (xy_index_.x() < max_xy_index_.x()) {
      ++xy_index_.x();
    } else {
      xy_index_.x() = min_xy_index_.x();
      ++xy_index_.y();
    }
    return *this;
  }

  Eigen::Array2i& operator*() { return xy_index_; }

  bool operator==(const XYIndexRangeIterator& other) const {
    return (xy_index_ == other.xy_index_).all();
  }

  bool operator!=(const XYIndexRangeIterator& other) const {
    return !operator==(other);
  }

  XYIndexRangeIterator begin() {
    return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
  }

  XYIndexRangeIterator end() {
    XYIndexRangeIterator it = begin();
    it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
    return it;
  }

 private:
  Eigen::Array2i min_xy_index_;
  Eigen::Array2i max_xy_index_;
  Eigen::Array2i xy_index_;
};

}  // namespace solex_robot::navigation::localization_2d