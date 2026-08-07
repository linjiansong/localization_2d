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

#include "include/scan_match/probability_grid.h"

#include <glog/logging.h>

#include <opencv2/opencv.hpp>

namespace solex_robot::navigation::localization_2d {
ProbabilityGrid::ProbabilityGrid(const MapLimits& map_limits,
                                 ValueConversionTables* conversion_tables)
    : map_limits_(map_limits),
      correspondence_cost_cells_(map_limits_.cell_limits().num_x_cells *
                                     map_limits_.cell_limits().num_y_cells,
                                 kUnknownCorrespondenceValue),
      conversion_tables_(conversion_tables) {
  PrecomputeValueToBoundedFloat();
}

ProbabilityGrid::ProbabilityGrid(
    const MapLimits& map_limits,
    const std::vector<uint16_t>& correspondence_cost_cells,
    ValueConversionTables* conversion_tables)
    : map_limits_(map_limits),
      correspondence_cost_cells_(correspondence_cost_cells),
      conversion_tables_(conversion_tables) {
  PrecomputeValueToBoundedFloat();
}

void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16_t& cell = correspondence_cost_cells_[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  known_cells_box_.extend(cell_index.matrix());
}

void ProbabilityGrid::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                           CellLimits* const limits) const {
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }

  *offset = known_cells_box_.min().array();
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                       known_cells_box_.sizes().y() + 1);
}

std::unique_ptr<ProbabilityGrid> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = map_limits_.resolution();
  // const Eigen::Vector2d max =
  //     map_limits_.max() - resolution * Eigen::Vector2d(offset.x(),
  //     offset.y());
  const Eigen::Vector2d max =
      map_limits_.max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      std::make_unique<ProbabilityGrid>(MapLimits(resolution, max, cell_limits),
                                        conversion_tables_);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (IsKnown(xy_index + offset)) {
      cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
    }
  }

  return cropped_grid;
}

void ProbabilityGrid::FinishUpdate() {
  while (!update_indices_.empty()) {
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}

void ProbabilityGrid::GrowLimits(const Eigen::Vector2f& point) {
  CHECK(update_indices_.empty());
  while (!map_limits_.Contains(map_limits_.GetCellIndex(point))) {
    const CellLimits& cell_limits = map_limits_.cell_limits();
    const int x_offset = cell_limits.num_x_cells / 2;
    const int y_offset = cell_limits.num_y_cells / 2;
    // const MapLimits new_limits(
    //     map_limits_.resolution(),
    //     map_limits_.max() +
    //         map_limits_.resolution() * Eigen::Vector2d(x_offset, y_offset),
    //     CellLimits(2 * map_limits_.cell_limits().num_x_cells,
    //                2 * map_limits_.cell_limits().num_y_cells));
    const MapLimits new_limits(
        map_limits_.resolution(),
        map_limits_.max() +
            map_limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * map_limits_.cell_limits().num_x_cells,
                   2 * map_limits_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;
    std::vector<uint16_t> new_cells(new_size, kUnknownCorrespondenceValue);
    for (int y_index = 0; y_index < cell_limits.num_y_cells; ++y_index) {
      for (int x_index = 0; x_index < cell_limits.num_x_cells; ++x_index) {
        new_cells[offset + x_index + y_index * stride] =
            correspondence_cost_cells_[x_index +
                                       y_index * cell_limits.num_x_cells];
      }
    }

    correspondence_cost_cells_ = std::move(new_cells);
    map_limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16_t>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);

  uint16_t* cell = &(correspondence_cost_cells_[flat_index]);
  CHECK_NOTNULL(cell);
  if (*cell >= kUpdateMarker) {
    return false;
  }

  update_indices_.push_back(flat_index);

  /* 对数几率累加公式：l_t = l_t-1 + nverse_sensor_model(z_t),
   * inverse_sensor_model(z_t)是固定值. 在高频雷达、大地图的情况下,
   * 计算量会非常巨大.
   * 由于概率更新的步长和范围是有限的，Cartographer在程序初始化时，把所有可能的“旧概率值”和“雷达观测结果”组合对应的“新概率值”，提前算好并存放在一个大数组table里.
   * 把当前栅格的值*cell直接当作索引，去table这个数组里查找对应的新值。
   * 然后利用查找表(Lookup
   * Table)的思想，将复杂的浮点数概率计算转化为了极高效的数组查表操作.
   * 将查出来的新值覆盖写回到原栅格中，完成地图概率的更新。
   */
  *cell = table[*cell];
  // LOG(INFO) << "*cell = " << *cell;

  DCHECK_GE(*cell, kUpdateMarker);
  known_cells_box_.extend(cell_index.matrix());
  return true;
}

int ProbabilityGrid::ToFlatIndex(const Eigen::Array2i& cell_index) const {
  CHECK(map_limits_.Contains(cell_index)) << cell_index;
  return map_limits_.cell_limits().num_x_cells * cell_index.y() +
         cell_index.x();
}

bool ProbabilityGrid::IsKnown(const Eigen::Array2i& cell_index) const {
  return map_limits_.Contains(cell_index) &&
         correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
             kUnknownCorrespondenceValue;
}

void ProbabilityGrid::PrecomputeValueToBoundedFloat() {
  value_to_correspondence_cost_.clear();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  /* 第一次循环 (repeat == 0)：计算的是原始值 value 的转换结果。
   * 第二次循环 (repeat == 1)：计算的是 value + kUpdateMarker 的转换结果。
   * 在 Cartographer的地图实现中，栅格的值经常会被加上一个位掩码（例如
   * 32768）来标记“该像素在本次扫描中已被更新”。
   * 通过预计算两倍的表长，代码可以在查找阶段通过简单的偏移寻址直接拿到结果，而不需要判断当前值是否带有更新标记。
   **/
  value_to_correspondence_cost_.reserve(kRepetitionCount * kValueCount);
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      value_to_correspondence_cost_.push_back(SlowValueToBoundedFloat(
          value, kUnknownProbabilityValue, kMaxCorrespondenceCost,
          kMinCorrespondenceCost, kMaxCorrespondenceCost));
    }
  }
}

// Returns the correspondence cost of the cell with 'cell_index'.
float ProbabilityGrid::GetCorrespondenceCost(
    const Eigen::Array2i& cell_index) const {
  if (!map_limits_.Contains(cell_index)) {
    return kMaxCorrespondenceCost;
  }

  return ValueToCorrespondenceCost(
      correspondence_cost_cells_[ToFlatIndex(cell_index)]);
}

// Returns the probability of the cell with 'cell_index'.
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!map_limits_.Contains(cell_index)) {
    return kMinProbability;
  }

  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells_[ToFlatIndex(cell_index)]));
}

void ProbabilityGrid::VisualizeGrid() {
  const int width = map_limits_.cell_limits().num_x_cells;
  const int height = map_limits_.cell_limits().num_y_cells;
  LOG(INFO) << "width = " << width << ", height = " << height;

  // 创建一个灰度图：Height 行, Width 列
  cv::Mat image(height, width, CV_8UC1);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      Eigen::Array2i index(x, y);

      // 获取原始的对应代价 (Correspondence Cost)
      // 注意：你需要确保可以通过接口获取该值，或者直接访问私有成员
      // 这里假设你添加了一个公开接口或者在类内调用
      const float probability = GetProbability(index);
      // LOG(INFO) << "index = (" << x << ", " << y << "), probability = " <<
      // probability;

      // 将 0.0-1.0 的 Cost 映射到 0-255 的灰度值
      // 通常：0.0 (空闲) -> 255 (白色), 1.0 (障碍物) -> 0 (黑色)
      image.at<uchar>(y, x) = static_cast<uchar>((1.0f - probability) * 255.0f);
    }
  }

  cv::imwrite("/home/linjs/图片/grid_map111.png", image);
  LOG(INFO) << "Save to /home/linjs/图片/grid_map.png";
}

}  // namespace solex_robot::navigation::localization_2d