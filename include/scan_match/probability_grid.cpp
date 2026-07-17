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
      conversion_tables_(conversion_tables) {}

ProbabilityGrid::ProbabilityGrid(
    const MapLimits& map_limits,
    const std::vector<uint16_t>& correspondence_cost_cells,
    ValueConversionTables* conversion_tables)
    : map_limits_(map_limits),
      correspondence_cost_cells_(correspondence_cost_cells),
      conversion_tables_(conversion_tables) {
  PrecomputeValueToBoundedFloat();
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

float ProbabilityGrid::SlowValueToBoundedFloat(const uint16_t value,
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

void ProbabilityGrid::PrecomputeValueToBoundedFloat() {
  auto result = std::make_unique<std::vector<float>>();
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

  // std::cout << "0 = "
  //           << SlowValueToBoundedFloat(
  //                  0, kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
  //                  kUnknownProbabilityValue, kMaxCorrespondenceCost)
  //           << std::endl;

  // std::cout << "1 = "
  //           << SlowValueToBoundedFloat(
  //                  1, kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
  //                  kUnknownProbabilityValue, kMaxCorrespondenceCost)
  //           << std::endl;

  // std::cout << "32767 = "
  //           << SlowValueToBoundedFloat(
  //                  32767, kUnknownCorrespondenceValue,
  //                  kMaxCorrespondenceCost, kUnknownProbabilityValue,
  //                  kMaxCorrespondenceCost)
  //           << std::endl;

  // std::cout << "16384 = "
  //           << SlowValueToBoundedFloat(
  //                  16384, kUnknownCorrespondenceValue,
  //                  kMaxCorrespondenceCost, kUnknownProbabilityValue,
  //                  kMaxCorrespondenceCost)
  //           << std::endl;
}

// Returns the correspondence cost of the cell with 'cell_index'.
float ProbabilityGrid::GetCorrespondenceCost(
    const Eigen::Array2i& cell_index) const {
  if (!map_limits_.Contains(cell_index)) {
    return kMaxCorrespondenceCost;
  }

  // LOG(INFO) << "cost = "
  //           << ValueToCorrespondenceCost(
  //                  correspondence_cost_cells_[ToFlatIndex(cell_index)])
  //           << ", (" << cell_index.x() << ", " << cell_index.y() << ")";

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
      // LOG(INFO) << "index = (" << x << ", " << y << "), probability = " << probability;

      // 将 0.0-1.0 的 Cost 映射到 0-255 的灰度值
      // 通常：0.0 (空闲) -> 255 (白色), 1.0 (障碍物) -> 0 (黑色)
      image.at<uchar>(y, x) = static_cast<uchar>((1.0f - probability) * 255.0f);
    }
  }

  cv::imwrite("/home/linjs/图片/grid_map111.png", image);
  LOG(INFO) << "Save to /home/linjs/图片/grid_map.png";
}

}  // namespace solex_robot::navigation::localization_2d