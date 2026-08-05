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

#include "include/scan_match/laser_data_inserter.h"

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr int kSubpixelScale = 1000;
constexpr float kPadding = 1e-6f;
constexpr float kHittingProbability = 0.55;
constexpr float kMissingProbability = 0.49;
constexpr bool kInsertFreeSpace = true;

bool isEqual(const Eigen::Array2i& lhs, const Eigen::Array2i& rhs) {
  return ((lhs - rhs).matrix().lpNorm<1>() == 0);
}
}  // namespace

LaserDataInserter::LaserDataInserter()
    : value_to_correspondence_cost_(PrecomputeValueToBoundedFloat()),
      hitting_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(kHittingProbability))),
      missing_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(kMissingProbability))) {}

std::vector<float> LaserDataInserter::PrecomputeValueToBoundedFloat() const {
  std::vector<float> value_to_correspondence_cost;
  value_to_correspondence_cost.reserve(kRepetitionCount * kValueCount);
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  /* 第一次循环 (repeat == 0)：计算的是原始值 value 的转换结果。
   * 第二次循环 (repeat == 1)：计算的是 value + kUpdateMarker 的转换结果。
   * 在 Cartographer的地图实现中，栅格的值经常会被加上一个位掩码（例如
   * 32768）来标记“该像素在本次扫描中已被更新”。
   * 通过预计算两倍的表长，代码可以在查找阶段通过简单的偏移寻址直接拿到结果，而不需要判断当前值是否带有更新标记。
   **/
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      value_to_correspondence_cost.push_back(SlowValueToBoundedFloat(
          value, kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
          kMinCorrespondenceCost, kMaxCorrespondenceCost));
    }
  }

  return value_to_correspondence_cost;
}

std::vector<uint16_t>
LaserDataInserter::ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) const {
  std::vector<uint16_t> result;
  result.reserve(kValueCount);
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           value_to_correspondence_cost_[cell]))))) +
        kUpdateMarker);
  }

  return result;
}

void LaserDataInserter::GrowAsNeeded(
    const sensor::LaserDataPtr& laser_data,
    ProbabilityGrid* const probability_grid) const {
  Eigen::AlignedBox2f bounding_box(
      laser_data->pose().translation().cast<float>().head<2>());
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  for (const auto& hitting_point : laser_data->hitting_points()) {
    bounding_box.extend(
        (laser_data->pose() * hitting_point->position).cast<float>().head<2>());
  }
  for (const auto& missing_point : laser_data->missing_points()) {
    bounding_box.extend(
        (laser_data->pose() * missing_point->position).cast<float>().head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

// Compute all pixels that contain some part of the line segment connecting
// 'scaled_begin' and 'scaled_end'. 'scaled_begin' and 'scaled_end' are scaled
// by 'subpixel_scale'. 'scaled_begin' and 'scaled_end' are expected to be
// greater than zero. Return values are in pixels and not scaled.
std::vector<Eigen::Array2i> LaserDataInserter::RayToPixelMask(
    const Eigen::Array2i& scaled_begin, const Eigen::Array2i& scaled_end,
    int subpixel_scale) const {
  // For simplicity, we order 'scaled_begin' and 'scaled_end' by their x
  // coordinate.
  if (scaled_begin.x() > scaled_end.x()) {
    return RayToPixelMask(scaled_end, scaled_begin, subpixel_scale);
  }

  CHECK_GE(scaled_begin.x(), 0);
  CHECK_GE(scaled_begin.y(), 0);
  CHECK_GE(scaled_end.y(), 0);
  std::vector<Eigen::Array2i> pixel_mask;
  // Special case: We have to draw a vertical line in full pixels, as
  // 'scaled_begin' and 'scaled_end' have the same full pixel x coordinate.
  if (scaled_begin.x() / subpixel_scale == scaled_end.x() / subpixel_scale) {
    Eigen::Array2i current(
        scaled_begin.x() / subpixel_scale,
        std::min(scaled_begin.y(), scaled_end.y()) / subpixel_scale);
    pixel_mask.push_back(current);
    const int end_y =
        std::max(scaled_begin.y(), scaled_end.y()) / subpixel_scale;
    for (; current.y() <= end_y; ++current.y()) {
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    return pixel_mask;
  }

  const int64_t dx = scaled_end.x() - scaled_begin.x();
  const int64_t dy = scaled_end.y() - scaled_begin.y();
  const int64_t denominator = 2 * subpixel_scale * dx;

  // The current full pixel coordinates. We scaled_begin at 'scaled_begin'.
  Eigen::Array2i current = scaled_begin / subpixel_scale;
  pixel_mask.push_back(current);

  // To represent subpixel centers, we use a factor of 2 * 'subpixel_scale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * subpixel_scale) / (2 * subpixel_scale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * subpixel_scale)
  // | | | | -- center of first subpixel = 1 / (2 * subpixel_scale)
  // +-+-+-+ -- 0 = 0 / (2 * subpixel_scale)

  // The center of the subpixel part of 'scaled_begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64_t sub_y = (2 * (scaled_begin.y() % subpixel_scale) + 1) * dx;

  // The distance from the from 'scaled_begin' to the right pixel border, to be
  // divided by 2 * 'subpixel_scale'.
  const int first_pixel =
      2 * subpixel_scale - 2 * (scaled_begin.x() % subpixel_scale) - 1;
  // The same from the left pixel border to 'scaled_end'.
  const int last_pixel = 2 * (scaled_end.x() % subpixel_scale) + 1;

  // The full pixel x coordinate of 'scaled_end'.
  const int end_x = std::max(scaled_begin.x(), scaled_end.x()) / subpixel_scale;

  // Move from 'scaled_begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      sub_y += dy * 2 * subpixel_scale;
    }
    // Move from the pixel border on the right to 'scaled_end'.
    sub_y += dy * last_pixel;
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
    return pixel_mask;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * subpixel_scale;
  }
  sub_y += dy * last_pixel;
  if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
  return pixel_mask;
}

void LaserDataInserter::CastRays(const sensor::LaserDataPtr& laser_data,
                                 const std::vector<uint16_t>& hitting_table,
                                 const std::vector<uint16_t>& missing_table,
                                 ProbabilityGrid* probability_grid) const {
  GrowAsNeeded(laser_data, probability_grid);

  const MapLimits& limits = probability_grid->map_limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.origin(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin = superscaled_limits.GetCellIndex(
      laser_data->pose().translation().cast<float>().head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(laser_data->hitting_points().size());
  for (const auto& hitting_point : laser_data->hitting_points()) {
    ends.push_back(superscaled_limits.GetCellIndex(
        (laser_data->pose() * hitting_point->position)
            .cast<float>()
            .head<2>()));
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale,
                                       hitting_table);

    LOG(INFO) << "hitting_cell = " << ends.back().x() / kSubpixelScale << ", "
              << ends.back().y() / kSubpixelScale;
  }

  if (!kInsertFreeSpace) {
    return;
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {
    const std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, missing_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  for (const auto& missing_point : laser_data->missing_points()) {
    const Eigen::Array2i cell_index = superscaled_limits.GetCellIndex(
        (laser_data->pose() * missing_point->position).cast<float>().head<2>());
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, cell_index, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, missing_table);
    }
  }
}

void LaserDataInserter::Insert(const sensor::LaserDataPtr& laser_data,
                               ProbabilityGrid* probability_grid) const {
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a missing_point in the same cell).
  CastRays(laser_data, hitting_table_, missing_table_, probability_grid);
  probability_grid->FinishUpdate();
}
}  // namespace solex_robot::navigation::localization_2d