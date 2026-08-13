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

#include <string>

namespace solex_robot::navigation::localization_2d {
namespace options {
struct SensorOptions {
  std::string laser_topic;
  std::string imu_topic;
  std::string odometry_topic;
  bool use_laser = false;
  bool use_imu = false;
  bool use_odometry = false;
};

struct LocalizationStatusOptions {
  float min_match_score = 0.f;
  float max_roaming_distance = 0.f;  // meter
  float max_roaming_angle = 0.f;     // degree
};

struct FastCorrelativeScanMatcherOptions {
  // 线性搜索窗口的大小（以米为单位）。
  // 这个参数决定了在多大的范围内进行平移搜索。
  float linear_search_window = 0.f;

  // 角度搜索窗口的大小（以弧度为单位）。
  // 这个参数决定了在多大的范围内进行旋转搜索。
  float angular_search_window = 0.f;

  // 分支定界算法（Branch-and-Bound）所使用的查找表层数。
  // Cartographer 会为地图构建多分辨率的查找表（类似图像金字塔），
  // 该值决定了层级深度。层数越多，搜索范围越广，计算量也越大。
  int32_t branch_and_bound_depth = 0;
};

struct RealTimeCorrelativeScanMatcherOptions {
  // 线性搜索窗口的大小（单位：米），用于限定在哪个范围内寻找最佳平移对齐位置
  float linear_search_window = 0.f;

  // 角度搜索窗口的大小（单位：弧度），用于限定在哪个范围内寻找最佳旋转对齐角度
  float angular_search_window = 0.f;

  // 平移变化惩罚权重（用于对偏离初始估计的平移施加惩罚得分）
  float translation_delta_cost_weight = 0.f;

  // 旋转变化惩罚权重（用于对偏离初始估计的旋转施加惩罚得分）
  float rotation_delta_cost_weight = 0.f;
};

struct MotionFilterOptions {
  float max_time = 0.f;      // second
  float max_distance = 0.f;  // meter
  float max_angle = 0.f;     // degree
};

struct PoseGraphOptions {
  int optimize_every_nodes = 0;
  int max_node_buffer_length = 0;
  float min_global_localization_score = 0.f;
  float min_relocalization_score = 0.f;
  float min_tracking_score = 0.f;
  RealTimeCorrelativeScanMatcherOptions
      real_time_correlative_scan_matcher_options;
  FastCorrelativeScanMatcherOptions fast_correlative_scan_matcher_options;
};

struct LocalMapBuilderOptions {
  bool use_real_time_correlative_scan_match = false;
  RealTimeCorrelativeScanMatcherOptions
      real_time_correlative_scan_matcher_options;
  MotionFilterOptions motion_filter_options;
};

struct LocalizationOptions {
  SensorOptions sensor_options;
  LocalizationStatusOptions localization_status_options;
  LocalMapBuilderOptions local_map_builder_options;
  PoseGraphOptions pose_graph_options;
};
}  // namespace options
}  // namespace solex_robot::navigation::localization_2d