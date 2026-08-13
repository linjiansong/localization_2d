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
//  or all damages (including recovery of attorneys' fees) which may be //`
//  suffered and or incurred as a result of your infringement.                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "common/options.h"
#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"
#include "src/node.h"

constexpr float kDegreeToRadian = M_PI / 180.0;
DEFINE_string(localization_node_log_dir, "",
              "The directory to store localization_2d log files.");
DEFINE_string(localization_option_file, "",
              "The directory to store localization_2d option files.");

solex_robot::navigation::localization_2d::options::LocalizationOptions
LoadOptionsFromYaml(const std::string& yaml_file) {
  // 1. 检查文件物理存在
  if (!std::filesystem::exists(yaml_file)) {
    LOG(FATAL) << "Configuration file not found at: " << yaml_file;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_file);
  } catch (const std::exception& e) {
    LOG(FATAL) << "Failed to load YAML file: " << e.what();
  }

  solex_robot::navigation::localization_2d::options::LocalizationOptions
      options;
  try {
    // SensorOptions
    if (config["sensors"]) {
      // Lidar
      options.sensor_options.laser_topic =
          config["sensors"]["lidar"]["topic"].as<std::string>();
      options.sensor_options.use_laser =
          config["sensors"]["lidar"]["enabled"].as<bool>();
      // IMU
      options.sensor_options.imu_topic =
          config["sensors"]["imu"]["topic"].as<std::string>();
      options.sensor_options.use_imu =
          config["sensors"]["imu"]["enabled"].as<bool>();
      // Odom
      options.sensor_options.odometry_topic =
          config["sensors"]["odom"]["topic"].as<std::string>();
      options.sensor_options.use_odometry =
          config["sensors"]["odom"]["enabled"].as<bool>();
    }

    if (config["localization_status"]) {
      options.localization_status_options.min_match_score =
          config["localization_status"]["min_match_score"].as<float>();
      options.localization_status_options.max_roaming_distance =
          config["localization_status"]["max_roaming_distance"].as<float>();
      options.localization_status_options.max_roaming_angle =
          config["localization_status"]["max_roaming_angle"].as<float>() * kDegreeToRadian;
    }

    // LocalMapBuilderOptions
    if (config["local_map_builder"]) {
      options.local_map_builder_options.use_real_time_correlative_scan_match =
          config["local_map_builder"]["use_online_correlative_scan_match"]
              .as<bool>();

      // 内部 RealTimeCorrelativeScanMatcherOptions
      if (config["local_map_builder"]["real_time_correlative_scan_matcher"]) {
        options.local_map_builder_options
            .real_time_correlative_scan_matcher_options.linear_search_window =
            config["local_map_builder"]["real_time_correlative_scan_matcher"]
                  ["linear_search_window"]
                      .as<float>();
        options.local_map_builder_options
            .real_time_correlative_scan_matcher_options.angular_search_window =
            config["local_map_builder"]["real_time_correlative_scan_matcher"]
                  ["angular_search_window"]
                      .as<float>() *
            kDegreeToRadian;
        options.local_map_builder_options
            .real_time_correlative_scan_matcher_options
            .translation_delta_cost_weight =
            config["local_map_builder"]["real_time_correlative_scan_matcher"]
                  ["translation_delta_cost_weight"]
                      .as<float>();
        options.local_map_builder_options
            .real_time_correlative_scan_matcher_options
            .rotation_delta_cost_weight =
            config["local_map_builder"]["real_time_correlative_scan_matcher"]
                  ["rotation_delta_cost_weight"]
                      .as<float>();
      }

      // 内部 MotionFilterOptions
      if (config["local_map_builder"]["motion_filter"]) {
        options.local_map_builder_options.motion_filter_options.max_time =
            config["local_map_builder"]["motion_filter"]["max_time"]
                .as<float>();
        options.local_map_builder_options.motion_filter_options.max_distance =
            config["local_map_builder"]["motion_filter"]["max_distance"]
                .as<float>();
        options.local_map_builder_options.motion_filter_options.max_angle =
            config["local_map_builder"]["motion_filter"]["max_angle"]
                .as<float>() *
            (float)kDegreeToRadian;
      }
    }

    // PoseGraphOptions
    if (config["pose_graph"]) {
      options.pose_graph_options.optimize_every_nodes =
          config["pose_graph"]["optimize_every_nodes"].as<int>();
      options.pose_graph_options.max_node_buffer_length =
          config["pose_graph"]["max_node_buffer_length"].as<int>();
      options.pose_graph_options.min_global_localization_score =
          config["pose_graph"]["min_global_localization_score"].as<float>();
      options.pose_graph_options.min_relocalization_score =
          config["pose_graph"]["min_relocalization_score"].as<float>();
      options.pose_graph_options.min_tracking_score =
          config["pose_graph"]["min_tracking_score"].as<float>();

      // PoseGraph 内部的 RealTimeCorrelativeScanMatcher
      if (config["pose_graph"]["real_time_correlative_scan_matcher"]) {
        options.pose_graph_options.real_time_correlative_scan_matcher_options
            .linear_search_window =
            config["pose_graph"]["real_time_correlative_scan_matcher"]
                  ["linear_search_window"]
                      .as<float>();
        options.pose_graph_options.real_time_correlative_scan_matcher_options
            .angular_search_window =
            config["pose_graph"]["real_time_correlative_scan_matcher"]
                  ["angular_search_window"]
                      .as<float>() *
            kDegreeToRadian;
        options.pose_graph_options.real_time_correlative_scan_matcher_options
            .translation_delta_cost_weight =
            config["pose_graph"]["real_time_correlative_scan_matcher"]
                  ["translation_delta_cost_weight"]
                      .as<float>();
        options.pose_graph_options.real_time_correlative_scan_matcher_options
            .rotation_delta_cost_weight =
            config["pose_graph"]["real_time_correlative_scan_matcher"]
                  ["rotation_delta_cost_weight"]
                      .as<float>();
      }

      // FastCorrelativeScanMatcher
      if (config["pose_graph"]["fast_correlative_scan_matcher"]) {
        options.pose_graph_options.fast_correlative_scan_matcher_options
            .linear_search_window =
            config["pose_graph"]["fast_correlative_scan_matcher"]
                  ["linear_search_window"]
                      .as<float>();
        options.pose_graph_options.fast_correlative_scan_matcher_options
            .angular_search_window =
            config["pose_graph"]["fast_correlative_scan_matcher"]
                  ["angular_search_window"]
                      .as<float>() *
            kDegreeToRadian;
        options.pose_graph_options.fast_correlative_scan_matcher_options
            .branch_and_bound_depth =
            config["pose_graph"]["fast_correlative_scan_matcher"]
                  ["branch_and_bound_depth"]
                      .as<int32_t>();
      }
    }
  } catch (const YAML::TypedBadConversion<float>& exc) {
    LOG(FATAL)
        << "YAML Type Error: Expected a number but found something else at: "
        << exc.what();
  } catch (const YAML::Exception& exc) {
    LOG(FATAL) << "YAML Parsing Error: " << exc.what();
  }

  LOG(INFO) << "Successfully loaded localization options from YAML.";
  return options;
}

void Run(const std::string& option_file) {
  // 1. 直接创建你的自定义节点实例
  // 确保 LocalizationNode 继承了 rclcpp::Node
  auto node = std::make_shared<
      solex_robot::navigation::localization_2d::LocalizationNode>(
      LoadOptionsFromYaml(option_file));

  // 2. 运行节点，开始监听消息
  rclcpp::spin(node);
}

int main(int argc, char** argv) {
  char* log_dir_env_ptr = std::getenv("LOCALIZATION_LOG_DIR");
  char* option_file_env_ptr = std::getenv("LOCALIZATION_OPTION_FILE");
  if (log_dir_env_ptr == nullptr || option_file_env_ptr == nullptr) {
    LOG(ERROR)
        << "LOCALIZATION_LOG_DIR or LOCALIZATION_OPTION_FILE is not set!";
    return -1;
  }

  const std::string log_dir = log_dir_env_ptr;
  const std::string option_file = option_file_env_ptr;

  // -1 表示连INFO级别的日志都无缓冲实时输出
  FLAGS_logbuflevel = -1;
  // 刷新缓冲区时间间隔设为0秒
  FLAGS_logbufsecs = 0;

  FLAGS_alsologtostderr = true;   // 同时输出到终端和文件
  FLAGS_colorlogtostderr = true;  // 终端输出带颜色

  // 设置日志目录（必须在ParseCommandLineFlags之后，InitGoogleLogging之前）
  if (!log_dir.empty()) {
    if (!std::filesystem::exists(log_dir)) {
      std::filesystem::create_directories(log_dir);
    }
    FLAGS_log_dir = log_dir;
  }

  // 初始化glog和信号捕捉
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  LOG(INFO) << "glog initialized. Real-time logging enabled. Log dir: "
            << (FLAGS_log_dir.empty() ? "stderr only" : FLAGS_log_dir);

  rclcpp::init(argc, argv);
  Run(option_file);

  google::ShutdownGoogleLogging();
  rclcpp::shutdown();
  return 0;
}