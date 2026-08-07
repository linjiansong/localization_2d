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

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "common/base_type.h"
#include "common/sensor_type.h"
#include "include/map_builder/local_map_builder.h"
#include "include/pose_estimator/pose_extrapolator.h"
#include "include/pose_graph/pose_graph.h"
#include "include/scan_match/probability_grid.h"

namespace solex_robot::navigation::localization_2d {

class Localization {
 public:
  Localization() = default;
  ~Localization();

  void AddImuData(const sensor::ImuDataPtr& imu_data);
  void AddOdometryData(const sensor::OdometryDataPtr& odometry_data);
  void AddLaserData(const sensor::LaserDataPtr& laser_data);
  void AddGridMap(std::shared_ptr<ProbabilityGrid> probability_grid);
  void AddInitialPose(const transform::Rigid2d& initial_pose);
  void RequestGlobalLocalization();
  void Init();

  const Eigen::Matrix4d GetLatestPose(const double timestamp);
  const ProbabilityGrid* GetLocalMap();

 private:
  void GlobalLocalization(const sensor::LaserDataPtr& laser_data);

  void InitialLocalization(const sensor::LaserDataPtr& laser_data);

  void Track(const sensor::LaserDataPtr& laser_data);

  float CalculateMatchScore(const sensor::LaserDataPtr& laser_data);

  void EvaluateLocalizationStatus(const sensor::LaserDataPtr& laser_data);

 private:
  std::shared_ptr<ProbabilityGrid> probability_grid_;
  std::shared_ptr<LocalMapBuilder> local_map_builder_;
  std::shared_ptr<PoseExtrapolator> pose_extrapolator_;
  std::shared_ptr<PoseGraph> pose_graph_;

  transform::Rigid2d initial_pose_;
  LocalizationStatus localization_status_ = LocalizationStatus::kUnknown;
  std::mutex localization_status_mutex_;

  double roaming_distance_ = 0.0;
  double roaming_angle_ = 0.0;
};
}  // namespace solex_robot::navigation::localization_2d
