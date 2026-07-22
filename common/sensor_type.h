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
#include <memory>
#include <vector>

#include "common/rigid_transform.h"

namespace solex_robot::navigation::localization_2d {
namespace sensor {

struct ImuData {
  double time = 0.0;
  transform::Rigid3d pose;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;

};

struct OdometryData {
  double time = 0.0;
  transform::Rigid3d pose;
};

}  // namespace sensor

}  // namespace solex_robot::navigation::localization_2d