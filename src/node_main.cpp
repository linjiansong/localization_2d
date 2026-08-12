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

#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"
#include "src/node.h"

namespace solex_robot::navigation::localization_2d {

void Run() {
  // 1. 直接创建你的自定义节点实例
  // 确保 LocalizationNode 继承了 rclcpp::Node
  auto node = std::make_shared<LocalizationNode>();

  // 2. 运行节点，开始监听消息
  rclcpp::spin(node);
}

}  // namespace solex_robot::navigation::localization_2d

int main(int argc, char** argv) {
  // 初始化rclcpp
  rclcpp::init(argc, argv);

  // Log同时输出到终端和文件
  FLAGS_alsologtostderr = true;

  // 初始化 gflags
  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // 安装崩溃信号处理器，捕捉段错误并打印堆栈
  google::InstallFailureSignalHandler();

  // 运行节点逻辑
  solex_robot::navigation::localization_2d::Run();

  // 清理资源
  rclcpp::shutdown();
  return 0;
}
