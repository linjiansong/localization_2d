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

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "ceres/ceres.h"

namespace solex_robot::navigation::localization_2d {

namespace {
constexpr double kEpsilon = 1.e-6;
}

template <typename T>
T signLocalVersion(T x) {
  return (T)(x > T(0)) - (T)(x < T(0));
}

template <typename T>
Eigen::Matrix<T, 3, 1> SkewMat2Vec(const Eigen::Matrix<T, 3, 3>& matrix) {
  Eigen::Matrix<T, 3, 1> v;
  v(0) = 0.5 * (matrix(2, 1) - matrix(1, 2));
  v(1) = 0.5 * (matrix(0, 2) - matrix(2, 0));
  v(2) = 0.5 * (matrix(1, 0) - matrix(0, 1));
  return v;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Vec2SkewMat(const Eigen::Matrix<T, 3, 1>& vec) {
  Eigen::Matrix<T, 3, 3> matrix = Eigen::Matrix<T, 3, 3>::Zero();
  matrix(0, 1) = -vec(2);
  matrix(0, 2) = vec(1);
  matrix(1, 0) = vec(2);
  matrix(1, 2) = -vec(0);
  matrix(2, 0) = -vec(1);
  matrix(2, 1) = vec(0);
  return matrix;
}

template <typename T>
Eigen::Matrix<T, 3, 3> ExpSo3(const Eigen::Matrix<T, 3, 1>& so3) {
  T n = so3.norm();
  Eigen::Matrix<T, 3, 3> rotation;
  const Eigen::Matrix<T, 3, 3> unit_matrix = Eigen::Matrix<T, 3, 3>::Identity();
  if (n < T(kEpsilon)) {
    const Eigen::Matrix<T, 3, 3> vx = Vec2SkewMat(so3);
    rotation = unit_matrix + vx + T(0.5) * vx * vx;
  } else {
    Eigen::Matrix<T, 3, 1> a = so3 / n;
    Eigen::Matrix<T, 3, 3> ax = Vec2SkewMat(a);
    rotation = unit_matrix + sin(n) * ax + (T(1.0) - cos(n)) * ax * ax;
  }
  return rotation;
}

template <typename T>
Eigen::Matrix<T, 3, 1> LogSo3(const Eigen::Matrix<T, 3, 3>& rotation_matrix) {
  Eigen::AngleAxis<T> angle_axis(rotation_matrix);
  Eigen::Matrix<T, 3, 1> so3 = angle_axis.angle() * angle_axis.axis();
  return so3;
}

template <typename T>
Eigen::Matrix<T, 3, 3> InverseRightApproxSo3(
    const Eigen::Matrix<T, 3, 1>& so3) {
  T phi = so3.norm();
  Eigen::Matrix<T, 3, 3> invJr, I3;
  I3.setIdentity();
  if (phi < T(kEpsilon)) {
    Eigen::Matrix<T, 3, 3> vx = Vec2SkewMat(so3);
    invJr = I3 + T(0.5) * vx + T(1.0 / 12.0) * vx * vx;
  } else {
    Eigen::Matrix<T, 3, 1> a = so3 / phi;
    Eigen::Matrix<T, 3, 3> ax = Vec2SkewMat(a);
    T x1 = T(0.5) * phi;
    T x2 = T(1.0) - x1 / tan(x1);
    invJr = I3 + x1 * ax + x2 * ax * ax;
  }
  return invJr;
}

class SE3SeprateLieAlgo : public ceres::LocalParameterization {
 public:
  SE3SeprateLieAlgo() = default;
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> r(x);
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> t(x + 3);
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> dr(delta);
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> dt(delta + 3);

    const Eigen::Matrix<double, 3, 3> R = ExpSo3<double>(r);
    const Eigen::Matrix<double, 3, 3> dR = ExpSo3<double>(dr);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> upd_v(x_plus_delta);
    upd_v.template topRows<3>() = LogSo3<double>(dR * R);
    upd_v.template bottomRows<3>() = t + dt;

    return true;
  }

  bool ComputeJacobian(const double* x, double* jacobian) const {
    Eigen::Matrix<double, 6, 6> jac = Eigen::Matrix<double, 6, 6>::Identity();
    jac.topLeftCorner(3, 3) =
        InverseRightApproxSo3<double>(-Eigen::Vector3d(x));
    ceres::MatrixRef(jacobian, 6, 6) = jac;
    return true;
  }

  int GlobalSize() const { return 6; }
  int LocalSize() const { return 6; }
  static ceres::LocalParameterization* Create() {
    return new SE3SeprateLieAlgo();
  }
};

class FixedPoseCost {
 public:
  FixedPoseCost(const Eigen::Matrix4d& fixed_pose,
                const Eigen::Matrix<double, 6, 6>& sqrt_info)
      : fixed_pose_(fixed_pose), sqrt_info_(sqrt_info) {};

  template <typename T>
  bool operator()(const T* pose_vec, T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> r0(pose_vec);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p0(pose_vec + 3);

    const Eigen::Matrix<T, 3, 3> R0 = ExpSo3<T>(r0);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> err(residual);
    err.template topRows<3>() =
        LogSo3<T>(R0 * fixed_pose_.template block<3, 3>(0, 0).transpose());
    err.template bottomRows<3>() = p0 - fixed_pose_.template block<3, 1>(0, 3);
    err = sqrt_info_ * err;
    return true;
  };

  static ceres::CostFunction* Create(
      const Eigen::Matrix4d& fixed_pose,
      const Eigen::Matrix<double, 6, 6>& sqrt_info) {
    return new ceres::AutoDiffCostFunction<FixedPoseCost, 6, 6>(
        new FixedPoseCost(fixed_pose, sqrt_info));
  }

 private:
  const Eigen::Matrix4d fixed_pose_;
  const Eigen::Matrix<double, 6, 6> sqrt_info_;
};

// T1 = T0 * dT
class RelativePoseCost {
 public:
  RelativePoseCost(const Eigen::Matrix4d& delta_transform,
                   const Eigen::Matrix<double, 6, 6>& sqrt_info)
      : delta_transform_(delta_transform), sqrt_info_(sqrt_info) {};

  template <typename T>
  bool operator()(const T* pose_vec_prev, const T* pose_vec_curr,
                  T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> r0(pose_vec_prev);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p0(pose_vec_prev + 3);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> r1(pose_vec_curr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(pose_vec_curr + 3);

    const Eigen::Matrix<T, 3, 3> R0 = ExpSo3<T>(r0);
    const Eigen::Matrix<T, 3, 3> R1 = ExpSo3<T>(r1);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> err(residual);
    err.template topRows<3>() = LogSo3<T>(
        R0 * delta_transform_.template block<3, 3>(0, 0) * R1.transpose());
    err.template bottomRows<3>() =
        R0 * delta_transform_.template block<3, 1>(0, 3) + p0 - p1;

    err = sqrt_info_ * err;
    return true;
  };

  static ceres::CostFunction* Create(
      const Eigen::Matrix4d& delta_transform,
      const Eigen::Matrix<double, 6, 6>& sqrt_info) {
    return new ceres::AutoDiffCostFunction<RelativePoseCost, 6, 6, 6>(
        new RelativePoseCost(delta_transform, sqrt_info));
  }

 private:
  const Eigen::Matrix4d delta_transform_;
  const Eigen::Matrix<double, 6, 6> sqrt_info_;
};
}  // namespace solex_robot::navigation::localization_2d
