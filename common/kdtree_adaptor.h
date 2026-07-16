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

#include <iostream>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "nanoflann.hpp"  // NOLINT

namespace solex_robot::navigation::localization_2d {

template <typename VectorOfEigenType, typename NumType = double,
          int Dimension = -1, typename Distance = nanoflann::metric_L2,
          typename IndexType = size_t>
class KDTreeVectorOfEigenAdaptor {
 public:
  using ThisType = KDTreeVectorOfEigenAdaptor<VectorOfEigenType, NumType,
                                              Dimension, Distance>;
  using MetricType =
      typename Distance::template traits<NumType, ThisType>::distance_t;
  using KDTreeIndexType =
      nanoflann::KDTreeSingleIndexAdaptor<MetricType, ThisType, Dimension,
                                          IndexType>;

  KDTreeVectorOfEigenAdaptor(const size_t /* dimensionality */,
                             const VectorOfEigenType& mat,
                             const int leaf_max_size = 10)
      : data_(mat) {
    CHECK(!mat.empty() && mat[0].size() != 0);
    const size_t dims = mat[0].size();
    CHECK((Dimension > 0 && static_cast<int>(dims) == Dimension) ||
           (Dimension == -1));
    index_ = std::make_unique<KDTreeIndexType>(
        static_cast<int>(dims), *this /* adaptor */,
        nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size));
  }

  virtual ~KDTreeVectorOfEigenAdaptor() = default;

  void BuildIndex() { index_->buildIndex(); }

  void Query(const NumType* query_point, const size_t num_closest,
             IndexType* out_indices, NumType* out_distances_sq,
             const int nChecks_IGNORED = 10) const {
    nanoflann::KNNResultSet<NumType, IndexType> resultSet(num_closest);
    resultSet.init(out_indices, out_distances_sq);
    index_->findNeighbors(resultSet, query_point,
                          nanoflann::SearchParams(nChecks_IGNORED));
  }

  void RadiusQuery(const NumType* query_point, const NumType radius,
                   std::vector<std::pair<size_t, NumType>>* matches) const {
    CHECK(matches != nullptr);
    const size_t num_matched = index_->radiusSearch(
        query_point, radius, *matches, nanoflann::SearchParams());
    (void)num_matched;  // result is contained in matches
  }

  const ThisType& derived() const { return *this; }

  size_t kdtree_get_point_count() const { return data_.size(); }

  NumType kdtree_get_pt(const size_t idx, const size_t dim) const {
    return data_[idx][dim];
  }

  const typename VectorOfEigenType::value_type& get_data(size_t index) const {
    CHECK(index < data_.size());
    return data_[index];
  }

  template <typename BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;
  }

 private:
  std::unique_ptr<KDTreeIndexType> index_;
  const VectorOfEigenType data_;
};

using KDTree3D =
    KDTreeVectorOfEigenAdaptor<std::vector<Eigen::Vector3d>, double>;
using KDTree2D =
    KDTreeVectorOfEigenAdaptor<std::vector<Eigen::Vector2d>, double>;
}  // namespace solex_robot::navigation::localization_2d
