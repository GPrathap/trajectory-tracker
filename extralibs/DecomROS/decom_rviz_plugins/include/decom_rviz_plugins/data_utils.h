/**
 * @file data_utils.h
 * @brief Provide a few widely used function for basic type
 */
#ifndef DATA_UTILS_H
#define DATA_UTILS_H

#include <decom_rviz_plugins/data_type.h>

///Template for transforming a vector
template <class T, class TF>
vec_E<T> transform_vec(const vec_E<T> &t, const TF &tf) {
  vec_E<T> new_t;
  for (const auto &it : t)
    new_t.push_back(tf * it);
  return new_t;
}

///Template for calculating distance
template <class T>
decimal_t total_distance(const vec_E<T>& vs){
  decimal_t dist = 0;
  for(unsigned int i = 1; i < vs.size(); i++)
    dist += (vs[i] - vs[i-1]).norm();

  return dist;
}


///Transform all entries in a vector using given TF
#define transform_vec3 transform_vec<Eigen::Matrix<decimal_t, 3, 1>, Eigen::Transform<decimal_t, 3, Eigen::Affine>>
///Sum up total distance for Eigen::Matrix<decimal_t, 3, 1>
#define total_distance3f total_distance<Eigen::Matrix<decimal_t, 3, 1>>
///Sum up total distance for Eigen::Matrix<int, 3, 1>
#define total_distance3i total_distance<Eigen::Matrix<int, 3, 1>>
#endif
