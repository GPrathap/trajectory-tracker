/**
 * @file geometric_utils.h
 * @brief basic geometry utils
 */
#ifndef DECOMP_GEOMETRIC_UTILS_H
#define DECOMP_GEOMETRIC_UTILS_H

#include <Eigen/Eigenvalues>
#include <decom_rviz_plugins/data_utils.h>
#include <decom_rviz_plugins/polyhedron.h>
#include <iostream>

/// Calculate eigen values
template <int Dim> Vecf<Dim> eigen_value(const Matf<Dim, Dim> &A) {
  Eigen::SelfAdjointEigenSolver<Matf<Dim, Dim>> es(A);
  return es.eigenvalues();
}

/// Calculate rotation matrix from a vector (aligned with x-axis)
inline Eigen::Matrix<decimal_t, 2,2> vec2_to_rotation(const Eigen::Matrix<decimal_t, 2, 1> &v) {
  decimal_t yaw = std::atan2(v(1), v(0));
  Eigen::Matrix<decimal_t, 2,2> R;
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  return R;
}

inline Eigen::Matrix<decimal_t, 3, 3> vec3_to_rotation(const Eigen::Matrix<decimal_t, 3, 1> &v) {
  // zero roll
  Eigen::Matrix<decimal_t, 3, 1> rpy(0, std::atan2(-v(2), v.topRows<2>().norm()),
            std::atan2(v(1), v(0)));
  Quatf qx(cos(rpy(0) / 2), sin(rpy(0) / 2), 0, 0);
  Quatf qy(cos(rpy(1) / 2), 0, sin(rpy(1) / 2), 0);
  Quatf qz(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2));
  return Eigen::Matrix<decimal_t, 3, 3>(qz * qy * qx);
}

/// Sort plannar points in the counter-clockwise order
inline std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> sort_pts(const std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> &pts) {
  /// if empty, dont sort
  if (pts.empty())
    return pts;
  /// calculate center point
  Eigen::Matrix<decimal_t, 2, 1> avg = Eigen::Matrix<decimal_t, 2, 1>::Zero();
  for (const auto &pt : pts)
    avg += pt;
  avg /= pts.size();

  /// sort in body frame
  vec_E<std::pair<decimal_t, Eigen::Matrix<decimal_t, 2, 1>>> pts_valued;
  pts_valued.resize(pts.size());
  for (unsigned int i = 0; i < pts.size(); i++) {
    decimal_t theta = atan2(pts[i](1) - avg(1), pts[i](0) - avg(0));
    pts_valued[i] = std::make_pair(theta, pts[i]);
  }

  std::sort(
      pts_valued.begin(), pts_valued.end(),
      [](const std::pair<decimal_t, Eigen::Matrix<decimal_t, 2, 1>> &i,
         const std::pair<decimal_t, Eigen::Matrix<decimal_t, 2, 1>> &j) { return i.first < j.first; });
  std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> pts_sorted(pts_valued.size());
  for (size_t i = 0; i < pts_valued.size(); i++)
    pts_sorted[i] = pts_valued[i].second;
  return pts_sorted;
}

/// Find intersection between two lines on the same plane, return false if they
/// are not intersected
inline bool line_intersect(const std::pair<Eigen::Matrix<decimal_t, 2, 1>, Eigen::Matrix<decimal_t, 2, 1>> &v1,
                           const std::pair<Eigen::Matrix<decimal_t, 2, 1>, Eigen::Matrix<decimal_t, 2, 1>> &v2, Eigen::Matrix<decimal_t, 2, 1> &pi) {
  decimal_t a1 = -v1.first(1);
  decimal_t b1 = v1.first(0);
  decimal_t c1 = a1 * v1.second(0) + b1 * v1.second(1);

  decimal_t a2 = -v2.first(1);
  decimal_t b2 = v2.first(0);
  decimal_t c2 = a2 * v2.second(0) + b2 * v2.second(1);

  decimal_t x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
  decimal_t y = (c1 * a2 - c2 * a1) / (a2 * b1 - a1 * b2);

  if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y))
    return false;
  else {
    pi << x, y;
    return true;
  }
}

/// Find intersection between multiple lines
inline std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> line_intersects(const vec_E<std::pair<Eigen::Matrix<decimal_t, 2, 1>, Eigen::Matrix<decimal_t, 2, 1>>> &lines) {
  std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> pts;
  for (unsigned int i = 0; i < lines.size(); i++) {
    for (unsigned int j = i + 1; j < lines.size(); j++) {
      Eigen::Matrix<decimal_t, 2, 1> pi;
      if (line_intersect(lines[i], lines[j], pi)) {
        pts.push_back(pi);
      }
    }
  }
  return pts;
}

/// Find extreme points of Polyhedron2D
inline std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> cal_vertices(const Polyhedron2D &poly) {
  vec_E<std::pair<Eigen::Matrix<decimal_t, 2, 1>, Eigen::Matrix<decimal_t, 2, 1>>> lines;
  const auto vs = poly.hyperplanes();
  for (unsigned int i = 0; i < vs.size(); i++) {
    Eigen::Matrix<decimal_t, 2, 1> n = vs[i].n_;
    Eigen::Matrix<decimal_t, 2, 1> v(-n(1), n(0));
    v = v.normalized();

    lines.push_back(std::make_pair(v, vs[i].p_));
    /*
    std::cout << "add p: " << lines.back().second.transpose() <<
      " v: " << lines.back().first.transpose() << std::endl;
      */
  }

  auto vts = line_intersects(lines);
  // for(const auto& it: vts)
  // std::cout << "vertice: " << it.transpose() << std::endl;

  std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> vts_inside = poly.points_inside(vts);
  vts_inside = sort_pts(vts_inside);

  return vts_inside;
}

/// Find extreme points of Polyhedron3D
inline vec_E<std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>>> cal_vertices(const Polyhedron3D &poly) {
  vec_E<std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>>> bds;
  const auto vts = poly.hyperplanes();
  //**** for each plane, find lines on it
  for (unsigned int i = 0; i < vts.size(); i++) {
    const Eigen::Matrix<decimal_t, 3, 1> t = vts[i].p_;
    const Eigen::Matrix<decimal_t, 3, 1> n = vts[i].n_;
    const Quatf q = Quatf::FromTwoVectors(Eigen::Matrix<decimal_t, 3, 1>(0, 0, 1), n);
    const Eigen::Matrix<decimal_t, 3, 3> R(q); // body to world
    vec_E<std::pair<Eigen::Matrix<decimal_t, 2, 1>, Eigen::Matrix<decimal_t, 2, 1>>> lines;
    for (unsigned int j = 0; j < vts.size(); j++) {
      if (j == i)
        continue;
      Eigen::Matrix<decimal_t, 3, 1> nw = vts[j].n_;
      Eigen::Matrix<decimal_t, 3, 1> nb = R.transpose() * nw;
      decimal_t bb = vts[j].p_.dot(nw) - nw.dot(t);
      Eigen::Matrix<decimal_t, 2, 1> v = Eigen::Matrix<decimal_t, 3, 1>(0, 0, 1).cross(nb).topRows<2>(); // line direction
      Eigen::Matrix<decimal_t, 2, 1> p;                                         // point on the line
      if (nb(1) != 0)
        p << 0, bb / nb(1);
      else if (nb(0) != 0)
        p << bb / nb(0), 0;
      else
        continue;
      lines.push_back(std::make_pair(v, p));
    }

    //**** find all intersect points
    std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> pts = line_intersects(lines);
    //**** filter out points inside polytope
    std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> pts_inside;
    for (const auto &it : pts) {
      Eigen::Matrix<decimal_t, 3, 1> p = R * Eigen::Matrix<decimal_t, 3, 1>(it(0), it(1), 0) + t; // convert to world frame
      if (poly.inside(p))
        pts_inside.push_back(it);
    }

    if (pts_inside.size() > 2) {
      //**** sort in plane frame
      pts_inside = sort_pts(pts_inside);

      //**** transform to world frame
      std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> points_valid;
      for (auto &it : pts_inside)
        points_valid.push_back(R * Eigen::Matrix<decimal_t, 3, 1>(it(0), it(1), 0) + t);

      //**** insert resulting polygon
      bds.push_back(points_valid);
    }
  }
  return bds;
}

/// Get the convex hull of a 2D points array, use wrapping method
inline std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> cal_convex_hull(const std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> &pts) {
  /// find left most point
  Eigen::Matrix<decimal_t, 2, 1> p0;
  decimal_t min_x = std::numeric_limits<decimal_t>::infinity();
  for (const auto &it : pts) {
    if (min_x > it(0) || (min_x == it(0) && it(1) < p0(1))) {
      min_x = it(0);
      p0 = it;
    }
  }

  std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> vs;
  vs.push_back(p0);

  while (vs.back() != p0 || vs.size() == 1) {
    const auto ref_pt = vs.back();
    Eigen::Matrix<decimal_t, 2, 1> end_pt = p0;
    for (size_t i = 0; i < pts.size(); i++) {
      if (pts[i] == ref_pt)
        continue;
      Eigen::Matrix<decimal_t, 2, 1> dir = (pts[i] - ref_pt).normalized();
      Hyperplane2D hp(ref_pt, Eigen::Matrix<decimal_t, 2, 1>(-dir(1), dir(0)));
      bool most_left_hp = true;
      for (size_t j = 0; j < pts.size(); j++) {
        if (hp.signed_dist(pts[j]) > 0 && pts[j] != pts[i] &&
            pts[j] != ref_pt) {
          // if(hp.signed_dist(pts[j]) > 0) {
          most_left_hp = false;
          break;
        }
      }

      if (most_left_hp) {
        end_pt = pts[i];
        break;
      }
    }
    // std::cout << "add: " << end_pt.transpose() << std::endl;
    vs.push_back(end_pt);
  }

  return vs;
}

inline Polyhedron2D get_convex_hull(const std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> &pts) {
  Polyhedron2D poly;
  Eigen::Matrix<decimal_t, 2, 1> prev_dir(-1, -1);
  for (size_t i = 0; i < pts.size() - 1; i++) {
    size_t j = i + 1;
    Eigen::Matrix<decimal_t, 2, 1> dir = (pts[j] - pts[i]).normalized();
    if (dir != prev_dir) {
      poly.add(Hyperplane2D((pts[i] + pts[j]) / 2, Eigen::Matrix<decimal_t, 2, 1>(-dir(1), dir(0))));
      prev_dir = dir;
    }
  }

  return poly;
}

/// Minkowski sum, add B to A with center Bc
inline Polyhedron2D minkowski_sum(const Polyhedron2D &A, const Polyhedron2D &B,
                                  const Eigen::Matrix<decimal_t, 2, 1> &Bc) {
  const auto A_vertices = cal_vertices(A);
  const auto B_vertices = cal_vertices(B);

  std::vector<Eigen::Matrix<decimal_t, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 2, 1>>> C_vertices;
  for (const auto &it : A_vertices) {
    for (const auto &itt : B_vertices)
      C_vertices.push_back(it + itt - Bc);
  }

  return get_convex_hull(cal_convex_hull(C_vertices));
}

#endif
