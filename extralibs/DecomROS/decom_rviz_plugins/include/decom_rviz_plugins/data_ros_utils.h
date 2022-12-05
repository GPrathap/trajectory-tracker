#ifndef decom_rviz_plugins_H
#define decom_rviz_plugins_H

#include <decom_rviz_plugins/ellipsoid.h>
#include <decom_rviz_plugins/polyhedron.h>
#include <sensor_msgs/PointCloud.h>
#include <decom_ros_msgs/PolyhedronArray.h>
#include <decom_ros_msgs/EllipsoidArray.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>


namespace DecompROS {

template <int Dim> nav_msgs::Path vec_to_path(const vec_Vecf<Dim> &vs) {
  nav_msgs::Path path;
  for (const auto& it : vs) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = Dim == 2 ? 0 : it(2);
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}

inline sensor_msgs::PointCloud vec_to_cloud(const std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> &pts) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());

  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }
  return cloud;
}

inline std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }
  return pts;
}

inline std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> cloud_to_vec(pcl::PointCloud<pcl::PointXYZ> latest_cloud_) {
  std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> pts;
  pts.resize(latest_cloud_.points.size());
  for (unsigned int i = 0; i < latest_cloud_.points.size(); i++) {
    pts[i](0) = latest_cloud_.points[i].x;
    pts[i](1) = latest_cloud_.points[i].y;
    pts[i](2) = latest_cloud_.points[i].z;
  }
  return pts;
}

inline std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> cloud_to_vec(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
  std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> pts;
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud latest_cloud_;
  projector_.projectLaser(*scan_in, latest_cloud_);
  pts.resize(latest_cloud_.points.size());
  for (unsigned int i = 0; i < latest_cloud_.points.size(); i++) {
    pts[i](0) = latest_cloud_.points[i].x;
    pts[i](1) = latest_cloud_.points[i].y;
    pts[i](2) = 0.0;
  }
  return pts;
}

inline Polyhedron3D ros_to_polyhedron(const decom_ros_msgs::Polyhedron& msg){
  Polyhedron3D poly;
  for(unsigned int i = 0; i < msg.points.size(); i++){
    Eigen::Matrix<decimal_t, 3, 1> pt(msg.points[i].x,
             msg.points[i].y,
             msg.points[i].z);
    Eigen::Matrix<decimal_t, 3, 1> n(msg.normals[i].x,
            msg.normals[i].y,
            msg.normals[i].z);
    poly.add(Hyperplane3D(pt, n));
  }
  return poly;
}

inline vec_E<Polyhedron3D> ros_to_polyhedron_array(const decom_ros_msgs::PolyhedronArray& msg) {
  vec_E<Polyhedron3D> polys(msg.polyhedrons.size());

  for(size_t i = 0; i < msg.polyhedrons.size(); i++)
    polys[i] = ros_to_polyhedron(msg.polyhedrons[i]);

  return polys;
}

inline decom_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly){
  decom_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = 0;
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = 0;
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  geometry_msgs::Point pt1, n1;
  pt1.x = 0, pt1.y = 0, pt1.z = 0.01;
  n1.x = 0, n1.y = 0, n1.z = 1;
  msg.points.push_back(pt1);
  msg.normals.push_back(n1);

  geometry_msgs::Point pt2, n2;
  pt2.x = 0, pt2.y = 0, pt2.z = -0.01;
  n2.x = 0, n2.y = 0, n2.z = -1;
  msg.points.push_back(pt2);
  msg.normals.push_back(n2);

  return msg;
}

inline decom_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly){
  decom_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = p.p_(2);
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = p.n_(2);
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  return msg;
}


template <int Dim>
decom_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
  decom_ros_msgs::PolyhedronArray msg;
  for (const auto &v : vs)
    msg.polyhedrons.push_back(polyhedron_to_ros(v));
  return msg;
}

template <int Dim>
decom_ros_msgs::EllipsoidArray ellipsoid_array_to_ros(const vec_E<Ellipsoid<Dim>>& Es) {
  decom_ros_msgs::EllipsoidArray ellipsoids;
  for (unsigned int i = 0; i < Es.size(); i++) {
    decom_ros_msgs::Ellipsoid ellipsoid;
    auto d = Es[i].d();
    ellipsoid.d[0] = d(0);
    ellipsoid.d[1] = d(1);
    ellipsoid.d[2] = Dim == 2 ? 0:d(2);

    auto C = Es[i].C();
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 3; y++) {
        if(x < Dim && y < Dim)
          ellipsoid.E[3 * x + y] = C(x, y);
        else
          ellipsoid.E[3 * x + y] = 0;
      }
    }
    ellipsoids.ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids;
}

}

#endif
