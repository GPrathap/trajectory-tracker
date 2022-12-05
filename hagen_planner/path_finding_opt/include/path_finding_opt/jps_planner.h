#ifndef JPS_PLANNER_BASE_H
#define JPS_PLANNER_BASE_H
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "graph_search.h"

#include <decom_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <decom_rviz_plugins/multi_detector.h>


class JPSPlanner
{
  public:
    
    JPSPlanner(bool verbose = false);
    int status();
    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getRawPath();
    std::vector<Eigen::Vector3d> removeLinePts(const std::vector<Eigen::Vector3d> &path);
    std::vector<Eigen::Vector3d> removeCornerPts(const std::vector<Eigen::Vector3d> &path);
    // void updateMap(vec_E<Polyhedron<3>>& polyhedrons);
    void setEnvironment(const hagen_planner::EDTEnvironment::Ptr& env);
    bool checkOccupiedConvex( Eigen::Matrix<int, 3, 1>& pn);
    bool plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, double eps = 1, bool use_jps = true);
    std::vector<Eigen::Vector3d> getOpenSet() const;
    std::vector<Eigen::Vector3d> getCloseSet() const;
    std::vector<Eigen::Vector3d> getAllSet() const;

    double total_distance(const std::vector<Eigen::Vector3d>& vs);
    bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt
    , Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);
    bool checkOccupancy(const Eigen::Vector3d &pos);

    bool isBlocked(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int8_t val = 100);
    std::vector<Eigen::Vector3i> rayTrace(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2);

  
    typedef std::shared_ptr<JPSPlanner> Ptr;
 
    std::shared_ptr<hagen_planner::GraphSearch> graph_search_;
    std::vector<Eigen::Vector3d> raw_path_;
    std::vector<Eigen::Vector3d> path_;
    int status_ = 0;
    bool planner_verbose_;
    std::vector<char> cmap_;
    vec_E<Polyhedron<3>> free_space;
    double res_ = 0.1;
};

typedef JPSPlanner JPSPlanner3D;

#endif
