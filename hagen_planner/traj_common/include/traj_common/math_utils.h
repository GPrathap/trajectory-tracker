#ifndef _PLANNING_MATHS_UTILS_H_
#define _PLANNING_MATHS_UTILS_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <spline_opt/non_uniform_bspline.h>
#include <casadi/casadi.hpp>
#include <cnpy.h>
#include <XmlRpcException.h>


using std::vector;
using namespace casadi;
namespace hagen_planner
{
class MathUtils
{
private:
  /* data */
  ros::NodeHandle node;

public:
  MathUtils(/* args */)
  {
  }
  ~MathUtils()
  {
  }

  std::string root_dir ="/";
  std::vector<Eigen::Vector3d> next_poses(Eigen::VectorXd start_position, Eigen::VectorXd end_position
      , double distance);
  std::deque<double> linsplit(double start_in, double end_in, double number_of_steps);
  std::vector<double> linspace(double start_in, double end_in, double number_of_steps);
  void get_squar_segments(Eigen::VectorXd info, std::vector<Eigen::Vector3d>& points_on_squar);
  void get_segments(double cx, double cy, double min_dis, double division, std::vector<double>& pose_x, std::vector<double>& pose_y);
  typedef std::shared_ptr<MathUtils> Ptr;
  
};
}  // namespace hagen_planner
#endif

