#ifndef _PLANNING_PARAM_PASSER_H_
#define _PLANNING_PARAM_PASSER_H_

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
class ParamPasser
{
private:
  /* data */
  ros::NodeHandle node;

public:
  ParamPasser(/* args */)
  {
  }
  ~ParamPasser()
  {
  }

  ParamPasser(ros::NodeHandle& nh);

  std::string root_dir ="/";
  bool passing_matrix(std::string param_name, Eigen::MatrixXd& mat);
  bool passing_vector(std::string param_name, Eigen::MatrixXd& mat);
  typedef std::shared_ptr<ParamPasser> Ptr;
  
};
}  // namespace hagen_planner
#endif

