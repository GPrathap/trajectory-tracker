#ifndef _NON_LINEAR_MPC_OPT_
#define _NON_LINEAR_MPC_OPT_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <casadi/casadi.hpp>
#include <map_building_opt/edt_environment.h>
#include <traj_common/planning_visualization.h>
#include <boost/circular_buffer.hpp>
#include <tf/tf.h>
#include <time.h>
#include "mpc_opt.h"

using namespace casadi;
namespace hagen_planner
{
class NonLinearMPCOpt : public  MPCOpt
{
private:
  

public:
  int n_states;
  int n_controls;
  std::map<std::string, DM> args, res, args_mhe, res_mhe;
  SXDict nlp_prob;
  Dict opts;

  MX states;
  MX controls;
  MX rhs;
  Function f;
  SX g;
  SX X;
  std::vector<std::vector<double>> obs_map;
  int obs_length;
  SX obj = 0;

  NonLinearMPCOpt();
  ~NonLinearMPCOpt() = default;

  void init(ros::NodeHandle& nh);
  void solver_init();
  std::tuple<double, SX, SX> shift(double T, double t0, SX x0, SX u, Function f);
  void setEnvironment(const EDTEnvironment::Ptr& env);
  double getYawFromQuat(const geometry_msgs::Quaternion &data);
  void getCollocationPoints(std::vector<double>& B, std::vector<std::vector<double>>& C, std::vector<double>& D);

 
  

  std::vector<std::vector<double>> C;
  std::vector<double> D;
  std::vector<double> B;
  
  DM x0;
  DM xs;
  DM u0;
};
}  // namespace hagen_planner
#endif
