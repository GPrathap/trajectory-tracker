#ifndef _NON_LINEAR_TRAJECTORY_TRACKER_OPT_
#define _NON_LINEAR_TRAJECTORY_TRACKER_OPT_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <casadi/casadi.hpp>
#include <map_building_opt/edt_environment.h>
#include <traj_common/planning_visualization.h>
#include <traj_common/planning_saving.h>
#include <traj_common/param_passer.h>
#include <traj_common/bspline_utils.h>
#include <rebound_opt/rebound_optimizer.h>
#include <boost/circular_buffer.hpp>
#include <tf/tf.h>
#include <time.h>
#include "mpc_opt/nonlinear_mpc_opt.h"
#include <decom_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <decom_rviz_plugins/multi_detector.h>
// #include <cstdlib>
#include <ctime>
// #include "../src/free_space/free_space_segmentation.h"
// #include "mpc_opt/corridor.h"
#include "mpc_opt/mpc_solver.h"



using namespace casadi;
using namespace std;


namespace hagen_planner
{
class TrajectoryTracker : public  NonLinearMPCOpt
{
private:
  double delta_t_desired = 0.02;
  KalmanFilter* kf; 
  // KalmanFilter* kf_position;
  KalmanFilter* kf_nmpc;
  std::string solver_state_error = "Infeasible_Problem_Detected";
  boost::circular_buffer<Eigen::Vector3d>* traj_cmd;
  int reference_trj_horizon = 40;

  std::mutex lock_on_local_to_global; 
  std::condition_variable condition_lock_on_local_to_global;
  bool refine_trajctory_ = false;

  
public:
   TrajectoryTracker();
  ~TrajectoryTracker() = default;

  void solver_init();
  void init(ros::NodeHandle& nh);
  void mpc_solver();
  void mpc_solver_with_collocation();
  void mpc_solver_with_multiple_shooting();
  void global_solver();
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void odomCallbackReal(const nav_msgs::OdometryConstPtr& msg);
  void setTrajectoryGenerator(const BSplineUtils::Ptr& manager);
  void setBoundaryChecker(const hagen_planner::BsplineOptimizer::Ptr& manager);
  void setIntermediateTrajectoryGenerator(const BSplineUtils::Ptr& manager);
  double get_distance(Eigen::MatrixXd norminal, Eigen::MatrixXd actual);
  

  void get_free_space(vec_E<Polyhedron<3>>& polyhedrons
                      , std::vector<Eigen::Vector3d>& waypoints
                      , std::vector<std::vector<Eigen::Vector3d>>& projected_space);

 void show_free_space(std::vector<Eigen::Vector3d>& projected_trajectory_
                      ,  std::vector<Eigen::MatrixXd>& a_ref, std::vector<Eigen::MatrixXd>& b_ref, int& total_cout);
  
  void showTunnel(vec_E<Polyhedron<3>>& polyhedrons, const Eigen::Vector3d& position
        , const Eigen::Vector3d& tangent_line, Polyhedron<3>& poly_tunnel);
  
  void find_free_space(Eigen::MatrixXd& projected_trajectory_);
     
  
  typedef std::shared_ptr<TrajectoryTracker> Ptr;
  BSplineUtils::Ptr bspline_utils_;
  hagen_planner::BsplineOptimizer::Ptr bound_checker;
  BSplineUtils::Ptr bspline_intermediate_;
  PlanningSaving::Ptr planner_saving;
  ParamPasser::Ptr passer;
  nav_msgs::Odometry reference_odom;
  nav_msgs::Odometry real_odom;
  bool is_reference_trj_set = false;
  bool is_real_odom_set = false;
  bool set_new_path = false;
  Eigen::Vector3d current_odom_pose;
  int start_index;
  int end_index;
  double current_time;
  bool still_running_global_planner = true;
  bool set_running_global_planner = false;
  // FreeSpaceSegmentation1 free_seg;
  hagen_planner::MpcSolver convex_mpc_solver;
  Eigen::MatrixXd horizon_control_points_total;
  ros::Time time_traj_start;
  int current_pose_obs = 0 ;
  bool global_opt_sudden_stop = false;
  std::string residual_dynamics_model = "";
  double delta_t_solver = 0.05;
  
};
}  // namespace hagen_planner
#endif
