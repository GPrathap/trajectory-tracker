#ifndef LINEARIZED_MPC_TRAJECORY_TRACKER
#define LINEARIZED_MPC_TRAJECORY_TRACKER
#include <unistd.h>

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
#include <decom_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <decom_rviz_plugins/multi_detector.h>
#include <ctime>
#include <traj_common/corridor.h>
#include "mpc_opt/mpc_solver.h"
#include <motion_model/spatial_bicycle_models.h>
#include "OsqpEigen/OsqpEigen.h"
#include <ctime>
#include "mpc_opt/linearized_mpc_opt.h"

using namespace casadi;
using namespace std;

namespace hagen_planner
{
class LinearizedMPCTrajectoryTracker: public  LinearizedMPCOpt
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
  OsqpEigen::Solver* osqpInterface_control;
  
  Eigen::SparseMatrix<c_float> P;
  bool set_hassian_mat = false;
  
public:
   LinearizedMPCTrajectoryTracker();
  ~LinearizedMPCTrajectoryTracker() = default;

  void solver_init();
  void init(ros::NodeHandle& nh);
  void mpc_solver();
  void mpc_solver_with_multiple_shooting();
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void odomCallbackReal(const nav_msgs::OdometryConstPtr& msg);
  void setTrajectoryGenerator(const BSplineUtils::Ptr& manager);
  void setBoundaryChecker(const BsplineOptimizer::Ptr& manager);
  void setIntermediateTrajectoryGenerator(const BSplineUtils::Ptr& manager);
  double get_distance(Eigen::MatrixXd norminal, Eigen::MatrixXd actual);
  bool getControl(int index);
  

 
  
  void get_free_space(vec_E<Polyhedron<3>>& polyhedrons, std::vector<Eigen::Vector3d>& waypoints, std::vector<std::vector<Eigen::Vector3d>>& projected_space);

  void show_free_space(std::vector<Eigen::Vector3d>& projected_trajectory_,  std::vector<Eigen::MatrixXd>& a_ref, std::vector<Eigen::MatrixXd>& b_ref, int& total_cout);
  
  void showTunnel(vec_E<Polyhedron<3>>& polyhedrons, const Eigen::Vector3d& position
        , const Eigen::Vector3d& tangent_line, Polyhedron<3>& poly_tunnel);
  
  void calculateCost(double theta, int horizon_, Eigen::Vector3d pos, Eigen::Vector3d vel);
  bool calculateConstrains(Eigen::MatrixXd& horizon_control_points, vec_E<Polyhedron<3>>& free_space_
                  , Eigen::SparseMatrix<double>& final_cons_set, Eigen::SparseMatrix<double>& sparse_b
                  , Eigen::SparseMatrix<double>& final_cons_b_low);
  // int solveMpcQp(Eigen::SparseMatrix<double> &stateTmp, Eigen::SparseMatrix<double> &end_state, std::vector<Eigen::Vector3d>& projected_path
  //                                                           ,  vec_E<Polyhedron<3>>& polyhedrons);
  void createTunnel(const Eigen::Vector3d& position
        , const Eigen::Vector3d& tangent_line_, Polyhedron<3>& poly_tunnel, int index, Eigen::MatrixXd& corridor_ref);

  void calPoly(vec_E<Polyhedron<3>> polyhedrons, std::vector<Eigen::MatrixXd>& filtered_);
 
  void printPolyhedron(Polyhedron<3>& poly_tunnel);
  Eigen::VectorXd get_eiegn_vec(Eigen::SparseMatrix<double> sparse_mat);
  void find_free_space(Eigen::MatrixXd& projected_trajectory_);
  bool get_next_waypoints(Eigen::MatrixXd& poses, int count, Eigen::Vector3d& pose);
  void global_solver();
  void updatePrediction(int starting_index);
  void visualizeStatePediction();

  void init_problme(int starting_index, OsqpEigen::Solver& solver);
  
  
  void setInitModel(int starting_index);
  
  
  typedef std::shared_ptr<LinearizedMPCTrajectoryTracker> Ptr;
  BSplineUtils::Ptr bspline_utils_;
  BsplineOptimizer::Ptr bound_checker;
  BSplineUtils::Ptr bspline_intermediate_;
  PlanningSaving::Ptr planner_saving;
  ParamPasser::Ptr passer;
  nav_msgs::Odometry reference_odom;
  nav_msgs::Odometry real_odom;
  bool is_reference_trj_set = false;
  bool is_real_odom_set = false;
  Eigen::SparseMatrix<double> x0;
  Eigen::SparseMatrix<double> u0;
  Eigen::SparseMatrix<double> xs;
  
  Eigen::Vector3d current_odom_pose;
  bool set_new_path = false;
  int start_index;
  int end_index;
  double current_time;
  bool still_running_global_planner = true;
  bool set_running_global_planner = false;
  // FreeSpaceSegmentation1 free_seg;
  Eigen::MatrixXd horizon_control_points_total;
  ros::Time time_traj_start;
  Eigen::MatrixXd tempory_data;
 
  bool global_opt_sudden_stop = false;
  int current_pose_obs = 0;
  double qVTheta = 0.005;
  double thetaMax = 3.0;
  double v_max = 2;
  double delta_max = 0.66;
  double ay_max = 4.0;
  double a_min = -1.4;
  double a_max = 1.4;

   // Model model;
  BicycleModel model;
  
  Eigen::SparseMatrix<double> xmin_dyn;
  Eigen::SparseMatrix<double> xmax_dyn;
  Eigen::SparseMatrix<double> umax_dyn;
  
  
  std::vector<Eigen::Vector3d> rebound_array;
  Eigen::Vector2d current_u;


};
}  // namespace hagen_planner
#endif
