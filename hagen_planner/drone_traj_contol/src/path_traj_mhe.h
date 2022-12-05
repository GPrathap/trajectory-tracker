#ifndef DRONE_REG_MHE
#define DRONE_REG_MHE

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <drone_traj_contol/DroneRegConfig.h>
#include <std_msgs/Float32.h>
#include <hagen_msgs/PoseCommand.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <casadi/casadi.hpp>
// #include <cnpy.h>
#include <traj_common/planning_visualization.h>
#include <boost/circular_buffer.hpp>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

using namespace casadi;
using namespace std;

namespace hagen_planner
{
class NonLinearMHEOpt
{
private:
  int n_states;
  int n_controls;
  std::map<std::string, DM> args_mhe, res_mhe;
  SXDict nlp_prob_mhe;
  Dict opts_mhe;

  SX states;
  SX controls;
  SX rhs;
  Function f;
  SX g;
  SX X;
  SX mhe_g;
  SX con_cov;
  SX meas_cov;
  SX V;
  SX W;
  vector<vector<double>> obs_map;
  int obs_length;
  Function solver_mhe;
  DM x0_mhe_init;
  DM u0_mhe_init;
  DM p_mhe_init;

  DM x0_mhe_init_previous;
  DM u0_mhe_init_previous;
  DM p_mhe_init_previous;


  std::shared_ptr<boost::circular_buffer<double>> x0_mhe;
  std::shared_ptr<boost::circular_buffer<double>> u0_mhe;
  std::shared_ptr<boost::circular_buffer<double>> p_u_mhe;
  std::shared_ptr<boost::circular_buffer<double>> p_x_mhe;


  hagen_msgs::PoseCommand  current_projected_pose;
  ros::Publisher pos_current_pos_pub;
  ros::Subscriber odometry_pose_sub, pos_current_sub, restart_estimator_sub, wait_for_goal_sub;
  double maximum_acceptable_error = 0.4;
  int simulated_duration = 40;
  nav_msgs::Odometry odom;
  bool init_buffer = false;
  bool is_allowed_for_execution_estimator = true;
  std::mutex lock_on_estimator; 
  std::condition_variable condition_on_estimator;
  bool granted_execution_estimator = true;
  std::thread estimator_thread;

public:
  static double limit_vel_, limit_acc_, limit_ratio_;

  NonLinearMHEOpt();
  ~NonLinearMHEOpt() = default;

  void init(ros::NodeHandle& nh);
  void solver_init();
  void mhe_estimator_init();
  void mhe_estimator();
  void restart_estimator(std_msgs::Empty msg); 
  tuple<double, SX, SX> shift(double T, double t0, SX x0, SX u, Function f);
//   void save_dm(DM trees, std::string file_name, int index);
//   void setEnvironment(const EDTEnvironment::Ptr& env);
  void actual_vehilce_odometry(const nav_msgs::Odometry& msg);
  void get_current_state(const hagen_msgs::PoseCommand& odom);
  double getYawFromQuat(const geometry_msgs::Quaternion &data);
  // void save_dm(DM trees, std::string file_name, int index);
  // void save_vector(vector<double> trees, std::string file_name, int index);
  void start_moving(std_msgs::Empty mes);
  void estimaorThread();
  std::thread startEstimatorThread();
  shared_ptr<vector<double>> vehicle_current_state;
  ros::NodeHandle node_;



  double delta_t = 0.2;
  int prediction_horizon = 5;
  int estimation_horizon = 5;
  double robot_diam = 0.3;
  double v_max = 0.4;
  double v_min = -v_max;
  double omega_max = pi/4;
  double omega_min = -omega_max;
  vector<double> map_dim = {-6, 6, -6, 6, 0, 6};
  double avoidance_distance = 0.7;
  std::shared_ptr<vector<double>> current_estimated_state;
  std::shared_ptr<vector<double>> current_estimated_state_previous;

  bool still_running = false;
  

  Eigen::Vector3d origin_, map_size_;
  Eigen::Vector3d min_range_, max_range_;  // map range in pos

  
  typedef std::shared_ptr<NonLinearMHEOpt> Ptr;
  PlanningVisualization::Ptr visualization_;

  bool force_terminate = false;
  bool stop_estimating = false;
  
  DM x0;
  DM xs;

};
}  // namespace hagen_planner
#endif



