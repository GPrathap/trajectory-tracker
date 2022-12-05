#ifndef _PLANNING_FSM_GROUP_H_
#define _PLANNING_FSM_GROUP_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <traj_common/planning_visualization.h>
#include <map_building_opt/edtoctomap.h>
#include <map_building_opt/edt_environment.h>
#include "state_machine/Bspline.h"
#include "mpc_opt/nonlinear_mpc_opt.h"
#include "mpc_opt/trajectory_tracker.h"
#include "mpc_opt/trajectory_regulator.h"
#include <traj_common/planning_saving.h>
#include "nav_msgs/Odometry.h"
#include "hagen_msgs/PoseCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/TwistStamped.h>
#include <ros/callback_queue.h>
#include <deque>
#include <math.h>
#include <tf/tf.h>
#include <queue>
#include <state_machine/backward.hpp>
#include <traj_common/bspline_utils.h>
#include <boost/circular_buffer.hpp>

using std::vector;

namespace hagen_planner
{
class FSM_Trajectory_Point_Stabilizer
{
private:
  bool trigger_, have_goal_;
  enum EXEC_STATE
  {
    WAIT_GOAL,
    EXEC_TRAJ
  };
  EXEC_STATE exec_state_;

  void changeExecState(EXEC_STATE new_state, string pos_call);
  void printExecState();

  EDTOctoMap::Ptr sdf_map_;
  EDTEnvironment::Ptr edt_env_;
  PlanningVisualization::Ptr visualization_;
  TrajectoryRegulator::Ptr trajectroy_regulator;

  double thresh_no_replan_, thresh_replan_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, inter_end_pt_;
  int current_wp_;
  bool stop_execution = false;
  int retry_generate_cout = 0;
  int retry_generate_cout_max = 3;

  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_, cmd_timer_;
  ros::Timer vis_timer_, query_timer_;

  ros::Subscriber waypoint_sub_, odometry_sub_, rc_sub, vehicle_current_pose_sub_, stop_execution_sub_, continue_execution_sub_;

  ros::Publisher replan_pub_, bspline_pub_, wait_for_goal, stat_moving, stop_moving, pos_cmd_pub, state_pub;

  void execFSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void currentPoseCallback(const nav_msgs::OdometryConstPtr& msg);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  double getYawFromQuat(const geometry_msgs::Quaternion &data);
  void currOdometrylback( nav_msgs::Odometry& msg);
  void stopExecutionCallback(const std_msgs::Empty msg);
  void continueExecutionCallback(const std_msgs::Empty msg);

public:
  FSM_Trajectory_Point_Stabilizer(/* args */)
  {
  }
  ~FSM_Trajectory_Point_Stabilizer()
  {
  }

  void init(ros::NodeHandle& nh);
  bool intermidiate_goal_is_set = false;
  
  void solverThread();
  std::thread startSolverThread();

  void fsmExecutor();
  std::thread execFSMThread();

  void cmdExecutor();
  std::thread execCMDThread();
  
  Eigen::Vector3d stop_pose;
  Eigen::VectorXd intermediate_stop_pose;
  vector<vector<double>> trees, trees_real;
  hagen_msgs::PoseCommand cmd;

  std::mutex mutex_odom; 
  std::mutex mutex_current_pose; 
  std::mutex mutex_goal_poses; 

  nav_msgs::Odometry odom;
  nav_msgs::Odometry current_pose;
  boost::circular_buffer<Eigen::Vector3d>* traj_cmd;
  boost::circular_buffer<Eigen::Vector3d>* traj_real;
  
  bool is_allowed_for_execution = true;
  std::mutex lock_on_solver; 
  std::condition_variable condition_on_solver;
  std::condition_variable condition_on_odom;
  std::condition_variable condition_on_current_pose;
  bool granted_execution = false;
  bool granted_execution_current_pose = false;
  bool granted_execution_odom = false;
  std::thread solver_thread;
  std::thread fsm_thread;
  std::thread cmd_thread;
  bool stop_pose_init = false;
  double current_yaw = 0;
  double stop_yaw_angle = 0;



  // double current_yaw = 0;
  int sampling_rate = 30;
  double avoidance_distance = 0.3;
  double avoidance_distance_max = 1.0;
  double avoidance_distance_intermediate = 0;
  double cone_outer_radius = 1.0;
  double cone_inner_radius = 1.0;

  double intermediate_goal_max_dis = 3.0;
  bool has_intermeditate_goal = false;
  bool had_intermeditate_goal = false;

  Eigen::Vector2d next_heading;
  Eigen::Vector2d current_heading;

  std::deque<Eigen::Vector3d> waypoints_list;
  template <typename T, typename Total, size_t N>
  class Moving_Average
  {
    public:
      void operator()(T sample)
      {
          if (num_samples_ < N)
          {
              samples_[num_samples_++] = sample;
              total_ += sample;
          }
          else
          {
              T& oldest = samples_[num_samples_++ % N];
              total_ += sample - oldest;
              oldest = sample;
          }
      }

      operator double() const { return total_ / std::min(num_samples_, N); }

    private:
      T samples_[N];
      size_t num_samples_{0};
      Total total_{0};
  };

  Moving_Average<double, double, 20> mov_fil;
  KalmanFilter* kf_yaw;
 
  int n_states = 3;
  int n_controls = 3;
  bool init_kf_yaw =  false;
  bool init_kf_velocity = false;
  double previous_yaw = 0;

};

}  // namespace hagen_planner

#endif