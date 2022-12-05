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
#include <traj_common/planning_saving.h>
#include <map_building_opt/edtoctomap.h>
#include <map_building_opt/edt_environment.h>
#include "state_machine/Bspline.h"
#include "mpc_opt/nonlinear_mpc_opt.h"
#include "mpc_opt/trajectory_tracker.h"
#include "nav_msgs/Odometry.h"
#include "hagen_msgs/PoseCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/TwistStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <deque>
#include <math.h>
#include <tf/tf.h>
#include <queue>
#include <traj_common/bspline_utils.h>
#include <rebound_opt/rebound_optimizer.h>
#include <state_machine/backward.hpp>
#include <boost/circular_buffer.hpp>
#include <path_finding_opt/dyn_a_star.h>
#include <path_finding_opt/jps_planner.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/move/move.hpp>

using std::vector;

namespace hagen_planner
{
class FSM_Trajectory_Tracker
{
private:
  bool trigger_, have_trajector_;
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
  BSplineUtils::Ptr bspline_utils_;
  PlanningVisualization::Ptr visualization_;
  TrajectoryTracker::Ptr trajectroy_tracker;
  hagen_planner::BsplineOptimizer::Ptr rebound_optimizer;
  double thresh_no_replan_, thresh_replan_;

  bool rc_sdk_enable_ = false;
  bool rc_sdk_was_enabled = true;
  std::atomic_bool control_command_sending_is_allowed;
  bool suddern_stop_is_set = false; 

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, inter_end_pt_;
  int current_wp_;
  bool stop_execution = false;
  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_, cmd_timer_;
  ros::Timer vis_timer_, query_timer_;

  ros::Subscriber waypoint_sub_, odometry_sub_, rc_sub, vehicle_current_pose_sub_
  , stop_execution_sub_, continue_execution_sub_, obstacle_poses_sub_, lidar_poses_2d_sub_;

  ros::Publisher replan_pub_, bspline_pub_, wait_for_goal, stat_moving, stop_moving, pos_cmd_pub, state_pub;

  void execFSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void currentPoseCallback(const nav_msgs::OdometryConstPtr& msg);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  double getYawFromQuat(const geometry_msgs::Quaternion &data);
  void stopExecutionCallback(const std_msgs::Empty msg);
  void continueExecutionCallback(const std_msgs::Empty msg);
  void detectedObs(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void laserScan(const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void saveTrajectory(const ros::TimerEvent& e);

public:
  FSM_Trajectory_Tracker(/* args */)
  {
  }
  ~FSM_Trajectory_Tracker()
  {
  }

  void init(ros::NodeHandle& nh);
  bool intermidiate_goal_is_set = false;
  std::atomic_bool planner_status;
  
  void solverThread();
  std::thread startSolverThread();

  void GlobalTrajectoryThread();
  std::thread startGlobalTrajectoryThread();

  void fsmExecutor();
  std::thread execFSMThread();

  void cmdExecutor();
  std::thread execCMDThread();
  
  Eigen::Vector3d stop_pose;
  vector<vector<double>> trees, trees_real;
  hagen_msgs::PoseCommand cmd;
  nav_msgs::Odometry odom;
  nav_msgs::Odometry current_pose;
  sensor_msgs::PointCloud laser_cloud;
  boost::circular_buffer<Eigen::Vector3d>* traj_cmd;
  boost::circular_buffer<Eigen::Vector3d>* traj_real;
  bool is_allowed_for_execution = true;
  std::mutex lock_on_solver; 
  std::mutex lock_on_global_solver; 
  std::condition_variable condition_on_solver;
  std::condition_variable condition_on_global_solver;
  bool granted_execution = false;
  bool granted_execution_global = false;
  std::thread solver_thread;
  std::thread solver_global_thread;
  std::thread fsm_thread;
  std::thread cmd_thread;
  bool stop_pose_init = false;
  laser_geometry::LaserProjection projector_;
  double current_yaw = 0;
  double stop_yaw_angle = 0;
  int sampling_rate = 30;
  double avoidance_distance = 0.3;
  bool has_intermeditate_goal = false;
  bool had_intermeditate_goal = false;
  std::ofstream outfile_waypoints;
  std::ofstream outfile_bspline;
  std::ofstream outfile_safeset;
  std::string waypoints_file = "";
  std::string bspline_file = "";
  std::string safeset_file = "";


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
  bool init_kf_yaw =  false;
  double yaw_sign = 1;
  double previous_yaw = 0;
};

}  // namespace hagen_planner

#endif