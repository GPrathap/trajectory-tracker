#ifndef _MPC_OPT_
#define _MPC_OPT_

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
#include <estimation_opt/kalman.h>


namespace hagen_planner
{
class MPCOpt
{
private:
  

public:
  
  EDTEnvironment::Ptr edt_env_;
  nav_msgs::Odometry current_projected_pose;
  ros::Publisher pos_current_pos_pub;
  ros::Subscriber odometry_sub_ref_, odometry_sub_real_;
  double maximum_acceptable_error = 0.4;
  int simulated_duration = 40;
  nav_msgs::Odometry odom;
  Eigen::Vector3d previos_pose;
  static double limit_vel_, limit_acc_, limit_ratio_;
  
  MPCOpt();
  ~MPCOpt() = default;

  void init(ros::NodeHandle& nh);
  void solver_init();
  void setEnvironment(const EDTEnvironment::Ptr& env);
  double getYawFromQuat(const geometry_msgs::Quaternion &data);
  std::shared_ptr<std::vector<double>> vehicle_current_state;
  ros::NodeHandle node_;

  double delta_t = 0.2;
  int prediction_horizon = 5;
  int estimation_horizon = 5;
  double robot_diam = 0.3;
  double v_max = 0.4;
  double v_min = -v_max;
  double omega_max = pi/4;
  double omega_min = -omega_max;
  double avoidance_distance = 0.7;
  bool still_running = false;
  int has_intermediate_goal =  false;
  double fluctuation_length = 2.0;
  int max_fluctuatio_allowed = 10;
  double obs_max_allowed_length = 10.0;
  double obs_min_allowed_length = 0.2;
  int collocation_degree = 3;
  bool use_collocation = false;


  Eigen::Vector3d origin_, map_size_;
  Eigen::Vector3d min_range_, max_range_;  // map range in pos

  PlanningVisualization::Ptr visualization_;

  bool force_terminate = false;
  bool need_intermediate_goal = false;
  
  template <typename T, typename Total, size_t N>
  class Cumulative_Sum
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

      // operator double() const { return total_/ std::min(num_samples_, N); }
      operator double() const { return total_; }

    private:
      T samples_[N];
      size_t num_samples_{0};
      Total total_{0};
  };

};
}  // namespace hagen_planner
#endif
