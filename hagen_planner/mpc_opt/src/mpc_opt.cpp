#include "mpc_opt/nonlinear_mpc_opt.h"
#include <ros/ros.h>

namespace hagen_planner
{
  MPCOpt::MPCOpt()
  {
    
  }

  void MPCOpt::solver_init(){

  }

  double MPCOpt::getYawFromQuat(const geometry_msgs::Quaternion &data)
  {
    tf::Quaternion quat(data.x, data.y, data.z, data.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void MPCOpt::setEnvironment(const EDTEnvironment::Ptr& env){
    edt_env_ = env;
  }

  void MPCOpt::init(ros::NodeHandle& nh){
      node_ = nh;
      /* ---------- param ---------- */
      node_.param("sdf_map/origin_x", origin_(0), -20.0);
      node_.param("sdf_map/origin_y", origin_(1), -20.0);
      node_.param("sdf_map/origin_z", origin_(2), 0.0);

      node_.param("sdf_map/map_size_x", map_size_(0), 40.0);
      node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
      node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

      node_.param("mpc_opt/delta_t", delta_t, 0.2);
      node_.param("mpc_opt/prediction_horizon", prediction_horizon, 5);
      node_.param("mpc_opt/estimation_horizon", estimation_horizon, 5);
      node_.param("mpc_opt/v_max", v_max, 0.4);
      node_.param("mpc_opt/v_min", v_min, -0.4);
      node_.param("mpc_opt/omega_max", omega_max, pi);
      node_.param("mpc_opt/omega_min", omega_min, -pi);
      node_.param("mpc_opt/robot_diam", robot_diam, 0.3);
      node_.param("mpc_opt/avoidance_distance", avoidance_distance, 0.3);
      node_.param("mpc_opt/maximum_acceptable_error", maximum_acceptable_error, 0.3);
      node_.param("mpc_opt/simulated_duration", simulated_duration, 40);
      node_.param("mpc_opt/fluctuation_length", fluctuation_length, 3.0);
      node_.param("mpc_opt/max_fluctuatio_allowed", max_fluctuatio_allowed, 4);
      node_.param("mpc_opt/obs_min_allowed_length", obs_min_allowed_length, 0.2);
      node_.param("mpc_opt/obs_max_allowed_length", obs_max_allowed_length, 10.0);
      node_.param("mpc_opt/collocation_degree", collocation_degree, 3);
      node_.param("mpc_opt/use_collocation", use_collocation, false);

      pos_current_pos_pub = node_.advertise<nav_msgs::Odometry>("/planning/current_state", 5);
      // closest_obs_poses = node_.advertise<nav_msgs::Odometry>("/planning/current_state", 5);
      min_range_ = origin_;
      max_range_ = origin_ + map_size_;
  }
}  // namespace hagen_planner
