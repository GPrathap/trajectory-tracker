#ifndef _KGB_TRAJECTORY_GENERATOR_H_
#define _KGB_TRAJECTORY_GENERATOR_H_

#include <deque>
#include <ros/ros.h>
#include <spline_opt/non_uniform_bspline.h>
#include <unsupported/Eigen/Splines>
#include <math.h>
#include <traj_common/planning_visualization.h>

namespace hagen_planner
{
class BSplineUtils
{
private:
  double time_sample_;
  double max_vel_;

  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;

  /* helper function */
  Eigen::Vector3d getFarPoint(const vector<Eigen::Vector3d>& path, Eigen::Vector3d x1, Eigen::Vector3d x2);

public:
  BSplineUtils()
  {
  }
  ~BSplineUtils();

  /* generated traj */
  int traj_id_;
  double traj_duration_, t_start_, t_end_, margin_, time_start_;
  ros::Time time_traj_start_;
  Eigen::Vector3d pos_traj_start_;
  NonUniformBspline traj_pos_, traj_vel_, traj_acc_;
  /* guided optimization */
  NonUniformBspline traj_init_;
  PlanningVisualization::Ptr visualization_;
  int current_state = 0;
  Eigen::MatrixXd waypoints_list_;
  int smoothing_factor = 100;
  int smoothing_order = 2;
  bool generateTrajectory(std::deque<Eigen::Vector3d> waypoints_list, int id_);  // front-end && back-end
  bool generateTrajectory(Eigen::MatrixXd& waypoints_list, int id_);
  void retrieveTrajectory();

  void setParam(ros::NodeHandle& nh);
  bool getIntermediatePoints(double delta_t_desired, int num_points, Eigen::MatrixXd& projected);
  bool downsample(int num_points, std::vector<Eigen::Vector3d>& horio_, Eigen::MatrixXd& projected_trajectory_);
  Eigen::MatrixXd control_points;

  /* ---------- evaluation ---------- */
  void getSolvingTime(double& ts, double& to, double& ta);
  typedef shared_ptr<BSplineUtils> Ptr;
};
}  // namespace hagen_planner

#endif