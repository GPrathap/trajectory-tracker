#ifndef _rebound_optIMIZER_H_
#define _rebound_optIMIZER_H_

#include <Eigen/Eigen>
#include <rebound_opt/uniform_bspline.h>
#include <map_building_opt/edt_environment.h>
#include <ros/ros.h>
#include <path_finding_opt/dyn_a_star.h>
#include <traj_common/planning_visualization.h>
#include <path_finding_opt/jps_planner.h>
#include <path_finding_opt/timer.hpp>
#include <decom_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <iostream>
#include "LBFGSB.h"


namespace hagen_planner
{
  #define nint1(a)  monty::new_array_ptr<int>({(a)})
  #define nint(a,b) monty::new_array_ptr<int>({(a),(b)})
  
  class ControlPoints
  {
  public:
    double clearance;
 
    int size;
    Eigen::MatrixXd points;
    // Eigen::MatrixXd points_tmp;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
    }
  };

  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    void setEnvironment(const hagen_planner::EDTEnvironment::Ptr &env);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);
    
    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init);

    bool MoveBack(Eigen::Vector3d start_pt, Eigen::Vector3d& start_free, int index, int& final_index);
    bool MoveForward(Eigen::Vector3d start_pt, Eigen::Vector3d& start_free, int index, int& final_index);

    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);
    void calPoly(vec_E<Polyhedron<3>> polyhedrons,  std::vector<Eigen::MatrixXd>& filtered_);
    bool refineDeadZone(int steps, Eigen::Vector3d start_pose, Eigen::Vector3d end_pose, Eigen::MatrixXd& refined_points);
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void save_matrix(std::vector<std::vector<Eigen::Vector3d>> data, std::string file_name);

    void optimize();
    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad);
    void combineCostRebound(const Eigen::VectorXd& x, Eigen::VectorXd& grad, double &f_combine);
    bool rebound_optimize_g1();
    bool calculateOptimalPoints(Eigen::MatrixXd& a, Eigen::MatrixXd& b
                    , Eigen::MatrixXd& p_init, Eigen::MatrixXd& refined_points);

    Eigen::MatrixXd getControlPoints();

    void getJSPPath(Eigen::Vector3d in, Eigen::Vector3d out,  std::vector<Eigen::Vector3d>& path);

    AStar::Ptr a_star_;
    std::atomic_bool planner_status;
    PlanningVisualization::Ptr visualization_;
    int max_points = 8;
    std::vector<Eigen::Vector3d> ref_pts_;

    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts, Eigen::Vector3d start_odom, int& current_pose_obs); // must be called after initControlPoints()

    inline int getOrder(void) { return order_; }
    hagen_planner::EDTEnvironment::Ptr grid_map_;
    JPSPlanner::Ptr jsp_planner;
    Eigen::MatrixXd horizon_trajectory;
    int seg_init = 0;
    int seg_end = 0;
    int current_pose_obs = 0;
    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    int previous_index_in = 0;
    int previous_index_out = 0;
    int times_segs = 3;

    
    
  private:
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting

    int a;
    double dist0_;             // safe distance
    double max_vel_, max_acc_; // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
    std::vector<Eigen::Vector3d> map_range;
    Eigen::Vector3d min_map_range;
    Eigen::Vector3d max_map_range;
    
    ControlPoints cps_;
    std::string root_dir;

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = false);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();

    /* for benckmark evaluation only */
  public:
    typedef shared_ptr<BsplineOptimizer> Ptr;
    Eigen::Vector3d start_odom;
    bool start_odom_set = false;
    vec_E<Polyhedron<3>> polyhedrons;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace hagen_planner
#endif