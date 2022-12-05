#ifndef REFERENCE_STATE
#define REFERENCE_STATE

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"


namespace hagen_planner
{
    
class Waypoint
{
    private:
       
       
    public:
        double x_ = 0;
        double y_ = 0;
        double z_ = 0;
        double psi_ = 0;        
        double kappa_ = 0;
        double v_ref = 0;
        double lb, ub, static_border_cells, dynamics_border_cells;
        Waypoint(double x, double y, double psi, double kappa);
        Waypoint(double x, double y, double z, double psi, double kappa);
        Waypoint() = default;
        ~Waypoint() = default;

        double sub(Waypoint other);
        double sub3d(Waypoint other);
};

class ReferencePath{

    private:
         OsqpEigen::Solver* osqpInterface; 
    public:
        ReferencePath();
        ~ReferencePath()= default;
        std::vector<Waypoint> waypoints;
        double length = 0;
        std::vector<double> segment_lengths;
        Waypoint get_waypoint(int index);
        
        void construct_waypoints(Eigen::MatrixXd points, bool use_3d=false);
        void update_path_constraints(int index, int N, double min_width
                        , double safety_margin, Eigen::MatrixXd& ub_, Eigen::MatrixXd& lb_, bool use_3d=false);
        void calculate_velocity_profile(double a_max_, double a_min_, double v_max_, double v_min_, double ay_max_);
        void refine_trajectory(double smoothing_dis, double resolution , Eigen::MatrixXd& waypoints
                            , Eigen::MatrixXd& smooth_wps, bool use_3d=false);
        void compute_length(std::vector<double>& segment_lengths, double& distance);

        template<typename T> std::vector<double> linspace(T start_in, T end_in, int num_in){
            std::vector<double> linspaced;
            double start = static_cast<double>(start_in);
            double end = static_cast<double>(end_in);
            double num = static_cast<double>(num_in);
            if (num == 0) { return linspaced; }
            if (num == 1) {
                linspaced.push_back(start);
                return linspaced;
            }
            double delta = (end - start) / (num - 1);
            for(int i=0; i < num-1; ++i)
                {
                linspaced.push_back(start + delta * i);
                }
            linspaced.push_back(end); 
            return linspaced;
        }
};

}  // namespace hagen_planner

#endif