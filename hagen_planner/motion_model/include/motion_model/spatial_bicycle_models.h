#ifndef SPATIAL_BICYCLE_MODELS_
#define SPATIAL_BICYCLE_MODELS_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <deque>
#include <math.h>
#include <queue>
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/move/move.hpp>
#include <spline_opt/reference_path.h>
#include <motion_model/spatial_state.h>
#include <motion_model/temporal_state.h>
#include <functional>   // std::multiplies
#include <numeric>      // std::partial_sum  

namespace hagen_planner
{
class SpatialBicycleModel
{
    private:


    public:
        Waypoint current_waypoint;
        ReferencePath reference_path;
        double s = 0.0;
        double eps = 1e-12;
        double Ts = 0.05;
        int wp_id = 0;
        double width = 2.0;
        double height = 0.5;
        int previous_index = 0;
        int wp_current_id = 0;

        SpatialState spatial_state;
        TemporalState temporal_state;

        SpatialBicycleModel(ReferencePath reference_path_, double height_, double width_, double Ts_);
        ~SpatialBicycleModel() = default;
        SpatialBicycleModel() = default;
        TemporalState s2t(Waypoint reference_waypt, SpatialState reference_state);
        SimpleSpartialState t2s(Waypoint reference_waypt, TemporalState reference_state);
        Eigen::Vector2d drive(Eigen::Vector2d u);
        double compute_safety_margin();
        bool set_current_waypoint(int index);
        void get_current_waypoint(int& index, Waypoint& current_wp);
        void get_current_waypoint_index(int index,  Waypoint& current_wp);
};

class BicycleModel : public SpatialBicycleModel{

    private:

    public:
        BicycleModel() = default;
        ~BicycleModel() = default;
        static constexpr unsigned long freq = 20; // frequency of discretization
        static constexpr double ts = 1.0/freq;
        static constexpr int numOrder = 3;        // number of orders
        static constexpr int numDimention = 3;    // number of dimentions
        static constexpr int numState = 3; // number of states
        static constexpr int numInput = 2; // number of inputs
        
        Eigen::SparseMatrix<double> Ad;
        Eigen::SparseMatrix<double> Bd;
        Eigen::SparseMatrix<double> Q;
        Eigen::SparseMatrix<double> R;
        Eigen::SparseMatrix<double> QN;

        double n_states = 3;
        double n_controls = 2;

        BicycleModel(ReferencePath reference_path, double height, double width, double Ts);

    Eigen::Vector2d get_temporal_derivatives(SpatialState state, Eigen::Vector2d input, double kappa);
    Eigen::Vector3d get_spatial_derivatives(SpatialState state, Eigen::Vector2d input, double kappa);
    void linearized(double v_ref, double kappa_ref, double delta_s, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::Vector3d& f);

};  // namespace hagen_planner
}
#endif