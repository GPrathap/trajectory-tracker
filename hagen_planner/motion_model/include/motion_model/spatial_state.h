#ifndef SPATIAL_STATE
#define SPATIAL_STATE

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>

namespace hagen_planner
{

class SpatialState
{
    private:
        
    public:
        double e_y = 0;
        double e_psi = 0;
        double t = 0;
        Eigen::Vector3d members;

        SpatialState() = default;
        ~SpatialState() = default;

        Eigen::Vector3d get_item();
        void set_y(double y_);
        void set_psi(double psi_);
        void set_t(double t_);
        int len();
        SpatialState* iadd(Eigen::Vector3d other);
};

class SimpleSpartialState : public SpatialState {
    public:
        SimpleSpartialState(double e_y_=0.0, double e_psi_=0.0, double t_=0.0);
        ~SimpleSpartialState() = default;
        SimpleSpartialState() = default;
};

}  // namespace hagen_planner

#endif