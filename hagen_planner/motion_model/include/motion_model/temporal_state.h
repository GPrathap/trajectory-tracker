#ifndef TMEPORALSTATE
#define TMEPORALSTATE

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>

namespace hagen_planner
{

class TemporalState
{
    private:
        
    public:

        double x_ = 0;
        double y_ = 0;
        double z_ = 0;
        double psi_ = 0;        
        Eigen::Vector3d members;
        TemporalState(double x, double y, double psi);
        TemporalState(double x, double y, double z, double psi);
        TemporalState() = default;
        ~TemporalState() = default;

        TemporalState* iadd(Eigen::Vector3d other);
        TemporalState* iadd(Eigen::VectorXd other);
};
}  // namespace hagen_planner

#endif