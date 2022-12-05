#include <motion_model/temporal_state.h>
namespace hagen_planner
{

    TemporalState::TemporalState(double x, double y, double psi){
        x_ = x;
        y_ = y;
        psi_ = psi;
    }

    TemporalState::TemporalState(double x, double y, double z, double psi){
        x_ = x;
        y_ = y;
        z_ = z;
        psi_ = psi;
    }

    TemporalState* TemporalState::iadd(Eigen::VectorXd other) {
        x_ += other[0];
        y_ += other[1];
        z_ += other[2];
        psi_ += other[3];
        return this;
    }

    TemporalState* TemporalState::iadd(Eigen::Vector3d other) {
        x_ += other[0];
        y_ += other[1];
        psi_ += other[2];
        return this;
    }
}  // namespace hagen_planner

