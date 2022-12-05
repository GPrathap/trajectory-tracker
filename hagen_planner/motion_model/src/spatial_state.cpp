#include <motion_model/spatial_state.h>

namespace hagen_planner
{

    Eigen::Vector3d SpatialState::get_item(){
        Eigen::Vector3d next_item(e_y, e_psi, t);
        return next_item; 
    }

    void SpatialState::set_y(double y_){
        e_y = y_;
    }

    void SpatialState::set_psi(double psi_){
        e_psi = psi_;
    }

    void SpatialState::set_t(double t_){
        t = t_;
    }

    int SpatialState::len(){
        return members.size();
    }

    SpatialState* SpatialState::iadd(Eigen::Vector3d other){
        e_y += other[0];
        e_psi += other[1];
        t += other[2];
        return this;
    }

    SimpleSpartialState::SimpleSpartialState(double e_y_, double e_psi_, double t_){
        SpatialState spatial_state;
        e_y = e_y_;
        e_psi = e_psi_;
        t = t_;
    }

}  // namespace hagen_planner
