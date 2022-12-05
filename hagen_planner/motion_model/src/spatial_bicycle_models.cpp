#include <motion_model/spatial_bicycle_models.h>


namespace hagen_planner
{

    SpatialBicycleModel::SpatialBicycleModel(ReferencePath reference_path_, double height_, double width_, double Ts_){
        reference_path = reference_path_;
        Ts = Ts_; 
        current_waypoint = reference_path.waypoints[wp_id];
        height = height_;
        width = width_;
    }

    TemporalState SpatialBicycleModel::s2t(Waypoint reference_waypt, SpatialState reference_state){
        double x = reference_waypt.x_ - reference_state.e_y*std::sin(reference_waypt.psi_);
        double y = reference_waypt.y_ + reference_state.e_y*std::cos(reference_waypt.psi_);
        double psi = reference_waypt.psi_ + reference_state.e_psi;
        TemporalState temstate(x, y, psi);
        return temstate; 
    }
    
    SimpleSpartialState SpatialBicycleModel::t2s(Waypoint reference_waypt, TemporalState reference_state){
        double e_y = std::cos(reference_waypt.psi_)*(reference_state.y_ - reference_waypt.y_) 
                    - std::sin(reference_waypt.psi_)*(reference_state.x_ - reference_waypt.x_);
        double e_psi = reference_state.psi_ - reference_waypt.psi_;
        e_psi = std::fmod((e_psi + M_PI),(2*M_PI)) - M_PI; 
        SimpleSpartialState simpleSpartialState(e_y, e_psi, 0.0);
        return simpleSpartialState;
    }

    Eigen::Vector2d SpatialBicycleModel::drive(Eigen::Vector2d u){
        double v, delta;
        v = u[0];
        delta = u[1];
        // std::cout<< "==============v delta Ts================="<< v << "," << delta << "," << Ts<< std::endl;
        // std::cout<< "============== temporal_state.psi_ ================="<< temporal_state.psi_  << "," << temporal_state.x_ << "," << temporal_state.y_<< std::endl;
        double x_dot = v * std::cos(temporal_state.psi_);
        double y_dot = v * std::sin(temporal_state.psi_);
        // std::cout<< "==============x dot========="<< x_dot << "  " << y_dot << std::endl;
        double psi_dot = (v/height)*std::tan(delta);
        //  std::cout<< "==============x dot========="<< psi_dot << std::endl;
        Eigen::Vector3d temporal_derivatives(x_dot, y_dot, psi_dot);
        // temporal_derivatives = temporal_derivatives;
        // std::cout<< "=============temporal_derivatives========="<< temporal_derivatives << std::endl;
        temporal_state.psi_ += temporal_derivatives[2]*Ts;
        temporal_state.x_ += temporal_derivatives[0]*Ts;
        temporal_state.y_ += temporal_derivatives[1]*Ts;
        double s_dot = (1/(1-spatial_state.e_y*current_waypoint.kappa_))*v*std::cos(spatial_state.e_psi);
        // std::cout<< "======== s =====s_dot=========" << s << "," << s_dot << std::endl;
        s += s_dot*Ts; 
        Eigen::Vector2d next_state(s_dot, temporal_state.psi_);
        return next_state;
    }

    void SpatialBicycleModel::get_current_waypoint(int& index, Waypoint& current_wp){
        std::vector<double> length_cum(reference_path.segment_lengths.size());
        std::partial_sum(reference_path.segment_lengths.begin(), reference_path.segment_lengths.end(), length_cum.begin());
        int next_wp_id = length_cum.size()-1;
        for(int i=0; i<length_cum.size(); i++){
            if(length_cum[i]>s){
                next_wp_id = i;
                break;
            }
        }
        int prev_wp_id = next_wp_id -1;
        double s_next = length_cum[next_wp_id];
        double s_prev = length_cum[prev_wp_id];
        if(std::abs(s - s_next) < std::abs(s - s_prev)){
            wp_current_id = next_wp_id;
        }else{
            wp_current_id = prev_wp_id;
        }
        current_wp = current_waypoint = reference_path.waypoints[wp_current_id];
        index = wp_current_id;
    }

    double SpatialBicycleModel::compute_safety_margin(){
        return width/std::sqrt(2);
    }

    void SpatialBicycleModel::get_current_waypoint_index(int index, Waypoint& current_point){
        current_point = reference_path.get_waypoint(index);
    }

    bool SpatialBicycleModel::set_current_waypoint(int index){
        wp_id = index; 
        if(index>= reference_path.waypoints.size()){
            wp_id = reference_path.waypoints.size()-1;
        }
        current_waypoint = reference_path.waypoints[wp_id];
        previous_index = index;
    }

    BicycleModel::BicycleModel(ReferencePath reference_path, double height, double width, double Ts){
        SpatialBicycleModel spacial_bycycle_model(reference_path, height, width, Ts);
        temporal_state = s2t(current_waypoint, spatial_state);
        // Ad.resize(numState,numState);
        // Bd.resize(numState,numInput);
        // Q.resize(numState, numState);
        // R.resize(numInput, numInput);
        // std::vector<Eigen::Triplet<double>> triplets;
        // for (int i=0; i<numState; ++i){
        //    Ad.coeffRef(i,i) = 1.0;
        //    Bd.coeffRef(i,i) = 1.0;
        //    Q.coeffRef(i,i) = 1.0;
        //    R.coeffRef(i,i) = 1.0;
        // }
    }
    
    Eigen::Vector2d BicycleModel::get_temporal_derivatives(SpatialState state, Eigen::Vector2d input, double kappa){
        double v = input[0];
        double delta = input[1];
        double s_dot = 1/(1-(state.e_y*kappa))*v*std::cos(state.e_psi);
        double psi_dot = v/(height*std::tan(delta));
        Eigen::Vector2d temp_devt(s_dot, psi_dot);
        return temp_devt;
    }

    Eigen::Vector3d BicycleModel::get_spatial_derivatives(SpatialState state, Eigen::Vector2d input, double kappa){
        Eigen::Vector2d temp_dev = get_temporal_derivatives(state, input, kappa);
        double v = input[0];
        double delta = input[1];
        double s_dot = temp_dev[0];
        double psi_dot = temp_dev[1];
        double d_e_y_d_s = v*std::sin(state.e_psi)/s_dot;
        double d_e_psi_d_s = psi_dot/(s_dot-kappa);
        double d_t_d_s = 1/(s_dot);
        Eigen::Vector3d spatial_dev(d_e_y_d_s, d_e_psi_d_s, d_t_d_s);
        return spatial_dev;
    }

    void BicycleModel::linearized(double v_ref, double kappa_ref, double delta_s, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::Vector3d& f){

        Eigen::Vector3d a_1(1, delta_s, 0);
        Eigen::Vector3d a_2(-std::pow(kappa_ref,2)*delta_s, 1, 0);
        Eigen::Vector3d a_3;
        // if(v_ref*delta_s < 0.0001){
            // a_3 << 0, 0, 1;
        // }else{
            a_3 << -kappa_ref/v_ref*delta_s, 0, 1;
        // }

        Eigen::Vector2d b_1(0,0);
        Eigen::Vector2d b_2(0, delta_s);
        Eigen::Vector2d b_3;
        double con = std::pow(v_ref,2);
        // std::cout<< con << std::endl;
        // if(con<0.0000000001){
            // b_3 << 0, 0;
        // }else{
            b_3 << (-1/con)*delta_s, 0;
        // }
        
        Eigen::Vector3d f_;
        // if(v_ref*delta_s <0.01){
            // f_<< 0,0,0;
        // }else{
            f_ << 0 , 0, 1/v_ref*delta_s;
        // }
        
        A.row(0) = a_1;
        A.row(1) = a_2;
        A.row(2) = a_3;

        B.row(0) = b_1;
        B.row(1) = b_2;
        B.row(2) = b_3; 

        f = f_;
    }
}  // namespace hagen_planner
