#include "mpc_opt/linear_mpc_trajectory_tracker.h"
#include <ros/ros.h>

namespace hagen_planner
{
  LinearMPCTrajectoryTracker::LinearMPCTrajectoryTracker()
  {
  }

  void LinearMPCTrajectoryTracker::setTrajectoryGenerator(const BSplineUtils::Ptr& manager){
    bspline_utils_ = manager;
  }

  void LinearMPCTrajectoryTracker::setIntermediateTrajectoryGenerator(const BSplineUtils::Ptr& manager){
    bspline_intermediate_ = manager;
  }

  void LinearMPCTrajectoryTracker::setBoundaryChecker(const hagen_planner::BsplineOptimizer::Ptr& opt){
    bound_checker = opt;
  }

  void LinearMPCTrajectoryTracker::solver_init(){

      initStatus = true;
      numState = model.numState;
      numInput = model.numInput;
      A.resize(0,0);
      xl.resize((prediction_horizon+1)*numState, 1);
      xu.resize((prediction_horizon+1)*numState, 1);
      ul.resize((prediction_horizon)*numInput, 1);
      uu.resize((prediction_horizon)*numInput, 1);
      P.resize(prediction_horizon*(numState+numInput)+numState, prediction_horizon*(numState+numInput)+numState);
      P.setZero();
      for(int i=0; i<(prediction_horizon)*numState; i+=numState){
        for(int j=0; j<numState; j++){
          for(int k=0; k<numState; k++){
            P.coeffRef(i+j, i+k) = model.Q(j, k);
          }
        }
      }
      for(int i=(prediction_horizon)*numState; i<(prediction_horizon+1)*numState; i+=numState){
        for(int j=0; j<numState; j++){
          for(int k=0; k<numState; k++){
            P.coeffRef(i+j, i+k) =  model.QN(j, k);
          }
        }
      }
      int index = 0;
      for(int i=(prediction_horizon+1)*numState; i<(prediction_horizon+1)*numState+numInput*prediction_horizon; i+=numInput){
        for(int j=0; j<numInput; j++){
          for(int k=0; k<numInput; k++){
            P.coeffRef(i+j, i+k) = model.R(j, k);
          }
        }
        index += numInput;
      }
      
      display_msgs.horizon = prediction_horizon;
      display_msgs.corridor =  std::make_shared<Corridor>();

      Eigen::MatrixXd k_A(numState, numState); // System dynamics matrix
      Eigen::MatrixXd k_C(numInput, numState); // Output matrix
      Eigen::MatrixXd k_Q(numState, numState); // Process noise covariance
      Eigen::MatrixXd k_R(numInput, numInput); // Measurement noise covariance
      Eigen::MatrixXd k_P(numState, numState); // Estimate error covariance
      k_A << 1,0,0,0,1,0,0,0,1;
      k_C << 1,0,0,0,1,0,0,0,1;
      
      bool passed = passer->passing_matrix("covariance_matrix_for_control_input_q", k_Q);
      if(!passed){
        double q_cov = 1;
        k_Q << q_cov, 0, 0, 0, q_cov, .0,0.0, 0, q_cov;
      }
      passed = passer->passing_matrix("covariance_matrix_for_control_input_r", k_R);
      if(!passed){
        double r_cov = 5000;
        k_R << r_cov, 0, 0, 0, r_cov, .0,0.0, 0, r_cov;
      }
      k_P << 1,0,0,0,1,0,0,0,1;
      std::cout << "A: \n" << k_A << std::endl;
      std::cout << "C: \n" << k_C << std::endl;
      std::cout << "Q: \n" << k_Q << std::endl;
      std::cout << "R: \n" << k_R << std::endl;
      std::cout << "P: \n" << k_P << std::endl;
      // Construct the filter
      kf = new KalmanFilter(0, k_A, k_C, k_Q, k_R, k_P);

      Eigen::MatrixXd nmpc_A(numState, numState); // System dynamics matrix
      Eigen::MatrixXd nmpc_C(numInput, numState); // Output matrix
      Eigen::MatrixXd nmpc_Q(numState, numState); // Process noise covariance
      Eigen::MatrixXd nmpc_R(numInput, numInput); // Measurement noise covariance
      Eigen::MatrixXd nmpc_P(numState, numState); // Estimate error covariance
      nmpc_A << 1,0,0,0,1,0,0,0,1;
      nmpc_C << 1,0,0,0,1,0,0,0,1;
      
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_q", nmpc_Q);
      if(!passed){
        double q_cov = 1;
        nmpc_Q << q_cov, 0, 0, 0, q_cov, .0,0.0, 0, q_cov;
      }
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_r", nmpc_R);
      if(!passed){
        double r_cov = 1;
        nmpc_R << r_cov, 0, 0, 0, r_cov, .0,0.0, 0, r_cov;
      }
      nmpc_P << 1,0,0,0,1,0,0,0,1;
      std::cout << "A: \n" << nmpc_A << std::endl;
      std::cout << "C: \n" << nmpc_C << std::endl;
      std::cout << "Q: \n" << nmpc_Q << std::endl;
      std::cout << "R: \n" << nmpc_R << std::endl;
      std::cout << "P: \n" << nmpc_P << std::endl;
      // Construct the filter
      kf_nmpc = new KalmanFilter(delta_t, nmpc_A, nmpc_C, nmpc_Q, nmpc_R, nmpc_P);

      vehicle_current_state = std::make_shared<std::vector<double>>(); 
      edt_env_->initConvexSegment();
  }

  void LinearMPCTrajectoryTracker::mpc_solver(){
    mpc_solver_with_multiple_shooting();
    return;
  }

  void LinearMPCTrajectoryTracker::calPoly(vec_E<Polyhedron<3>> polyhedrons,  std::vector<Eigen::MatrixXd>& filtered_){
    Eigen::Vector3d point = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d normal = Eigen::MatrixXd::Zero(3,1);
    int numPolys = polyhedrons.size();
    std::vector<Eigen::MatrixXd> selected_poly;
    std::vector<double> point_normal;
    for(int i=0; i < numPolys; ++i){
        auto hperplanes = polyhedrons[i].vs_;
        int j=0;
        Eigen::MatrixXd set_a_b(hperplanes.size(), 4);
        for(auto po : hperplanes){
            point << po.p_[0], po.p_[1], po.p_[2];
            normal << po.n_[0], po.n_[1], po.n_[2];
            std::cout << normal << std::endl;
            std::cout<<  normal.transpose() * point << std::endl;
            double d = normal.transpose() * point;
            set_a_b.block(j,0, 1, numState) = normal.transpose();
            set_a_b(j,3) = 1.0*d;
            j++;
        }
        selected_poly.push_back(set_a_b);
    }
    filtered_ = selected_poly;
    return;
  }

  bool LinearMPCTrajectoryTracker::calculateConstrains(Eigen::MatrixXd& horizon_control_points, vec_E<Polyhedron<3>>& free_space_
                  , Eigen::MatrixXd& final_cons_set, Eigen::MatrixXd& final_cons_b, 
                                Eigen::MatrixXd& final_cons_b_low){

        int numPolys = free_space_.size();
        if(numPolys<1){
          final_cons_b.resize(0,0);
          final_cons_b_low.resize(0,0);
          std::cout<< "Polyhedrons count less than zero..."<< std::endl;
          return false;
        }

        for(int i=0; i < numPolys; ++i){
          auto hperplanes = free_space_[i].vs_;
          int j=0;
          for(auto po : hperplanes){
              if(isnan(po.p_[0]) || isnan(po.p_[1]) || isnan(po.p_[2]) || isnan(po.n_[0]) || isnan(po.n_[1]) || isnan(po.n_[2])){
                std::cout<< "not good , " << po.p_ << "  "<< po.n_ << std::endl;
                return false;
              }
          }
        }

        int prediction_size = horizon_control_points.cols()-1;
        if(prediction_size> prediction_horizon){
          prediction_size = prediction_horizon;
        }
        double maxArea = 0;
        int total_counts = 0;
        std::vector<Eigen::MatrixXd> constraints_init;
        std::vector<Eigen::MatrixXd> constraints_b;
        std::vector<Eigen::MatrixXd> pre_set;
        calPoly(free_space_, pre_set);
        int total_cols =(prediction_horizon+1)*numState + prediction_horizon*numInput;
        Eigen::MatrixXd constrait_set = Eigen::MatrixXd::Zero(total_counts, total_cols);
        Eigen::MatrixXd constrait_b = Eigen::MatrixXd::Zero(total_counts, 1);
        int previous_index_rows = 0;
        int previous_index_cols = 0;
        for(int j=0; j<constraints_init.size(); j++){
            Eigen::MatrixXd next_set = constraints_init[j];
            if(next_set.rows()>0){
              constrait_set.block(previous_index_rows, previous_index_cols, next_set.rows(), numState) = next_set;
              constrait_b.block(previous_index_rows, 0, next_set.rows(), 1) =  constraints_b[j];
            }
            previous_index_rows += next_set.rows();
            previous_index_cols += numState;
        }

        visualization_->refer_pub.publish(display_msgs.refTraj_msg);
        visualization_->global_pub.publish(display_msgs.theta_msg);
        visualization_->drone_pub.publish(display_msgs.drone_msg);
        visualization_->predict_pub.publish(display_msgs.trajPred_msg);

        decom_ros_msgs::PolyhedronArray poly_msg1 = DecompROS::polyhedron_array_to_ros(_filtered_visu);
        poly_msg1.header.frame_id = "map";
        visualization_->flight_tunnel_pub.publish(poly_msg1);

        decom_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(free_space_);
        poly_msg.header.frame_id = "map";
        visualization_->vis_polytope_pub.publish(poly_msg);

        if(pre_set.size()>0){
            auto selected = pre_set[0];
          final_cons_set = selected.block(0, 0, selected.rows(), 3);
          final_cons_b = selected.block(0,3, selected.rows(), 1);
          return true;
        }else{
          return false;
        }
    }

  
    void LinearMPCTrajectoryTracker::printPolyhedron(Polyhedron<3>& poly_tunnel){
      auto hperplanes = poly_tunnel.vs_;
      Eigen::Vector3d point;
      Eigen::Vector3d normal;
      for(auto po : hperplanes){
              point << po.p_[0], po.p_[1], po.p_[2];
              normal << po.n_[0], po.n_[1], po.n_[2];
              std::cout<< "point: " << point.transpose() << " normal: "<< normal.transpose() << std::endl; 
      }
    }

    void LinearMPCTrajectoryTracker::createTunnel(const Eigen::Vector3d& position
        , const Eigen::Vector3d& tangent_line_, Polyhedron<3>& poly_tunnel, int index, Eigen::MatrixXd& constraint_set){
        if(index ==0){
            _filtered_visu.clear();
        }
        for ( int j = 0; j < constraint_set.rows(); j++ ){
            Eigen::Vector3d normal = constraint_set.block(j, 0, 1, 3).transpose().normalized();
            double D = constraint_set(j,3) / constraint_set.block(j, 0, 1, 3).transpose().norm();
            Eigen::Vector3d point_on_plane = - normal * (-D);
            Eigen::Matrix<double, 3, 1> p_(point_on_plane(0), point_on_plane(1), point_on_plane(2));
            Eigen::Matrix<double, 3, 1> n_(normal(0), normal(1), normal(2));
            Hyperplane3D hyper_planeyu(p_, n_);
            poly_tunnel.add(hyper_planeyu);
        }

        Eigen::Vector3d extra_point, extra_normal = tangent_line_.normalized();
        extra_point = position + extra_normal * 1e-1;
        const Eigen::Matrix<double, 3, 1> p_(extra_point(0), extra_point(1), extra_point(2));
        const Eigen::Matrix<double, 3, 1> n_(extra_normal(0), extra_normal(1), extra_normal(2));
        Hyperplane3D hyper_planer(p_, n_);
        poly_tunnel.add(hyper_planer);
        extra_point = position - extra_normal * 1e-1;
        const Eigen::Matrix<double, 3, 1> p1_(extra_point(0), extra_point(1), extra_point(2));
        const Eigen::Matrix<double, 3, 1> n1_(-extra_normal(0), -extra_normal(1), -extra_normal(2));
        Hyperplane3D hyper_plane1(p1_, n1_);
        poly_tunnel.add(hyper_plane1);
        _filtered_visu.push_back(poly_tunnel);
        return;
    } 

    Eigen::VectorXd LinearMPCTrajectoryTracker::get_eiegn_vec(Eigen::SparseMatrix<double> sparse_mat){
      Eigen::VectorXd vec(sparse_mat.rows());
      for(int i=0;i< sparse_mat.rows(); i++){
        vec[i] = sparse_mat.coeffRef(i, 0);
      }
      return vec;
    }
  
  void LinearMPCTrajectoryTracker::global_solver(){
    clock_t t2, t1 = clock();
    std::cout<< "still_running_global_planner: "<< still_running_global_planner<< std::endl;
    while(still_running_global_planner){
      t2 = clock();
      if(!still_running){
        std::cout<< "Stop global planning" << std::endl;
        break;
      }

      if((t2-t1) > delta_t){
          lock_on_local_to_global.lock();
          int instance_index = start_index;
          int instance_end_index = end_index;
          int max_index = horizon_control_points_total.cols()-1;
          if(set_running_global_planner && (instance_end_index-instance_index > prediction_horizon) 
                    && (instance_end_index > instance_index) && max_index>0){
            instance_end_index = (max_index >= instance_end_index)? instance_end_index : max_index;
            if(!set_new_path){
                    int next_free_index = instance_index;
                    int step_size = 5;
                    while(true){
                      next_free_index = instance_index + (int)(current_time + (step_size+reference_trj_horizon)*(delta_t_desired))/0.1;
                      if(next_free_index>=max_index){
                        next_free_index = max_index;
                        break;
                      }
                      if(edt_env_->get_free_distance(horizon_control_points_total.col(next_free_index))>avoidance_distance){
                        break;
                      }
                      step_size += 5;
                    }
                    int start_in = 4;
                    Eigen::MatrixXd ref_horizon_points(3, next_free_index-(instance_index-start_in)); 
                    ref_horizon_points.block(0, start_in,3, next_free_index-instance_index) = horizon_control_points_total.block(0, instance_index, 3, next_free_index-instance_index);
                    ref_horizon_points.col(0) = current_odom_pose;
                    ref_horizon_points.col(1) = current_odom_pose;
                    ref_horizon_points.col(2) = current_odom_pose;
                    ref_horizon_points.col(3) = current_odom_pose;
                    bound_checker->initControlPoints(ref_horizon_points, true);
                    bool is_opt = bound_checker->BsplineOptimizeTrajRebound(ref_horizon_points, 0.1, current_odom_pose, current_pose_obs);
                    std::vector<Eigen::Vector3d> rebound_array;
                    for(int jkl=0; jkl<ref_horizon_points.cols(); jkl++){
                      auto row = ref_horizon_points.col(jkl);
                      Eigen::Vector3d row_vector(row[0], row[1], row[2]);
                      rebound_array.push_back(row_vector);
                    }
                    if(next_free_index <= horizon_control_points_total.cols()){
                      if(instance_index-start_in < 0){
                        horizon_control_points_total.block(0, instance_index, 3, next_free_index-instance_index) 
                                                  = ref_horizon_points.block(0,start_in,3,next_free_index-instance_index);
                      }else{
                        horizon_control_points_total.block(0, instance_index-start_in, 3, next_free_index-(instance_index-start_in)) 
                                                  = ref_horizon_points;
                      }
                      visualization_->drawPath(rebound_array, 0.2, Eigen::Vector4d(0.6, 0.2 ,0.8, 1), 230);
                      global_opt_sudden_stop = false;
                    }else{
                      ROS_WARN("Wont set the state someting wrong with global planning,...(:");
                      global_opt_sudden_stop = true;
                    }
            }else{
              std::cout<< "Path is not set..." << std::endl;
            }
        }
        lock_on_local_to_global.unlock();
        condition_lock_on_local_to_global.notify_one();
        t1 = clock();
      }else{
        usleep(5000);
      }
    }
  }

  void LinearMPCTrajectoryTracker::mpc_solver_with_multiple_shooting(){

    double t0 = 0;
    vector<double> t;
    t.push_back(t0);
    int sim_time = simulated_duration;
    int mpciter = 0;
    vector<vector<double>> xx1, u_cl, xx0, xx;
    still_running = true;
    Eigen::VectorXd k_x0(numState);
    k_x0 << 0, 0, 0;
    kf->init(0, k_x0);
    Eigen::VectorXd k_p1(numState);
    k_p1 = x0.col(0);
    kf_nmpc->init(0, k_p1);
    clock_t t2, t1 = clock();
    double t_cmd_start, t_cmd_end;
    bspline_utils_->traj_pos_.getTimeSpan(t_cmd_start, t_cmd_end);
    bspline_utils_->retrieveTrajectory();
    time_traj_start = ros::Time::now();
    double error((x0.col(0)-xs.head(3)).norm());
    traj_cmd = new boost::circular_buffer<Eigen::Vector3d>(10000);
    double tm, tmp;
    auto trajectory = bspline_utils_->traj_pos_;
    trajectory.getTimeSpan(tm, tmp);
    
    std::vector<Eigen::Vector3d> horio_;
    for (double t = tm; t <= tmp; t += delta_t_desired)
    {
      Eigen::Vector3d pt = trajectory.evaluateDeBoor(t);
      horio_.push_back(pt);
    }
    int k=0;
    horizon_control_points_total.resize(3, horio_.size()+3);
    for (auto pose : horio_)
    {
      horizon_control_points_total.col(k) << pose;
      k++;
    }
    Eigen::Vector3d last_pose = horizon_control_points_total.col(k-1);
    horizon_control_points_total.col(k) << last_pose;
    k++;
    horizon_control_points_total.col(k) << last_pose;
    k++;
    horizon_control_points_total.col(k) << last_pose;

    int solveStatus = 1;
  
   
    u_min.resize(numInput, 1);
    u_min << -0.4, -0.4, -0.4;
    u_max.resize(numInput, 1);
    u_max << 0.4, 0.4, 0.4;
    x_min.resize(numState, 1);
    x_min << -OSQP_INFTY, -OSQP_INFTY, -OSQP_INFTY;
    x_max.resize(numState, 1);
    x_max << OSQP_INFTY, OSQP_INFTY, OSQP_INFTY;

    Eigen::VectorXd gradient;
    set_running_global_planner =  true;
    Eigen::MatrixXd current_control(numInput, prediction_horizon);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1), numState*(prediction_horizon+1));
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1), numInput*prediction_horizon);
    Eigen::MatrixXd uq = Eigen::MatrixXd::Zero(numState*(prediction_horizon), 1);

    for(int n=0; n<prediction_horizon; n++){
        A.block(numState*(n+1), numState*(n), numState, numState) = model.Ad;
        B.block(numState*(n+1), numInput*(n), numState, numInput) = model.Bd;
    }

    while(error > maximum_acceptable_error){
        Eigen::Vector3d end_pose = horizon_control_points_total.col(horizon_control_points_total.cols()-1);
        Eigen::Vector3d start_pose(x0.col(0));
        error = (end_pose-start_pose).norm();
        t2 = clock();
        int k_index = numInput + numState;
        gradient = Eigen::VectorXd::Zero((prediction_horizon+1)*numState + prediction_horizon*numInput);
        if(double(t2 - t1)/CLOCKS_PER_SEC > delta_t){
          ros::Time time_now = ros::Time::now();
          std::vector<Eigen::Vector3d> vis_horizon_desired;
          std::vector<Eigen::Vector3d> vis_horizon_desired_velocities;
          current_time = (time_now - time_traj_start).toSec();
          start_index = (int)(current_time/delta_t_desired);
          start_index = (start_index < 0) ? 0 : start_index;
          start_index = (start_index >= horizon_control_points_total.cols()) ? horizon_control_points_total.cols()-1 : start_index;
          end_index = (int)(current_time + reference_trj_horizon*(delta_t_desired))/0.1;
          end_index = (end_index >= horizon_control_points_total.cols()) ? horizon_control_points_total.cols()-1 : end_index;
          
          current_odom_pose << real_odom.pose.pose.position.x,
          real_odom.pose.pose.position.y, real_odom.pose.pose.position.z;

          x0 << real_odom.pose.pose.position.x,
          real_odom.pose.pose.position.y, real_odom.pose.pose.position.z;
          if(horizon_control_points_total.cols()<=start_index){
            start_index = horizon_control_points_total.cols()-1;
          }
          set_new_path = false;
          double dis = (current_odom_pose - horizon_control_points_total.col(start_index)).norm();
         
          if(dis>fluctuation_length){
            std::cout<< "different: "<< fluctuation_length << std::endl;
            bool refine_trajctory_ = true;
            std::unique_lock<std::mutex> lk(lock_on_local_to_global);
            condition_lock_on_local_to_global.wait(lk, [&]{return refine_trajctory_;});
            if((horizon_control_points_total.cols()-start_index<2)){
              ROS_WARN("Solver wont able to solve the trajectory..., (;");
            }
            int num_cols = horizon_control_points_total.cols()-start_index;
            Eigen::MatrixXd ref_horizon_points = horizon_control_points_total.block(0, start_index, 3, num_cols);
            int size_of_pro = ref_horizon_points.cols();
            int iinter_points = (int)(size_of_pro/max_fluctuatio_allowed);
            
            if(size_of_pro>10 && iinter_points>3){
              Eigen::MatrixXd projected_(3, iinter_points);
              projected_.block(0,0, 3, 1) =  current_odom_pose;
              int index_o = 0;
              for(int k = 1; k<iinter_points; k++){
                if(index_o< size_of_pro){
                  projected_.block(0,k, 3, 1) =  ref_horizon_points.col(index_o);
                }else{
                  projected_.block(0,k, 3, 1) =  ref_horizon_points.col(size_of_pro-1);
                }
                index_o += max_fluctuatio_allowed;
              }
              projected_.block(0, iinter_points-1, 3, 1) =  ref_horizon_points.col(size_of_pro-1);
              bool having_trj = bspline_intermediate_->generateTrajectory(projected_, 1);
              if(having_trj){
                Eigen::MatrixXd refined_trj;
                bool is_refined = bspline_intermediate_->getIntermediatePoints(delta_t_desired, size_of_pro, refined_trj);
                if(is_refined){
                  horizon_control_points_total = refined_trj;
                  time_traj_start = ros::Time::now();
                  start_index = 0;
                  end_index = (end_index >= horizon_control_points_total.cols()) ? horizon_control_points_total.cols()-1 : end_index;
                  set_new_path = true;
                }
              }
            }
            lk.unlock();
            condition_lock_on_local_to_global.notify_one();
          }
          
          Eigen::Vector3d previos_free_pose = current_odom_pose;
          int index_x = 0;
          int index_u = prediction_horizon*numState + numState;
          Eigen::MatrixXd check_refine_points(numState, prediction_horizon);
          int set_index_1 = (int)prediction_horizon/4;
          int set_index_2 = (int)prediction_horizon/2;

          if( bound_checker->current_pose_obs == 2){
              Eigen::Vector3d current_pose = previos_free_pose;
              int start_index_ = start_index;
              for(int k=0; k<prediction_horizon; k++){
                vis_horizon_desired.push_back(current_pose);
                Eigen::VectorXd ref_pose(3);
                ref_pose << current_pose[0], current_pose[1], current_pose[2];
                check_refine_points.col(k) = ref_pose;
              }
          }else{
              for(int k=0; k<prediction_horizon; k++){
                Eigen::Vector3d current_pose;
                if(global_opt_sudden_stop){
                  current_pose = previos_free_pose;
                }else if(start_index+k<end_index){
                    current_pose = horizon_control_points_total.col(start_index+k);
                }
                if((current_odom_pose-current_pose).norm()>2.0){
                  current_pose = previos_free_pose;
                }
                vis_horizon_desired.push_back(current_pose);
                Eigen::VectorXd ref_pose(3);
                ref_pose << current_pose[0], current_pose[1], current_pose[2];
                check_refine_points.col(k) = ref_pose;
              }
          }

          Eigen::MatrixXd a1;
          Eigen::MatrixXd b1;
          Eigen::MatrixXd constraint_set_b_low;
          Eigen::MatrixXd horizon_control_points_ = check_refine_points;
          std::vector<Eigen::Vector3d> projected_trajectory_;   
          projected_trajectory_.push_back(current_odom_pose);
          projected_trajectory_.push_back(horizon_control_points_.col(prediction_horizon-1).head(3));
          vec_E<Polyhedron<3>> free_space_;
          edt_env_->find_free_space_odom(projected_trajectory_, free_space_);
          bool is_corridor_set = calculateConstrains(horizon_control_points_, free_space_, a1, b1, constraint_set_b_low);
          visualization_->drawPath(vis_horizon_desired, 0.2, Eigen::Vector4d(1, 0 ,0, 1), 234);
          visualization_->drawPath(vis_horizon_desired_velocities, 0.2, Eigen::Vector4d(1, 0 ,0, 1), 236);
          Eigen::SparseMatrix<c_float> AA;
          castMPCToQPConstraintMatrix(A, B, AA, a1);
          Eigen::MatrixXd xr = check_refine_points.col(check_refine_points.cols()-1).head(3);
          Eigen::MatrixXd q((prediction_horizon+1)*numState + numInput*prediction_horizon, 1);
          Eigen::MatrixXd gradient_q = (-1.0*model.Q*xr).replicate(prediction_horizon, 1);
          Eigen::MatrixXd gradient_q_n = (-1.0*model.QN*xr);
          Eigen::MatrixXd gradient_u = Eigen::MatrixXd::Zero(prediction_horizon*numInput, 1);
          q << gradient_q, gradient_q_n, gradient_u;
          Eigen::VectorXd gradient1(q.rows());
          for(int i=0; i< q.rows(); i++){
              gradient1[i] = q(i, 0);
          }
          Eigen::VectorXd lowerBound;
          Eigen::VectorXd upperBound;
          castMPCToQPConstraintVectors(x_max, x_min, u_max, u_min, x0, uq, lowerBound, upperBound, b1);
          OsqpEigen::Solver qp_solver;
          qp_solver.settings()->setVerbosity(false);
          qp_solver.settings()->setWarmStart(true); 
          qp_solver.data()->setNumberOfVariables(P.rows());
          qp_solver.data()->setNumberOfConstraints(AA.rows());
          qp_solver.data()->setHessianMatrix(P);
          qp_solver.data()->setGradient(gradient1);
          qp_solver.data()->setLinearConstraintsMatrix(AA);
          qp_solver.data()->setLowerBound(lowerBound);
          qp_solver.data()->setUpperBound(upperBound);
          qp_solver.initSolver();
          set_init_solver = true;
          save_model(AA, "1A_");
          save_model(gradient, "1q_");
          save_model(lowerBound, "1l_");
          save_model(upperBound, "1u_");
          save_model(P, "1P_");
          bool set_correct = true;
          if(!qp_solver.solve()) set_correct = false;
            Eigen::VectorXd QPSolution;
            if(set_correct){
              std::cout << "Control policy is calculated linear ..." << std::endl;
              int h = 0;
              QPSolution = qp_solver.getSolution();
              Eigen::MatrixXd predicted_state(numState, prediction_horizon+1);
              int input_index = 0;
              predicted_state.col(0) = x0;
              for(int j=numState*(prediction_horizon+1); j<(numState*(prediction_horizon+1))+numInput*prediction_horizon; j+=numInput){
                current_control.col(input_index) << QPSolution[j], QPSolution[j+1], QPSolution[j+2];
                predicted_state.col(input_index+1) = model.Ad*predicted_state.col(input_index) + delta_t*model.Bd*current_control.col(input_index);
                input_index++;
              }
              x0 = model.Ad*x0 + delta_t*model.Bd*current_control.col(0);
              std::vector<Eigen::Vector3d> vis_horizon;
              for(int kl=0; kl< predicted_state.cols(); kl++){
                Eigen::Vector3d poss = predicted_state.col(kl).head(3);
                vis_horizon.push_back(poss);
              }
              visualization_->drawPath(vis_horizon, 0.2, Eigen::Vector4d(0.2, 1 ,0.5, 1), 0);
              infeasibility_counter = 0;
              current_u.resize(3);
              current_u<< current_control(0,0), current_control(1,0), current_control(2,0);
              set_solver_ = 1;
            }else{
              std::cout << "Error while solving... using previous control...." << std::endl;
              if(set_solver_ == 0){
                std::cout<< "Problem can not be solved...." << std::endl;
                force_terminate =  true;
                still_running = false;
                return; 
              }
              int id = infeasibility_counter;
              auto current_u_ = current_control.col(id);
              current_u = current_u_;
              infeasibility_counter += 1;
              x0 = model.Ad*x0 + delta_t*model.Bd*current_control.col(id);
          }
          lowerBound.block(0,0,numState,1) = -x0;
          upperBound.block(0,0,numState,1) = -x0;

          if(infeasibility_counter == prediction_horizon-1){
            std::cout<< "Problem can not be solved...." << std::endl;
            force_terminate =  true;
          }
        
          t.push_back(t0);
        
          if(force_terminate){
            still_running = false;
            std::cout<< "Solver has been interrupted..." <<  std::endl;
            return;
          }
          Eigen::VectorXd k_y(numInput);
          std::vector<double> k_estimated_values;
          k_y = delta_t*current_u;
          kf->update(k_y);
          Eigen::MatrixXd B_ = Eigen::MatrixXd::Identity(numState, numState);
          Eigen::VectorXd k_p_(numState);
          kf_nmpc->update(x0, B_, k_y);
          current_projected_pose.pose.pose.position.x =  kf_nmpc->state()[0];
          current_projected_pose.pose.pose.position.y = kf_nmpc->state()[1];
          current_projected_pose.pose.pose.position.z = kf_nmpc->state()[2];

          std::cout<< "============x0 " << x0.transpose()  << std::endl;
          std::cout<< "============current_odom_pose: " << current_odom_pose.transpose() << std::endl;
          std::cout<< "============ kf_nmpc->state(): " <<  kf_nmpc->state().transpose() << std::endl;
          current_projected_pose.twist.twist.linear.x = current_u(0);
          current_projected_pose.twist.twist.linear.y = current_u(1);
          current_projected_pose.twist.twist.linear.z = current_u(2);
          current_projected_pose.twist.twist.angular.z = 0.0;
          pos_current_pos_pub.publish(current_projected_pose);
          mpciter = mpciter + 1;
          t1 = clock();
        }else{
          usleep(5000);
        }
    }

    still_running = false;
    need_intermediate_goal = false;
    cout<< "Finish mpc solver...."<< endl;
  }

  double LinearMPCTrajectoryTracker::get_distance(Eigen::MatrixXd norminal, Eigen::MatrixXd actual){
    double dis = 0;
    for(int i=0; i< norminal.cols(); i++){
      dis+=(norminal.col(i)-actual.col(i)).norm();
    }
    return dis;
  }

  void LinearMPCTrajectoryTracker::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    reference_odom = *msg;
    is_reference_trj_set = true;
  }

  void LinearMPCTrajectoryTracker::odomCallbackReal(const nav_msgs::OdometryConstPtr& msg){
    real_odom = *msg;
    is_real_odom_set = true;
  }


  void LinearMPCTrajectoryTracker::init(ros::NodeHandle& nh){
      node_ = nh;
      LinearizedMPCOpt::init(nh);
      node_.param("mpc_opt/delta_t_desired", delta_t_desired, 0.02);
      node_.param("mpc_opt/reference_trj_horizon", reference_trj_horizon, 40);
      odometry_sub_ref_ = node_.subscribe<nav_msgs::Odometry>("/odom", 50, &LinearMPCTrajectoryTracker::odomCallback, this);
      odometry_sub_real_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 50, &LinearMPCTrajectoryTracker::odomCallbackReal, this);
      
      bspline_intermediate_.reset(new BSplineUtils);
      bspline_intermediate_->setParam(nh);
      bspline_intermediate_->visualization_.reset(new PlanningVisualization(nh));
      
      solver_init();
  }

}  // namespace hagen_planner
