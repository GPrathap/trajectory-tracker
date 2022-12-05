#include "mpc_opt/trajectory_tracker.h"
#include <ros/ros.h>

namespace hagen_planner
{
  TrajectoryTracker::TrajectoryTracker()
  {
    
  }

  void TrajectoryTracker::setTrajectoryGenerator(const BSplineUtils::Ptr& manager){
    bspline_utils_ = manager;
  }

  void TrajectoryTracker::setIntermediateTrajectoryGenerator(const BSplineUtils::Ptr& manager){
    bspline_intermediate_ = manager;
  }

  void TrajectoryTracker::setBoundaryChecker(const hagen_planner::BsplineOptimizer::Ptr& opt){
    bound_checker = opt;
  }

  void TrajectoryTracker::solver_init(){

      MX x = MX::sym("x");
      MX y = MX::sym("y");
      MX z = MX::sym("z");
      // MX theta = MX::sym("theta");
      states = vertcat(x, y, z);
      n_states = states.size1();

      MX v_x = MX::sym("v_x");
      MX v_y = MX::sym("v_y");
      MX v_z = MX::sym("v_z");
      // MX omega = MX::sym("omega");
      controls = vertcat(v_x, v_y, v_z);
      n_controls = controls.size1();

      Eigen::MatrixXd k_A(n_states, n_states); // System dynamics matrix
      Eigen::MatrixXd k_C(n_controls, n_states); // Output matrix
      Eigen::MatrixXd k_Q(n_states, n_states); // Process noise covariance
      Eigen::MatrixXd k_R(n_controls, n_controls); // Measurement noise covariance
      Eigen::MatrixXd k_P(n_states, n_states); // Estimate error covariance
      k_A << 1,0,0,0,1,0,0,0,1;
      k_C << 1,0,0,0,1,0,0,0,1;
      
      bool passed = passer->passing_matrix("covariance_matrix_for_control_input_q", k_Q);
      if(!passed){
        double q_cov = 1;
        k_Q << q_cov, 0, 0,  0, q_cov, .0, 0,0.0, q_cov;
      }
      passed = passer->passing_matrix("covariance_matrix_for_control_input_r", k_R);
      if(!passed){
        double r_cov = 5000;
        k_R << r_cov, 0, .0, 0, r_cov, .0,  .0, 0, r_cov;
      }
      k_P << 1,0,0,0,1,0,0,0,1;
      std::cout << "A: \n" << k_A << std::endl;
      std::cout << "C: \n" << k_C << std::endl;
      std::cout << "Q: \n" << k_Q << std::endl;
      std::cout << "R: \n" << k_R << std::endl;
      std::cout << "P: \n" << k_P << std::endl;
      // Construct the filter
      kf = new KalmanFilter(0, k_A, k_C, k_Q, k_R, k_P);

      Eigen::MatrixXd nmpc_A(n_states, n_states); // System dynamics matrix
      Eigen::MatrixXd nmpc_C(n_controls, n_states); // Output matrix
      Eigen::MatrixXd nmpc_Q(n_states, n_states); // Process noise covariance
      Eigen::MatrixXd nmpc_R(n_controls, n_controls); // Measurement noise covariance
      Eigen::MatrixXd nmpc_P(n_states, n_states); // Estimate error covariance
      nmpc_A << 1,0,0,0,1,0,0,0,1;
      nmpc_C << 1,0,0,0,1,0,0,0,1;
      
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_q", nmpc_Q);
      if(!passed){
        double q_cov = 1;
        nmpc_Q << q_cov, 0, .0, 0, q_cov, .0,0.0, 0.0, q_cov;
      }
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_r", nmpc_R);
      if(!passed){
        double r_cov = 1;
        nmpc_R << r_cov, 0, .0, 0, r_cov, .0, .0, 0, r_cov;
      }
      nmpc_P << 1,0,0,0,1,0,0,0,1;
      std::cout << "A: \n" << nmpc_A << std::endl;
      std::cout << "C: \n" << nmpc_C << std::endl;
      std::cout << "Q: \n" << nmpc_Q << std::endl;
      std::cout << "R: \n" << nmpc_R << std::endl;
      std::cout << "P: \n" << nmpc_P << std::endl;
      // Construct the filter
      kf_nmpc = new KalmanFilter(delta_t, nmpc_A, nmpc_C, nmpc_Q, nmpc_R, nmpc_P);
      rhs = vertcat(v_x*cos(0.0)-v_y*sin(0.0), v_y*cos(0.0) + v_x*sin(0.0), v_z);
      // f = Function("f", {states, controls}, {rhs}, {"x", "u"}, {"rhs"});
      f = Function::load(residual_dynamics_model);

      SX U = SX::sym("U", n_controls, prediction_horizon);
      SX P = SX::sym("P", n_states + prediction_horizon*(n_states+n_controls));
     
      SX Q = DM::zeros(n_states, n_states);
      SX R = DM::zeros(n_controls, n_controls);
      Eigen::MatrixXd Q_tmp(n_states, n_states); 
      passed = passer->passing_matrix("covariance_matrix_for_trajectory_tracker_q", Q_tmp);
      if(!passed){
        Q(0,0) = 1;
        Q(1,1) = 1;
        Q(2,2) = 1;
      }else{
        Q(0,0) = Q_tmp(0,0);
        Q(1,1) = Q_tmp(1,1);
        Q(2,2) = Q_tmp(2,2);
      }
      
      Eigen::MatrixXd R_tmp(n_states, n_states); 
      passed = passer->passing_matrix("covariance_matrix_for_trajectory_tracker_r", R_tmp);
      if(!passed){
        R(0,0) = 1;
        R(1,1) = 1;
        R(2,2) = 1;
        // R(3,3) = 0.05;
      }else{
        R(0,0) = R_tmp(0,0);
        R(1,1) = R_tmp(1,1);
        R(2,2) = R_tmp(2,2);
      }

      if(!use_collocation){
        obj = 0;
        g = SX::sym("g", prediction_horizon+1, n_states);
        X = SX::sym("X", n_states, prediction_horizon+1);
        SX st = X(Slice(0, X.size1()), Slice(0));

        g(Slice(0), Slice(0, g.size2())) = st - P(Slice(0, n_states));
        int ibj = 1;
        SX con = 0;
        int k_index = n_controls + n_states;
        for(int k=0; k<prediction_horizon; k++){
          st = X(Slice(0, X.size1()), Slice(k));
          con = U(Slice(0, U.size1()), Slice(k));
          
          // obj = obj + mtimes((st-P(Slice(n_states,n_states*2))).T(), mtimes(Q,(st-P(Slice(n_states,n_states*2))))) + mtimes(con.T(), mtimes(R, con));
          obj = obj + mtimes((st-P(Slice(k_index*(k+1)-n_controls, k_index*(k+1)))).T(), mtimes(Q,(st-P(Slice(k_index*(k+1)-n_controls, k_index*(k+1)))))) 
                + mtimes((con-P(Slice(k_index*(k+1), k_index*(k+1)+n_states))).T(), mtimes(R, con-P(Slice(k_index*(k+1), k_index*(k+1)+ n_states))));
          
          SX st_next = X(Slice(0, X.size1()), Slice(k+1));
          SXDict f_in = {{"x", st}, {"u", con}};
          SXDict f_value = f(f_in);
          SX st_next_euler = st + delta_t*f_value["rhs"];
          g(Slice(ibj), Slice(0, g.size2())) = st_next - st_next_euler;
          ibj += 1;
        }

        g = reshape(g, n_states*(prediction_horizon+1), 1);
        SX OPT_variables = vertcat(reshape(X, n_states*(prediction_horizon+1), 1), reshape(U, n_controls*prediction_horizon, 1));
        
        opts["ipopt.tol"] = 1e-3;
        opts["ipopt.max_iter"] = 1000;
        opts["expand"] = true;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.acceptable_tol"] = 1e-4;

        nlp_prob = {{"f",obj}, {"x",OPT_variables}, {"p", P}, {"g",g}};

        DM lbx = DM(n_states*(prediction_horizon+1)+n_controls*prediction_horizon, 1);
        DM ubx = DM(n_states*(prediction_horizon+1)+n_controls*prediction_horizon, 1);

        lbx(Slice(0, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[0];
        ubx(Slice(0, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[0];
        lbx(Slice(1, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[1];
        ubx(Slice(1, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[1];
        lbx(Slice(2, n_states*(prediction_horizon+1), n_states), Slice(0)) = min_range_[2];
        ubx(Slice(2, n_states*(prediction_horizon+1), n_states), Slice(0)) = max_range_[2];
        
        lbx(Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(prediction_horizon+1)+1, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(prediction_horizon+1)+1, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(prediction_horizon+1)+2, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(prediction_horizon+1)+2, n_states*(prediction_horizon+1) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;

        DM p(n_states*2);
        args["lbx"] = lbx;
        args["ubx"] = ubx;
        args["p"] = p;

      }else{
        
        cout<< "Number of points: "<<  collocation_degree << endl;
        C = vector<vector<double>>(collocation_degree+1,vector<double>(collocation_degree+1,0));
        D = vector<double>(collocation_degree+1, 0);
        B = vector<double>(collocation_degree+1, 0);
        int num_states = prediction_horizon + 1 + collocation_degree*prediction_horizon;
        getCollocationPoints(B, C, D);

        obj = 0;
        X = SX::sym("X", n_states, num_states);
        g = SX::sym("g", num_states, n_states);

        SX Xk = X(Slice(0, X.size1()), Slice(0));
        
        g(Slice(0), Slice(0, g.size2())) = Xk - P(Slice(0, n_states));
        // SX con = 0;
        int pos_index = 0;
        int constraints = 1;
        for(int k=0; k<prediction_horizon; k++){
          SX Uk = U(Slice(0, U.size1()), Slice(k));
          vector<SX> Xc;
          for(int j=0; j<collocation_degree; j++){
            SX Xkj = X(Slice(0, X.size1()), Slice(pos_index+1+j));
            Xc.push_back(Xkj);
          }
          pos_index += collocation_degree+1;
          SX Xk_end = D[0]*Xk;
          for (int j=1; j<collocation_degree+1; j++){
            SX xp = C[0][j]*Xk;
            for (int r=0; r<collocation_degree; r++){
              xp = xp + C[r+1][j]*Xc[r];
            }
            SXDict f_in = {{"x", Xc[j-1]}, {"u", Uk}};
            SXDict f_value = f(f_in);
            auto qj = mtimes((Xc[j-1]-P(Slice(n_states,n_states*2))).T(), mtimes(Q,(Xc[j-1]-P(Slice(n_states,n_states*2))))) + mtimes(Uk.T(), mtimes(R, Uk));
            g(Slice(constraints), Slice(0, g.size2())) = delta_t*f_value["rhs"] - xp;
            constraints +=1;
            Xk_end = Xk_end + D[j]*Xc[j-1];
            obj += B[j]*qj*delta_t;
          }
          Xk =  X(Slice(0, X.size1()), Slice(pos_index));
          g(Slice(constraints), Slice(0, g.size2())) = Xk_end-Xk;
          constraints +=1;
        }

        g = reshape(g, n_states*(num_states), 1);
        SX OPT_variables = vertcat(reshape(X, n_states*(num_states), 1), reshape(U, n_controls*prediction_horizon, 1));
        
        opts["ipopt.tol"] = 1e-3;
        opts["ipopt.max_iter"] = 1000;
        opts["expand"] = true;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.acceptable_tol"] = 1e-4;

        nlp_prob = {{"f",obj}, {"x",OPT_variables}, {"p", P}, {"g", g}};

        DM lbx = DM(n_states*(num_states)+n_controls*prediction_horizon, 1);
        DM ubx = DM(n_states*(num_states)+n_controls*prediction_horizon, 1);

        lbx(Slice(0, n_states*(num_states), n_states), Slice(0)) = min_range_[0];
        ubx(Slice(0, n_states*(num_states), n_states), Slice(0)) = max_range_[0];
        lbx(Slice(1, n_states*(num_states), n_states), Slice(0)) = min_range_[1];
        ubx(Slice(1, n_states*(num_states), n_states), Slice(0)) = max_range_[1];
        lbx(Slice(2, n_states*(num_states), n_states), Slice(0)) = min_range_[2];
        ubx(Slice(2, n_states*(num_states), n_states), Slice(0)) = max_range_[2];
        
        lbx(Slice(n_states*(num_states), n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(num_states), n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(num_states)+1, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(num_states)+1, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;
        lbx(Slice(n_states*(num_states)+2, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_min;
        ubx(Slice(n_states*(num_states)+2, n_states*(num_states) + n_controls*prediction_horizon, n_controls), Slice(0)) = v_max;

        DM p(n_states*2);
        args["lbx"] = lbx;
        args["ubx"] = ubx;
        args["p"] = p;
      }
      vehicle_current_state = std::make_shared<std::vector<double>>(); 
      edt_env_->initConvexSegment();
  }

  void TrajectoryTracker::mpc_solver(){
    if(use_collocation){
      mpc_solver_with_collocation();
    }else{
      mpc_solver_with_multiple_shooting();
    }
    return;
  }

  
  void TrajectoryTracker::global_solver(){
    clock_t t2, t1 = clock();
    std::cout<< "still_running_global_planner: "<< still_running_global_planner<< std::endl;
    while(still_running_global_planner){
      t2 = clock();
      if(!still_running){
        std::cout<< "Stop global planning" << std::endl;
        // still_running_global_planner =  false;
        break;
      }
      
      if(double(t2 - t1)/CLOCKS_PER_SEC > delta_t){

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
                    bool is_foud = false;
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
              std::cout<< "Path not set..." << std::endl;
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

  void TrajectoryTracker::mpc_solver_with_multiple_shooting(){

    double t0 = 0;
    vector<double> t;
    t.push_back(t0);
    DM U0 = repmat(u0, 1, prediction_horizon).T();
    DM X0 = repmat(x0, 1, prediction_horizon+1).T();
    int sim_time = simulated_duration;
    int mpciter = 0;
    vector<vector<double>> xx1, u_cl, xx0, xx;
    args["x0"] = x0;

    Eigen::VectorXd k_x0(n_states);
    k_x0 << 0, 0, 0;
    kf->init(0, k_x0);

    Eigen::VectorXd k_p1(3);
    k_p1 <<  (double)x0(0,0), (double)x0(1, 0), (double)x0(2, 0);
    kf_nmpc->init(0, k_p1);

    clock_t t2, t1 = clock();
    previos_pose << (double)x0(0, 0), (double)x0(1, 0), (double)x0(2, 0);

    double t_cmd_start, t_cmd_end;
    bspline_utils_->traj_pos_.getTimeSpan(t_cmd_start, t_cmd_end);
    bspline_utils_->retrieveTrajectory();
    time_traj_start = ros::Time::now();
    double error(norm_2(x0-xs));
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

    set_running_global_planner =  true;
    while(error > maximum_acceptable_error && mpciter < sim_time/delta_t){
        Eigen::Vector3d end_pose = horizon_control_points_total.col(horizon_control_points_total.cols()-1);
        Eigen::Vector3d start_pose((double)x0(0,0), (double)x0(1,0), (double)x0(2,0));
        error = (end_pose-start_pose).norm();
        t2 = clock();
        int k_index = n_controls + n_states;
        if(double(t2 - t1)/CLOCKS_PER_SEC > delta_t_solver){
          args["p"] = DM::zeros(n_states + prediction_horizon*(n_states+n_controls));
          args["p"](Slice(0, n_states)) = x0;
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
              force_terminate = true;
            }
            int num_cols = horizon_control_points_total.cols()-start_index;
            Eigen::MatrixXd ref_horizon_points = horizon_control_points_total.block(0, start_index, 3, num_cols);
            int size_of_pro = ref_horizon_points.cols();
            double distance = 1000000.0;
            int closest_index = 0;
            for(int w=0; w<size_of_pro; w++){
              double dis = std::abs((double)(ref_horizon_points.col(w).head(3) - current_odom_pose).norm());
              if((dis < distance) && (dis > 0.2)){
                distance = dis;
                closest_index = w;
              }
            }
            Eigen::MatrixXd waypoints = bspline_utils_->waypoints_list_;
            int closest_waypoint_index = 0;
            distance = 1000000.0;
            const Eigen::Vector3d cloest_point = ref_horizon_points.col(closest_index).head(3);
            for(int gw=0; gw<waypoints.cols(); gw++){
              double dis = std::abs((double)(waypoints.col(gw).head(3) - cloest_point).norm());
              if(dis < distance){
                distance = dis;
                closest_waypoint_index = gw;
              }
            }
            int number_of_points = waypoints.cols()-(closest_waypoint_index+1)+1;
            Eigen::MatrixXd projected_;
            std::vector<Eigen::Vector3d> projected_waypoints;
            projected_waypoints.push_back(current_odom_pose); 
            if(number_of_points < 4){
              Eigen::Vector3d next_pose_ = current_odom_pose;
              for(int jk=closest_waypoint_index; jk<waypoints.cols(); jk++){
                  Eigen::Vector3d mid_point = (next_pose_ + waypoints.col(jk).head(3))/2.0;
                  if(std::abs((mid_point-next_pose_).norm()) > 0.2){
                          projected_waypoints.push_back(mid_point);
                          next_pose_ = mid_point;
                  }
                  if(std::abs((mid_point-waypoints.col(jk).head(3)).norm()) > 0.2){
                     projected_waypoints.push_back(waypoints.col(jk).head(3));
                     next_pose_ = waypoints.col(jk).head(3);
                  }
              }
              projected_.resize(3, projected_waypoints.size());
              int index_ = 0;
              for(auto pose : projected_waypoints){
                projected_.block(0, index_, 3, 1) = pose;
                index_++;
              }
            }else{
              projected_.resize(3, waypoints.cols()+1);
              projected_.block(0, 0, 3, 1) = current_odom_pose;
              for(int i=0; i< waypoints.cols(); i++){
                projected_.block(0, i+1, 3, 1) = waypoints.col(i);
              }
            }
            if( projected_.cols()<4){
              std::cout<< "Selected waypoints: " << projected_ << std::endl;
              ROS_WARN("Close to end point, no planning..., (;");
              force_terminate = true;
            }else{
              std::cout<< "Selected waypoints: " << projected_ << std::endl;
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
              }else{
                ROS_WARN("Solver wont able to solve the trajectory..., (;");
                force_terminate = true;
              }
            }
            lk.unlock();
            condition_lock_on_local_to_global.notify_one();
          }
          
          Eigen::Vector3d previos_free_pose = current_odom_pose;
          if( bound_checker->current_pose_obs == 2){
              Eigen::Vector3d current_pose = previos_free_pose;
              int start_index_ = start_index;
              for(int k=0; k<prediction_horizon; k++){
                vis_horizon_desired.push_back(current_pose);
                double x_ref = current_pose[0];
                double y_ref = current_pose[1];
                double z_ref = current_pose[2];
                DM ref_state_vector = DM::zeros(1, n_states);
                DM ref_control_inputs = DM::zeros(1, n_states);
                ref_state_vector(0) = x_ref;
                ref_state_vector(1) = y_ref;
                ref_state_vector(2) = z_ref;
                ref_control_inputs(0) = ref_control_inputs(1) = ref_control_inputs(2) = 0;
                args["p"](Slice(k_index*(k+1)-n_controls, k_index*(k+1))) = ref_state_vector;
                args["p"](Slice(k_index*(k+1), k_index*(k+1)+n_states)) = ref_control_inputs;
                start_index_--;
              }
          }else{
              for(int k=0; k<prediction_horizon; k++){
                Eigen::Vector3d current_pose;
                if(global_opt_sudden_stop){
                  current_pose = previos_free_pose;
                }else if(start_index+k<end_index){
                    current_pose = horizon_control_points_total.col(start_index+k);
                }
                if(edt_env_->get_free_distance(current_pose)<avoidance_distance){
                  current_pose = previos_free_pose;
                }else{
                  previos_free_pose = current_pose;
                }
                if((current_odom_pose-current_pose).norm()>2.0){
                  current_pose = previos_free_pose;
                }
                vis_horizon_desired.push_back(current_pose);
                double x_ref = current_pose[0];
                double y_ref = current_pose[1];
                double z_ref = current_pose[2];
                DM ref_state_vector = DM::zeros(1, n_states);
                DM ref_control_inputs = DM::zeros(1, n_states);
                ref_state_vector(0) = x_ref;
                ref_state_vector(1) = y_ref;
                ref_state_vector(2) = z_ref;
                ref_control_inputs(0) = ref_control_inputs(1) = ref_control_inputs(2) = 0;
                args["p"](Slice(k_index*(k+1)-n_controls, k_index*(k+1))) = ref_state_vector;
                args["p"](Slice(k_index*(k+1), k_index*(k+1)+n_states)) = ref_control_inputs;
              }
          }

          vector<Eigen::VectorXd> obs_map_;
          Eigen::Vector3d obs;
          double obs_dis;
          Eigen::VectorXd obs_pose(4);
          edt_env_->get_close_obstacle(current_odom_pose, obs, obs_dis);
          double previous_dis = 2.0;
          if(obs_dis<edt_env_->sdf_map_->max_avoidance_distance_*2.0){
                obs_pose.head(3) = obs;
                obs_pose[3] = obs_dis;
                obs_map_.push_back(obs_pose);
                previous_dis = obs_dis;
          }
          for(int k=0; k<40; k++){
            Eigen::Vector3d current_pose;
            if(start_index+k<end_index){
                current_pose = horizon_control_points_total.col(start_index+k);
            }else{
                current_pose = horizon_control_points_total.col(end_index);
            }

            if(edt_env_->get_free_distance(current_pose)<avoidance_distance){
              current_pose = previos_free_pose;
            }else{
              previos_free_pose = current_pose;
            }

            Eigen::Vector3d obs;
            double obs_dis;
            Eigen::VectorXd obs_pose(4);
            edt_env_->get_close_obstacle(current_pose, obs, obs_dis);
            if(std::abs(obs_dis-previous_dis)< obs_min_allowed_length){
            //   cout<< "Obs at the same pose" << obs.transpose() << endl;
            }else{
              if(obs_dis>edt_env_->sdf_map_->max_avoidance_distance_*2.0){
                // std::cout<< "Longer obs, not taking into account, " << obs_dis << std::endl;
              }else{
                Eigen::VectorXd obs_pose_check(4);
                Eigen::Vector3d obs_cuu;
                double obs_dis_cuu;
                edt_env_->get_close_obstacle(obs, obs_cuu, obs_dis_cuu);
                if(obs_dis_cuu <= edt_env_->sdf_map_->max_avoidance_distance_){
                  obs_pose.head(3) = obs;
                  obs_pose[3] = obs_dis;
                  obs_map_.push_back(obs_pose);
                  previous_dis = obs_dis;
                }else{
                  std::cout<< "======removing wrong obstacle poses current: "<< obs.transpose() << std::endl;
                }
              }
            }
          }
          std::sort(std::begin(obs_map_), std::end(obs_map_)
                          , [](Eigen::VectorXd a, Eigen::VectorXd b) {return a[3] > b[3]; });
          if(obs_map_.size() <= obs_max_allowed_length){
            obs_length = obs_map_.size();
          }else{
            obs_length = obs_max_allowed_length;
          }
          obs_length = obs_map_.size();
          visualization_->drawPath(vis_horizon_desired, 0.2, Eigen::Vector4d(1, 0 ,0, 1), 234);
          visualization_->drawPath(vis_horizon_desired_velocities, 0.2, Eigen::Vector4d(1, 0 ,0, 1), 236);
          args["x0"] = vertcat(reshape(X0.T(), n_states*(prediction_horizon+1), 1), reshape(U0.T(), n_controls*prediction_horizon, 1));
        
          DM lbg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
          DM ubg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
          lbg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
          ubg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
          lbg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) 
                                        + (prediction_horizon+1)*obs_length)) = -inf;
          ubg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) 
                                        + (prediction_horizon+1)*obs_length)) = 0;
          
          args["lbg"] = lbg; 
          args["ubg"] = ubg;
          int obs_cout = 0;
          SX obs_g = SX::sym("obs_g",(prediction_horizon+1)*obs_length);
          for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
            SX st = X(Slice(0, X.size1()), Slice(trj_i));
            for(int obs_index = 0; obs_index<obs_length; obs_index++){
              Eigen::VectorXd obs = obs_map_[obs_index];
              obs_g(obs_cout) = -sqrt(pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + avoidance_distance;
              obs_cout++;
            }
          }
          std::vector<Eigen::Vector3d> vis_obs;
          for(auto obs : obs_map_){
            vis_obs.push_back(obs.head(3));
          }
          visualization_->drawPath(vis_obs, 0.2, Eigen::Vector4d(0.0, 0.2 ,0.9, 1), 239);

          SX final_g = vertcat(g, obs_g);
          nlp_prob["g"] = final_g;
 
          Eigen::Vector3d real_vel = { real_odom.twist.twist.linear.x, real_odom.twist.twist.linear.y, real_odom.twist.twist.linear.z};
          
          std::vector<double> state_in = {kf->state()[0], kf->state()[1], kf->state()[2], real_vel[0], real_vel[1], real_vel[2] };
          Function solver = nlpsol("nlpsol", "ipopt", nlp_prob, opts);
          res = solver(args);
          DM u = reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T();
          vector<double> u11(reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T());
          vector<double> xx11(res["x"](Slice(0, (n_states*(prediction_horizon+1)))).T());
          xx1.push_back(xx11);
          std::vector<Eigen::Vector3d> vis_horizon;

          for(int kl=0; kl< int(xx11.size()/n_states); kl+=3){
            Eigen::Vector3d poss(xx11[kl], xx11[kl+1], xx11[kl+2]);
            vis_horizon.push_back(poss);
          }
          visualization_->drawPath(vis_horizon, 0.2, Eigen::Vector4d(0.2, 1 ,0.5, 1), 0);
          u_cl.push_back(u11);
          t.push_back(t0);

          DM current_control = u(Slice(0), Slice(0, u.size2())).T();
          DM current_pose = x0;
        
          if(force_terminate){
            still_running = false;
            std::cout<< "Solver has been interrupted..." <<  std::endl;
            return;
          }
          Eigen::VectorXd k_y(n_controls);
          std::vector<double> k_estimated_values;
          k_y << (double)current_control(0), (double)current_control(1), (double)current_control(2);
          kf->update(k_y);

          Eigen::MatrixXd B_(n_controls, n_controls);
          B_ << (double)cos(0.0), (double)-sin(0.0), 0,  (double)sin(0.0), (double)cos(0.0), 0, 0, 0, 1;
          B_ = delta_t*B_;
          Eigen::VectorXd k_p_(n_states);
          k_p_ << (double)x0(0), (double)x0(1), (double)x0(2);

          kf_nmpc->update(k_p_, B_, k_y);

          x0(0,0) = current_projected_pose.pose.pose.position.x = kf_nmpc->state()[0];
          x0(1,0) = current_projected_pose.pose.pose.position.y = kf_nmpc->state()[1];
          x0(2,0) = current_projected_pose.pose.pose.position.z = kf_nmpc->state()[2];
          
          current_projected_pose.twist.twist.linear.x = kf->state()[0];
          current_projected_pose.twist.twist.linear.y = kf->state()[1];
          current_projected_pose.twist.twist.linear.z = kf->state()[2];
          current_projected_pose.twist.twist.angular.z = 0.0;
          
          Eigen::Vector3d pos;
          pos(0) = current_projected_pose.pose.pose.position.x;
          pos(1) = current_projected_pose.pose.pose.position.y;
          pos(2) = current_projected_pose.pose.pose.position.z;
          traj_cmd->push_back(pos);
          visualization_->displayTrajWithColor(*traj_cmd, 0.03, Eigen::Vector4d(1, 1, 0, 1), 21);
          pos_current_pos_pub.publish(current_projected_pose);
          tuple<double, DM, DM> shiftted_val = shift(delta_t, t0, x0, u, f); 
          x0 = get<1>(shiftted_val);
          u = get<2>(shiftted_val);
          t0 = get<0>(shiftted_val);

          X0 = reshape(res["x"](Slice(0, n_states*(prediction_horizon+1))).T(), n_states, prediction_horizon+1).T();
          vector<double> xxxo(x0);
          xx0.push_back(xxxo);

          SX x0_rest = X0(Slice(1, X0.size1()), Slice(0, X0.size2()));
          SX x0_last = X0(Slice(X0.size1()-1, X0.size1()), Slice(0, X0.size2()));
          X0 = vertcat(x0_rest, x0_last);
          mpciter = mpciter + 1;

          t1 = clock();
          double time_delat = double(t2 - t1)/CLOCKS_PER_SEC;
          std::vector<double> state_out = {kf->state()[0], kf->state()[1], kf->state()[2], time_delat, 0, 0};
          planner_saving->save_training_set(state_in, state_out);
        }else{
          usleep(5000);
        }
    }

    still_running = false;
    need_intermediate_goal = false;
    cout<< "Finish mpc solver...."<< endl;
  }

  double TrajectoryTracker::get_distance(Eigen::MatrixXd norminal, Eigen::MatrixXd actual){
    double dis = 0;
    for(int i=0; i< norminal.cols(); i++){
      dis += (norminal.col(i)-actual.col(i)).norm();
    }
    return dis;
  }

  void TrajectoryTracker::mpc_solver_with_collocation(){

  }

  void TrajectoryTracker::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    reference_odom = *msg;
    is_reference_trj_set = true;
  }

  void TrajectoryTracker::odomCallbackReal(const nav_msgs::OdometryConstPtr& msg){
    real_odom = *msg;
    is_real_odom_set = true;
  }


  void TrajectoryTracker::init(ros::NodeHandle& nh){
      node_ = nh;
      NonLinearMPCOpt::init(nh);
      node_.param("mpc_opt/delta_t_desired", delta_t_desired, 0.02);
      node_.param("mpc_opt/reference_trj_horizon", reference_trj_horizon, 40);
      node_.param("mpc_opt/delta_t_solver", delta_t_solver, 0.05);
      
      node_.param<std::string>("mpc_opt/residual_dynamics_model", residual_dynamics_model, "/root/path");
      
      odometry_sub_ref_ = node_.subscribe<nav_msgs::Odometry>("/odom", 50, &TrajectoryTracker::odomCallback, this);
      odometry_sub_real_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 50, &TrajectoryTracker::odomCallbackReal, this);
      
      bspline_intermediate_.reset(new BSplineUtils);
      bspline_intermediate_->setParam(nh);
      bspline_intermediate_->visualization_.reset(new PlanningVisualization(nh));
      
      solver_init();
  }

}  // namespace hagen_planner
