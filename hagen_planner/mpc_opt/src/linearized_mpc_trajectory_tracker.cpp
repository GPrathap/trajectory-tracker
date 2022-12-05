#include "mpc_opt/linearized_mpc_trajectory_tracker.h"

namespace hagen_planner
{
  LinearizedMPCTrajectoryTracker::LinearizedMPCTrajectoryTracker()
  {
    
  }

  void LinearizedMPCTrajectoryTracker::setTrajectoryGenerator(const BSplineUtils::Ptr& manager){
    bspline_utils_ = manager;
  }

  void LinearizedMPCTrajectoryTracker::setIntermediateTrajectoryGenerator(const BSplineUtils::Ptr& manager){
    bspline_intermediate_ = manager;
  }

  void LinearizedMPCTrajectoryTracker::setBoundaryChecker(const hagen_planner::BsplineOptimizer::Ptr& opt){
    bound_checker = opt;
  }

  void LinearizedMPCTrajectoryTracker::setInitModel(int starting_index){
    Waypoint current_wp;
    model.get_current_waypoint(starting_index, current_wp);
    model.spatial_state.e_psi = 0;
    model.spatial_state.e_y  = 0; 
    model.spatial_state.t = 0;
    model.temporal_state = model.s2t(current_wp, model.spatial_state);
  }

  bool LinearizedMPCTrajectoryTracker::getControl(int starting_index){

      int nx = numState;
      int nu = numInput;
      Waypoint current_wp;
      model.get_current_waypoint(starting_index, current_wp);
      model.current_waypoint = current_wp;
      Eigen::MatrixXd control_policy(1, numInput*prediction_horizon);
      model.spatial_state = model.t2s( model.current_waypoint, model.temporal_state);
     
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1), numState*(prediction_horizon+1));
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1), numInput*prediction_horizon);
      Eigen::MatrixXd ur = Eigen::MatrixXd::Zero(numInput*(prediction_horizon), 1);
      Eigen::MatrixXd uq = Eigen::MatrixXd::Zero(numState*(prediction_horizon), 1);
      Eigen::MatrixXd xr = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1), 1);

      Eigen::MatrixXd fg = current_control.block(0, 3, 1, numInput*prediction_horizon-3);
      fg = fg.array() + current_control(0, numInput*prediction_horizon-1);
      kappa_pred = (1/model.height)*fg.array().tan();
      for(int n=0; n<prediction_horizon; n++){
        Waypoint current_wp;
        Waypoint next_waypoint;
        model.get_current_waypoint_index(starting_index + n, current_wp);
        model.get_current_waypoint_index(starting_index + n+1, next_waypoint);
        double delta_s = next_waypoint.sub(current_wp);
        double kappa_ref = current_wp.kappa_;
        double v_ref = current_wp.v_ref;
        Eigen::MatrixXd A_lin = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd B_lin = Eigen::MatrixXd::Zero(3, 2);
        Eigen::MatrixXd ref_pose = Eigen::MatrixXd::Zero(2, 1);
        ref_pose << v_ref, kappa_ref;
        Eigen::Vector3d f;
        model.linearized(v_ref, kappa_ref, delta_s, A_lin, B_lin, f);
        A.block(numState*(n+1), numState*(n), numState, numState) = A_lin;
        B.block(numState*(n+1), numInput*(n), numState, numInput) = B_lin;
        ur.block(n*numInput, 0, numInput, 1) << v_ref, kappa_ref;
        uq.block(n*numState, 0, numState, 1) = (B_lin*(ref_pose)) - f;
        double vmax_dyn = std::sqrt(ay_max/(std::abs(kappa_pred(0, n)+1e-12)));
        if(vmax_dyn < uu.coeffRef(numInput*n, 0)){
          uu.coeffRef(numInput*n, 0) = vmax_dyn;
        }
      }
    
      Eigen::MatrixXd ub_;
      Eigen::MatrixXd lb_;
      model.reference_path.update_path_constraints(starting_index+1, prediction_horizon, 0.5, 0.141421, ub_, lb_);

      Eigen::MatrixXd center_line = (ub_ + lb_)/2.0;
      int state_index = 0;
      for(int n=0; n<prediction_horizon; n++){
        xr.block(state_index, 0, numState, 1)<< center_line(0, n), 0, 0;
        state_index += numState;
      }

      Eigen::SparseMatrix<c_float> AA;
      castMPCToQPConstraintMatrix(A, B, AA);
      Eigen::MatrixXd x0(3,1);
      x0 << model.spatial_state.e_y, model.spatial_state.e_psi, model.spatial_state.t;

      Eigen::VectorXd lowerBound;
      Eigen::VectorXd upperBound;

      castMPCToQPConstraintVectors(x_max, x_min, u_max, u_min, x0, uq, lowerBound, upperBound);
    
      Eigen::MatrixXd Q_didg = Eigen::MatrixXd(model.Q).diagonal();
      Eigen::MatrixXd QN_didg = Eigen::MatrixXd(model.QN).diagonal();
      Eigen::MatrixXd R_didg = Eigen::MatrixXd(model.R).diagonal();
      Eigen::MatrixXd state_cost;
      Eigen::MatrixXd state_cost_p = -1.0*xr.block(0,0, prediction_horizon*numState, 1);
      tile(prediction_horizon, Q_didg, state_cost_p, state_cost);
    
      Eigen::MatrixXd control_cost;
      ur = -1.0*ur.array();
      tile(prediction_horizon, R_didg, ur, control_cost);
      Eigen::MatrixXd terminal_cost;
      state_cost_p = -1.0*xr.block(prediction_horizon*numState, 0, numState, 1);
      tile(1, QN_didg, state_cost_p, terminal_cost);
      Eigen::SparseMatrix<c_float> q;
      q.resize((prediction_horizon+1)*numState + prediction_horizon*numInput, 1);

      for (int i = 0; i < state_cost.rows(); i++){
        q.insert(i,0) = state_cost(i, 0);
      }

      for (int i = state_cost.rows(); i < state_cost.rows()+terminal_cost.rows(); i++)
      {
        q.insert(i,0) = terminal_cost(i-state_cost.rows(), 0);
      }

      for (int i = state_cost.rows() + terminal_cost.rows(); i < state_cost.rows() + terminal_cost.rows() + control_cost.rows(); i++)
      {
        q.insert(i, 0) = control_cost(i-(state_cost.rows()+terminal_cost.rows()), 0);
      }
    
      Eigen::VectorXd gradient(q.rows());
      for(int i=0; i< q.rows(); i++){
          gradient[i] = q.coeffRef(i, 0);
      }
      OsqpEigen::Solver qp_solver;
      qp_solver.settings()->setVerbosity(false);
      qp_solver.settings()->setWarmStart(true); 
      qp_solver.data()->setNumberOfVariables(P.rows());
      qp_solver.data()->setNumberOfConstraints(AA.rows());
      qp_solver.data()->setHessianMatrix(P);
      qp_solver.data()->setGradient(gradient);
      qp_solver.data()->setLinearConstraintsMatrix(AA);
      qp_solver.data()->setLowerBound(lowerBound);
      qp_solver.data()->setUpperBound(upperBound);
      qp_solver.initSolver();
      set_init_solver =  true;
      bool set_correct = true;
      if(!qp_solver.solve()) set_correct = false;
      Eigen::VectorXd QPSolution;
       Eigen::MatrixXd ffff(1, numInput*prediction_horizon);
      if(set_correct){
          int h = 0;
          QPSolution = qp_solver.getSolution();
          for(int j=numState*(prediction_horizon+1); j<(numState*(prediction_horizon+1))+numInput*prediction_horizon; j+=2){
              control_policy(0, h) = QPSolution[j];
              control_policy(0, h+1) = std::atan(QPSolution[j+1]*model.height);
              ffff(0, h) =  QPSolution[j];
              ffff(0, h+1) =  QPSolution[j+1];
              h+=2;
          }
          current_control = control_policy;
          h = 0;
          for(int j=0; j< numState*(prediction_horizon+1); j+=3){
              predicted_states.col(h) << QPSolution[j], QPSolution[j+1], QPSolution[j+2];
              h++;
          }
          infeasibility_counter = 0;
          current_u<< current_control(0,0), current_control(0,1);
          set_solver_ = 1;
      }else{
        std::cout << "Error while solving... using previous control...." << std::endl;
        if(set_solver_ == 0){
          std::cout<< "Problem can not be solved...." << std::endl;
          return false; 
        }
        int id = numInput*(infeasibility_counter+1);
        auto current_u_ = current_control.block(0, id, 1, 2);
        current_u << current_u_(0,0), current_u_(0,1);
        infeasibility_counter += 1;
    }

    if(infeasibility_counter == prediction_horizon-1){
      std::cout<< "Problem can not be solved...." << std::endl;
      return false;
    }
    return true;
  }

  void LinearizedMPCTrajectoryTracker::visualizeStatePediction(){
    visualization_->drawPath(rebound_array, 0.2, Eigen::Vector4d(0.5, 0.2 ,0.3, 1), 136);
  }

  void LinearizedMPCTrajectoryTracker::updatePrediction(int starting_index){
    rebound_array.clear();
    for (int n = 0; n < prediction_horizon; n++){
      Waypoint current_wp;
      model.get_current_waypoint_index(starting_index + n, current_wp);
      SpatialState spatial_state_prediction;
      spatial_state_prediction.e_y = predicted_states(0, n);
      spatial_state_prediction.e_psi = predicted_states(1, n);
      spatial_state_prediction.t = predicted_states(2, n);
      TemporalState predicted_temporal_state = model.s2t(current_wp, spatial_state_prediction);
      Eigen::Vector3d next_pose(predicted_temporal_state.x_, predicted_temporal_state.y_, 0.0);
      rebound_array.push_back(next_pose);
    }
  }

  void LinearizedMPCTrajectoryTracker::init_problme(int starting_index, OsqpEigen::Solver& solver){
  }

  void LinearizedMPCTrajectoryTracker::solver_init(){
      initStatus = true;
      Eigen::MatrixXd Q_tmp(numState, numState); 
      bool passed = false;
      passed = passer->passing_matrix("covariance_matrix_for_trajectory_tracker_q", Q_tmp);
      if(passed){
        model.Q.resize(Q_tmp.rows(), Q_tmp.cols());
        model.Q.setZero();
        for(int b=0; b<numState; b++){
          model.Q.coeffRef(b, b) = Q_tmp(b, b);
        }
      }

      passed = false;
      passed = passer->passing_matrix("covariance_matrix_for_trajectory_tracker_qn", Q_tmp);
      if(passed){
        model.QN.resize(Q_tmp.rows(), Q_tmp.cols());
        model.QN.setZero();
        for(int b=0; b<numState; b++){
          model.QN.coeffRef(b,b) = Q_tmp(b, b);
        }
      }
      
      Eigen::MatrixXd R_tmp(numInput, numInput); 
      passed = passer->passing_matrix("covariance_matrix_for_trajectory_tracker_r", R_tmp);
      if(passed){
        model.R.resize(R_tmp.rows(), R_tmp.cols());
        model.R.setZero();
        for(int b=0; b<numInput; b++){
          model.R.coeffRef(b,b) = R_tmp(b, b);
        }
      }
      
      Eigen::MatrixXd state_constraints(1, 1); 
      passed = passer->passing_matrix("thetaMax", state_constraints);
      if(passed){
        thetaMax = state_constraints(0,0);
      }

      passed = passer->passing_matrix("v_max", state_constraints);
      if(passed){
        v_max = state_constraints(0,0);
      }

      passed = passer->passing_matrix("delta_max", state_constraints);
      if(passed){
        delta_max = state_constraints(0,0);
      }

      passed = passer->passing_matrix("ay_max", state_constraints);
      if(passed){
        ay_max = state_constraints(0,0);
      }

      passed = passer->passing_matrix("a_min", state_constraints);
      if(passed){
        a_min = state_constraints(0,0);
      }

      passed = passer->passing_matrix("a_max", state_constraints);
      if(passed){
        a_max = state_constraints(0,0);
      }

      passed = passer->passing_matrix("height", state_constraints);
      if(passed){
        model.height = state_constraints(0,0);
      }

      passed = passer->passing_matrix("width", state_constraints);
      if(passed){
        model.width = state_constraints(0,0);
      }
      u_min.resize(numInput, 1);
      u_min << 0.0, -std::tan(delta_max)/model.height;
      u_max.resize(numInput, 1);
      u_max << v_max, std::tan(delta_max)/model.height;
      x_min.resize(numState, 1);
      x_min << -OSQP_INFTY, -OSQP_INFTY, -OSQP_INFTY;
      x_max.resize(numState, 1);
      x_max << OSQP_INFTY, OSQP_INFTY, OSQP_INFTY;
      A.resize(numState*(prediction_horizon+1), numState*(prediction_horizon+1));
      xr.resize(numState*(prediction_horizon+1), 1);
      ur.resize(numState*(prediction_horizon), 1);
      xl.resize((prediction_horizon+1)*numState, 1);
      xu.resize((prediction_horizon+1)*numState, 1);
      ul.resize((prediction_horizon)*numInput, 1);
      uu.resize((prediction_horizon)*numInput, 1);
      uq.resize(prediction_horizon, numState);
      Eigen::SparseMatrix<double> stateUs(numState,1);
      Eigen::SparseMatrix<double> stateLs(numState,1);
      Eigen::SparseMatrix<double> inputUs(numInput,1);
      Eigen::SparseMatrix<double> inputLs(numInput,1);
      for (int i=0; i<numState; ++i){
          if(i==0){
            stateUs.coeffRef(i,0) = 0;
            stateLs.coeffRef(i,0) = -0;
          }else{
            stateUs.coeffRef(i,0) = 1000000;
            stateLs.coeffRef(i,0) = -1000000;
          }
      }

      for (int i=0; i<numInput; ++i){
          inputUs.coeffRef(i,0) = u_max(i,0);
          inputLs.coeffRef(i,0) = u_min(i,0);
      }
      Eigen::SparseMatrix<double> ones_(prediction_horizon, 1);
      for(int i=0; i<prediction_horizon; i++){
        ones_.coeffRef(i,0) = 1.0;
      }
      Eigen::SparseMatrix<double> ones_i(prediction_horizon+1, 1);
      for(int i=0; i<prediction_horizon+1; i++){
          ones_i.coeffRef(i,0) = 1.0;
      }
      xl = Eigen::kroneckerProduct(ones_i, stateLs).eval();
      ul = Eigen::kroneckerProduct(ones_, inputLs).eval();
      xu = Eigen::kroneckerProduct(ones_i, stateUs).eval();
      uu = Eigen::kroneckerProduct(ones_, inputUs).eval();
      current_control = Eigen::MatrixXd::Zero(1, numInput*prediction_horizon);
      predicted_states.resize(3, numState*(prediction_horizon+1));
      Eigen::SparseMatrix<c_float> cost_;
      cost_.resize(prediction_horizon, prediction_horizon);
      cost_.setIdentity();
      Eigen::SparseMatrix<c_float> stateCost =  Eigen::kroneckerProduct(cost_,  model.Q).eval();
      Eigen::SparseMatrix<c_float> controlCost =  Eigen::kroneckerProduct(cost_,  model.R).eval();
      P.resize((prediction_horizon+1)*numState+numInput*prediction_horizon, (prediction_horizon+1)*numState+numInput*prediction_horizon);
      P.setZero();
      for(int i=0; i<(prediction_horizon)*numState; i+=numState){
        for(int j=0; j<numState; j++){
          for(int k=0; k<numState; k++){
            P.coeffRef(i+j, i+k) = stateCost.coeffRef(i+j, i+k);
          }
        }
      }
      for(int i=(prediction_horizon)*numState; i<(prediction_horizon+1)*numState; i+=numState){
        for(int j=0; j<numState; j++){
          for(int k=0; k<numState; k++){
            P.coeffRef(i+j, i+k) =  model.QN.coeffRef(j, k);
          }
        }
      }
      int index = 0;
      for(int i=(prediction_horizon+1)*numState; i<(prediction_horizon+1)*numState+numInput*prediction_horizon; i+=numInput){
        for(int j=0; j<numInput; j++){
          for(int k=0; k<numInput; k++){
            P.coeffRef(i+j, i+k) = controlCost.coeffRef(index+j, index+k);
          }
        }
        index += numInput;
      }
      Eigen::MatrixXd k_A(numState, numState); // System dynamics matrix
      Eigen::MatrixXd k_C(numInput, numState); // Output matrix
      Eigen::MatrixXd k_Q(numState, numState); // Process noise covariance
      Eigen::MatrixXd k_R(numInput, numInput); // Measurement noise covariance
      Eigen::MatrixXd k_P(numState, numState); // Estimate error covariance
      k_A << 1,0,0,0,1,0,0,0,1;
      k_C << 1,0,0,0,1,0;
      passed = passer->passing_matrix("covariance_matrix_for_control_input_q", k_Q);
      if(!passed){
        double q_cov = 1;
        k_Q << q_cov, 0, .032, 0, 0, q_cov, .0, .032, .08,0.0, q_cov, 0, 0,0,0,1;
      }
      passed = passer->passing_matrix("covariance_matrix_for_control_input_r", k_R);
      if(!passed){
        double r_cov = 5000;
        k_R << r_cov, 0, .0, 0, 0, r_cov, .0, .0, .0, 0, r_cov, 0, 0,0,0, r_cov;
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
      nmpc_C << 1,0,0,0,1,0;
      
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_q", nmpc_Q);
      if(!passed){
        double q_cov = 1;
        nmpc_Q << q_cov, 0, .032, 0, 0, q_cov, .0, .032, .08,0.0, q_cov, 0, 0,0,0,1;
      }
      passed = passer->passing_matrix("covariance_matrix_for_nmpc_r", nmpc_R);
      if(!passed){
        double r_cov = 1;
        nmpc_R << r_cov, 0, .0, 0, 0, r_cov, .0, .0, .0, 0, r_cov, 0, 0,0,0, r_cov;
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
  }

  void LinearizedMPCTrajectoryTracker::mpc_solver(){
    mpc_solver_with_multiple_shooting();
    return;
  }
  
  void LinearizedMPCTrajectoryTracker::global_solver(){
    clock_t t2, t1 = clock();
    std::cout<< "still_running_global_planner: "<< still_running_global_planner<< std::endl;
    while(still_running_global_planner){
      t2 = clock();
      if(!still_running){
        std::cout<< "Stop global planning" << std::endl;
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
                    std::vector<Eigen::Vector3d> reference_rebound_traj;
                    for(int jkl=0; jkl<ref_horizon_points.cols(); jkl++){
                      auto row = ref_horizon_points.col(jkl);
                      Eigen::Vector3d row_vector(row[0], row[1], row[2]);
                      reference_rebound_traj.push_back(row_vector);
                    }
                    if(next_free_index <= horizon_control_points_total.cols()){
                      if(instance_index-start_in < 0){
                        horizon_control_points_total.block(0, instance_index, 3, next_free_index-instance_index) 
                                                  = ref_horizon_points.block(0,start_in,3,next_free_index-instance_index);
                      }else{
                        horizon_control_points_total.block(0, instance_index-start_in, 3, next_free_index-(instance_index-start_in)) 
                                                  = ref_horizon_points;
                      }
                        visualization_->drawPath(reference_rebound_traj, 0.2, Eigen::Vector4d(0.6, 0.2 ,0.8, 1), 230);
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

  void LinearizedMPCTrajectoryTracker::mpc_solver_with_multiple_shooting(){

    clock_t t2, t1 = clock();
    double current_time;

    double tm, tmp;
    auto trajectory = bspline_utils_->traj_pos_;
    trajectory.getTimeSpan(tm, tmp);

    std::vector<Eigen::Vector3d> horio_;
    for (double t = tm; t <= tmp; t += delta_t_desired){
      Eigen::Vector3d pt = trajectory.evaluateDeBoor(t);
      horio_.push_back(pt);
    }
    int k = 0;
    int depth = 2;
    bool use_3d = false;
    Eigen::MatrixXd horizon_control_points_total;
    horizon_control_points_total.resize(depth, horio_.size());
    for (auto pose : horio_){
      horizon_control_points_total.col(k) << pose.head(depth);
      k++;
    }
    Eigen::MatrixXd horizon_control_points_total_smoothed;
    model.reference_path.refine_trajectory(5, 0.05, horizon_control_points_total, horizon_control_points_total_smoothed, use_3d);
    model.reference_path.construct_waypoints(horizon_control_points_total_smoothed, use_3d);

    setInitModel(0);
    model.reference_path.calculate_velocity_profile(a_max, a_min, v_max, v_min, ay_max);
    
    ros::Time time_traj_start = ros::Time::now();
    bool is_feasible = true;
    set_solver_ = 0;
    int static_counter  = 0;
    current_control = Eigen::MatrixXd::Zero(1, numInput*prediction_horizon);
    int start_index = 0;
    while(true){
        t2 = clock();
        int k_index = numInput + numState;
        std::vector<Eigen::Vector3d> vis_horizon_desired;
        std::vector<Eigen::Vector3d> vis_horizon_desired_velocities;
        Waypoint current_wp;
        model.get_current_waypoint(start_index, current_wp);
        is_feasible = getControl(start_index);
        if(is_feasible == false){
          force_terminate = true;
          std::cout<< "Problem is infeasible..." <<  std::endl;
        }
        if(force_terminate){
            still_running = false;
            std::cout<< "Solver has been interrupted..." <<  std::endl;
            return;
        }
        
        updatePrediction(start_index);
        visualizeStatePediction();
        auto next_state_ = model.drive(current_u);
        current_projected_pose.pose.pose.position.x = rebound_array[0][0];
        current_projected_pose.pose.pose.position.y = rebound_array[0][1];
        current_projected_pose.pose.pose.position.z = rebound_array[0][2];
        current_projected_pose.twist.twist.linear.x = current_u[0]*std::cos(model.temporal_state.psi_);
        current_projected_pose.twist.twist.linear.y = current_u[0]*std::sin(model.temporal_state.psi_);
        current_projected_pose.twist.twist.linear.z = 0.0;
        current_projected_pose.twist.twist.angular.z = model.temporal_state.psi_;
        current_projected_pose.twist.twist.angular.x = current_u[1];
        pos_current_pos_pub.publish(current_projected_pose);
        static_counter++;
        t1 = clock();
    }
    
    still_running = false;
    need_intermediate_goal = false;
    cout<< "Finish mpc solver...."<< endl;
  }

  double LinearizedMPCTrajectoryTracker::get_distance(Eigen::MatrixXd norminal, Eigen::MatrixXd actual){
    double dis = 0;
    for(int i=0; i< norminal.cols(); i++){
      dis+=(norminal.col(i)-actual.col(i)).norm();
    }
    return dis;
  }

  void LinearizedMPCTrajectoryTracker::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    reference_odom = *msg;
    is_reference_trj_set = true;
  }

  void LinearizedMPCTrajectoryTracker::odomCallbackReal(const nav_msgs::OdometryConstPtr& msg){
    real_odom = *msg;
    is_real_odom_set = true;
  }

  void LinearizedMPCTrajectoryTracker::init(ros::NodeHandle& nh){
      node_ = nh;
      LinearizedMPCOpt::init(nh);
      node_.param("mpc_opt/delta_t_desired", delta_t_desired, 0.02);
      node_.param("mpc_opt/reference_trj_horizon", reference_trj_horizon, 40);
      odometry_sub_ref_ = node_.subscribe<nav_msgs::Odometry>("/odom", 50, &LinearizedMPCTrajectoryTracker::odomCallback, this);
      odometry_sub_real_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 50, &LinearizedMPCTrajectoryTracker::odomCallbackReal, this);
      
      bspline_intermediate_.reset(new BSplineUtils);
      bspline_intermediate_->setParam(nh);
      bspline_intermediate_->visualization_.reset(new PlanningVisualization(nh));
      
      solver_init();
  }

}  // namespace hagen_planner
