#include "mpc_opt/trajectory_regulator.h"
#include <ros/ros.h>

namespace hagen_planner
{
  TrajectoryRegulator::TrajectoryRegulator()
  {
    
  }

  void TrajectoryRegulator::solver_init(){
      MX x = MX::sym("x");
      MX y = MX::sym("y");
      MX z = MX::sym("z");
      states = vertcat(x, y, z);
      n_states = states.size1();

      MX v_x = MX::sym("v_x");
      MX v_y = MX::sym("v_y");
      MX v_z = MX::sym("v_z");
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
      f = Function("f", {states, controls}, {rhs}, {"x", "u"}, {"rhs"});

      SX U = SX::sym("U", n_controls, prediction_horizon);
      SX P = SX::sym("P", n_states + n_states);

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
      }else{
        R(0,0) = R_tmp(0,0);
        R(1,1) = R_tmp(1,1);
        R(2,2) = R_tmp(2,2);
      }
      
      if(!use_collocation){
        std::cout<< "Intializing multiple shooting ... n_states: " << n_states << endl;
        obj = 0;
        X = SX::sym("X", n_states, prediction_horizon+1);
        g = SX::sym("g", prediction_horizon+1, n_states);
        SX st = X(Slice(0, X.size1()), Slice(0));

        g(Slice(0), Slice(0, g.size2())) = st - P(Slice(0, n_states));
        int ibj = 1;
        SX con = 0;
        for(int k=0; k<prediction_horizon; k++){
          st = X(Slice(0, X.size1()), Slice(k));
          con = U(Slice(0, U.size1()), Slice(k));
          obj = obj + mtimes((st-P(Slice(n_states,  n_states*2))).T(), mtimes(Q,(st-P(Slice(n_states,n_states*2))))) + mtimes(con.T(), mtimes(R, con));
          SX st_next = X(Slice(0, X.size1()), Slice(k+1));
          SXDict f_in = {{"x", st}, {"u", con}};
          SXDict f_value = f(f_in);
          SX st_next_euler = st + delta_t*f_value["rhs"];
          g(Slice(ibj), Slice(0, g.size2())) = st_next - st_next_euler;
          ibj += 1;
        }

        g = reshape(g, n_states*(prediction_horizon+1), 1);
        SX OPT_variables = vertcat(reshape(X, n_states*(prediction_horizon+1), 1), reshape(U, n_controls*prediction_horizon, 1));
        
        opts["ipopt.tol"] = 1e-4;
        opts["ipopt.max_iter"] = 1000;
        opts["expand"] = true;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.acceptable_tol"] = 1e-8;

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

      }
      vehicle_current_state = std::make_shared<std::vector<double>>();  
  }


  void TrajectoryRegulator::mpc_solver(){
    if(use_collocation){
      mpc_solver_with_collocation();
    }else{
      mpc_solver_with_multiple_shooting();
    }
    return;
  }

  void TrajectoryRegulator::mpc_solver_with_multiple_shooting(){

    double t0 = 0;
    vector<double> t;
    t.push_back(t0);
    DM U0 = repmat(u0, 1, prediction_horizon).T();
    DM X0 = repmat(x0, 1, prediction_horizon+1).T();
    int sim_time = simulated_duration;
    int mpciter = 0;
    vector<vector<double>> xx1, u_cl, xx0, xx;
    args["x0"] = x0;

    double error(norm_2(x0-xs));

    Cumulative_Sum<double, double, 5> cumulative_sum_checker;
    still_running = true;
    Eigen::VectorXd k_x0(n_states);
    k_x0 << 0, 0, 0;
    kf->init(0, k_x0);

    Eigen::VectorXd k_p1(3);
    k_p1 <<  (double)x0(0,0), (double)x0(1,0), (double)x0(2,0);
    kf_nmpc->init(0, k_p1);
    clock_t t2, t1 = clock();
    previos_pose<< (double)x0(0,0), (double)x0(1,0), (double)x0(2,0);
    int stuck_point = 0;
    double previous_error = 0;
    early_stop = false;
    while(error > maximum_acceptable_error && mpciter < sim_time/delta_t){
        t2 = clock();
        if(force_terminate){
              still_running = false;
              need_intermediate_goal = false;
              std::cout<< "[Solver]: Solver has been interrupted..." <<  std::endl;
              return;
        }
        if(double(t2 - t1)/CLOCKS_PER_SEC > simulator_min_time_){
          args["p"] = vertcat(x0, xs);
          args["x0"] = vertcat(reshape(X0.T(), n_states*(prediction_horizon+1), 1), reshape(U0.T(), n_controls*prediction_horizon, 1));
          vector<Eigen::VectorXd> obs_map_;
          double previous_dis = 20;
          for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
            SX st = X(Slice(0, X.size1()), Slice(trj_i));
            Eigen::Vector3d pos((double)X0(trj_i,0), (double)X0(trj_i,1), (double)X0(trj_i,2));
            Eigen::Vector3d obs;
            double obs_dis;
            Eigen::VectorXd obs_pose(4);
            edt_env_->get_close_obstacle(pos, obs, obs_dis);
            if(std::abs(obs_dis-previous_dis)< obs_min_allowed_length){
              // cout<< "Obs at the same pose" << obs.transpose() << endl;
            }else{
              obs_pose.head(3) = obs;
              obs_pose[3] = obs_dis;
              obs_map_.push_back(obs_pose);
              previous_dis = obs_dis;
            }
            
          }
          std::sort(std::begin(obs_map_), std::end(obs_map_), [](Eigen::VectorXd a
              , Eigen::VectorXd b) {return a[3] > b[3]; });
          
          if(obs_map_.size() <= obs_max_allowed_length){
            obs_length = obs_map_.size();
          }else{
            obs_length = obs_max_allowed_length;
          }
          DM lbg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
          DM ubg = DM(1, n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length);
          lbg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
          ubg(Slice(0), Slice(0, n_states*(prediction_horizon+1))) = 0;
          lbg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = -inf;
          ubg(Slice(0), Slice(n_states*(prediction_horizon+1), n_states*(prediction_horizon+1) + (prediction_horizon+1)*obs_length)) = 0;
          args["lbg"] = lbg; 
          args["ubg"] = ubg;
          SX obs_g = SX::sym("obs_g",(prediction_horizon+1)*obs_length);
          int obs_cout=0;
          for(int trj_i=0; trj_i< (prediction_horizon+1); trj_i++){
            SX st = X(Slice(0, X.size1()), Slice(trj_i));
            for(int i=0; i< obs_length; i++){
              Eigen::VectorXd obs = obs_map_[i];
              obs_g(obs_cout) = -sqrt(pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + (avoidance_distance);
              obs_cout++;
            }            
          }
          
          std::vector<Eigen::Vector3d> vis_obs;
          for(auto obs : obs_map_){
            vis_obs.push_back(obs.head(3));
          }
          visualization_->drawPath(vis_obs, 0.5, Eigen::Vector4d(0.0, 0.2 ,0.9, 1), 345);
          SX final_g = vertcat(g, obs_g);
          nlp_prob["g"] = final_g;
          Function solver = nlpsol("nlpsol", "ipopt", nlp_prob, opts);
          res = solver(args);
          std::string retstat = solver.stats().at("return_status");
           double err = std::abs((error - previous_error));
          if(retstat == solver_state_error){
            stuck_point++;
          }else{
            stuck_point--;
          }
          if(stuck_point<0){
            stuck_point = 0;
          }
          previous_error = error;

          DM u = reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T();
          vector<double> u11(reshape(res["x"](Slice(n_states*(prediction_horizon+1), res.at("x").size1())).T(), n_controls, prediction_horizon).T());
          vector<double> xx11(res["x"](Slice(0, (n_states*(prediction_horizon+1)))).T());
          xx1.push_back(xx11);
          std::vector<Eigen::Vector3d> vis_horizon;

          for(int kl=0; kl< int(xx11.size()/n_states); kl+=n_states){
            Eigen::Vector3d poss(xx11[kl], xx11[kl+1], xx11[kl+2]);
            vis_horizon.push_back(poss);
          }
          
          visualization_->drawPath(vis_horizon, 0.2, Eigen::Vector4d(0.9, 1.0 ,0.5, 1), 0);
          DM current_control = u(Slice(0), Slice(0, u.size2())).T();
          DM current_pose = x0;

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

          x0(0,0) = current_projected_pose.pose.pose.position.x =  kf_nmpc->state()[0];
          x0(1,0) = current_projected_pose.pose.pose.position.y = kf_nmpc->state()[1];
          x0(2,0) = current_projected_pose.pose.pose.position.z = kf_nmpc->state()[2];

          current_projected_pose.twist.twist.linear.x = kf->state()[0];
          current_projected_pose.twist.twist.linear.y = kf->state()[1];
          current_projected_pose.twist.twist.linear.z = kf->state()[2];

          if(!init_previous_pose){
            init_previous_pose =  true;
          }
          previous_projected_pose = current_projected_pose;
          pos_current_pos_pub.publish(current_projected_pose);
          mpciter = mpciter + 1;
          
          u_cl.push_back(u11);
          t.push_back(t0);
          tuple<double, DM, DM> shiftted_val = shift(delta_t, t0, x0, u, f); 
          x0 = get<1>(shiftted_val);
          u = get<2>(shiftted_val);
          t0 = get<0>(shiftted_val);

          error = (double)(norm_2(x0-xs));
        
          X0 = reshape(res["x"](Slice(0, n_states*(prediction_horizon+1))).T(), n_states, prediction_horizon+1).T();
          vector<double> xxxo(x0);
          xx0.push_back(xxxo);

          SX x0_rest = X0(Slice(1, X0.size1()), Slice(0, X0.size2()));
          SX x0_last = X0(Slice(X0.size1()-1, X0.size1()), Slice(0, X0.size2()));
          X0 = vertcat(x0_rest, x0_last);

          if((err < 0.0000001) && (retstat == solver_state_success)){
            early_stop = true;
            still_running = false;
            return;
          }
          t1 = clock();
        }else{
          usleep(5000);
        }
    }

    still_running = false;
    need_intermediate_goal = false;
    cout<< "[Solver]: Finish mpc solver...."<< endl;
  }

  void TrajectoryRegulator::mpc_solver_with_collocation(){

  }

  void TrajectoryRegulator::odom_callback_ref(const nav_msgs::OdometryConstPtr& msg){
    reference_odom = *msg;
    reference_pose << reference_odom.pose.pose.position.x, reference_odom.pose.pose.position.y, reference_odom.pose.pose.position.z;
    xs = DM({reference_pose(0), reference_pose(1), reference_pose(2)});
  }

  void TrajectoryRegulator::init(ros::NodeHandle& nh){
      node_ = nh;
      /* ---------- param ---------- */
      node_.param("mpc_opt/simulator_min_time", simulator_min_time_, 0.05);
      odometry_sub_ref_ = node_.subscribe<nav_msgs::Odometry>("/odom_ref", 50, &TrajectoryRegulator::odom_callback_ref, this);
      NonLinearMPCOpt::init(nh);
      solver_init();
  }

}  // namespace hagen_planner
