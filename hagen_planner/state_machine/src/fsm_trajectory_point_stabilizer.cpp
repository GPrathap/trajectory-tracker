
#include <state_machine/fsm_trajectory_point_stabilizer.h>

namespace hagen_planner
{
void FSM_Trajectory_Point_Stabilizer::init(ros::NodeHandle& nh)
{

  /* ---------- fsm param ---------- */
  nh.param("fsm/sampling_rate", sampling_rate, 30);
  nh.param("fsm/avoidance_distance", avoidance_distance, 0.3);
  nh.param("fsm/avoidance_distance_max", avoidance_distance_max, 0.3);
  nh.param("fsm/intermediate_goal_max_dis", intermediate_goal_max_dis, 3.0);
  nh.param("fsm/cone_inner_radius", cone_inner_radius, 2.0);
  nh.param("fsm/cone_outer_radius", cone_outer_radius, 1.0);

  current_wp_ = 0;
  exec_state_ = EXEC_STATE::WAIT_GOAL;
  have_goal_ = false;

  sdf_map_.reset(new EDTOctoMap);
  sdf_map_->init(nh);

  edt_env_.reset(new EDTEnvironment);
  edt_env_->setMap(sdf_map_);

  trajectroy_regulator.reset(new TrajectoryRegulator);
  trajectroy_regulator->visualization_.reset(new PlanningVisualization(nh));
  trajectroy_regulator->passer.reset(new ParamPasser(nh));
  trajectroy_regulator->planner_saving.reset(new PlanningSaving(nh));
  trajectroy_regulator->init(nh);
  trajectroy_regulator->setEnvironment(edt_env_);

  visualization_.reset(new PlanningVisualization(nh));

  traj_cmd = new boost::circular_buffer<Eigen::Vector3d>(10000);
  traj_real = new boost::circular_buffer<Eigen::Vector3d>(10000);

  
  safety_timer_ = node_.createTimer(ros::Duration(0.01), &FSM_Trajectory_Point_Stabilizer::safetyCallback, this);
  waypoint_sub_ = node_.subscribe("/planner/waypoints", 1, &FSM_Trajectory_Point_Stabilizer::waypointCallback, this);
  stat_moving = node_.advertise<std_msgs::Empty>("/planning/start_moving", 10);
  stop_moving = node_.advertise<std_msgs::Empty>("/planning/stop_moving", 10);
  odometry_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 50, &FSM_Trajectory_Point_Stabilizer::odomCallback, this);
  vehicle_current_pose_sub_ = node_.subscribe<nav_msgs::Odometry>("/planning/current_state", 5, &FSM_Trajectory_Point_Stabilizer::currentPoseCallback, this);
  stop_execution_sub_ = node_.subscribe<std_msgs::Empty>("/planning/stop_execution", 2, &FSM_Trajectory_Point_Stabilizer::stopExecutionCallback, this);
  continue_execution_sub_ = node_.subscribe<std_msgs::Empty>("/planning/continue_execution", 2, &FSM_Trajectory_Point_Stabilizer::continueExecutionCallback, this);

  pos_cmd_pub = node_.advertise<hagen_msgs::PoseCommand>("/planning/pos_cmd", 50);

  Eigen::MatrixXd k_A(1, 1); // System dynamics matrix
  Eigen::MatrixXd k_C(1, 1); // Output matrix
  Eigen::MatrixXd k_Q(1, 1); // Process noise covariance
  Eigen::MatrixXd k_R(1, 1); // Measurement noise covariance
  Eigen::MatrixXd k_P(1, 1); // Estimate error covariance
  k_A << 1;
  k_C << 1;
  // Reasonable covariance matrices
  bool passed = trajectroy_regulator->passer->passing_matrix("covariance_matrix_for_yaw_angle_q", k_Q);
  if(!passed){
    double q_cov = 1;
    k_Q << q_cov;
  }
  passed = trajectroy_regulator->passer->passing_matrix("covariance_matrix_for_yaw_angle_r", k_R);
  if(!passed){
    double r_cov = 5000;
     k_R << r_cov;
  }

  k_P << 1;
  std::cout << "A: \n" << k_A << std::endl;
  std::cout << "C: \n" << k_C << std::endl;
  std::cout << "Q: \n" << k_Q << std::endl;
  std::cout << "R: \n" << k_R << std::endl;
  std::cout << "P: \n" << k_P << std::endl;
  kf_yaw = new KalmanFilter(0, k_A, k_C, k_Q, k_R, k_P);

  Eigen::VectorXd inter_stop_pose(6);
  intermediate_stop_pose = inter_stop_pose;
  fsm_thread = execFSMThread();
  solver_thread = startSolverThread();
  cmd_thread = execCMDThread();
}

void FSM_Trajectory_Point_Stabilizer::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
    return;
  std::unique_lock<std::mutex> guard(mutex_odom);  
  odom = *msg;
  mutex_odom.unlock();
  traj_real->push_back(Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
  if(stop_pose_init == false){
    stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
    stop_pose_init = true;
    stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
    Eigen::VectorXd inter_stop_pose(6);
    inter_stop_pose << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, 0, 0, 0;
    intermediate_stop_pose = inter_stop_pose;
  }
}

void FSM_Trajectory_Point_Stabilizer::currentPoseCallback(const nav_msgs::OdometryConstPtr& msg){
  std::unique_lock<std::mutex> guard(mutex_current_pose);
  current_pose = *msg;
  guard.unlock();
}

void FSM_Trajectory_Point_Stabilizer::stopExecutionCallback(const std_msgs::Empty msg){
  if(!stop_execution){
    stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
    stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
    std_msgs::Empty emt;
    stop_moving.publish(emt);
    granted_execution = false;
    trajectroy_regulator->force_terminate = true;
    have_goal_ = false;
    stop_execution = true;
  }
}

void FSM_Trajectory_Point_Stabilizer::continueExecutionCallback(const std_msgs::Empty msg){
  if(stop_execution){
    trigger_ = true;
    waypoints_list.push_front(end_pt_);
    stop_execution = false;
    have_goal_ = false;
    cout << "Triggered! continue execution... ! " << endl;
  }
}

void FSM_Trajectory_Point_Stabilizer::waypointCallback(const nav_msgs::PathConstPtr& msg)
{
  std_msgs::Empty emt;
  stop_moving.publish(emt);
  granted_execution = false;
  trajectroy_regulator->force_terminate = true;

  if (msg->poses.size() < 0.0){
    cout<< "empty waypoints are detected.." << endl;
    return;
  }
    
  cout << "Triggered!" << endl;
  std::unique_lock<std::mutex> guard(mutex_odom);
  stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
  intermediate_stop_pose << odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z, 0, 0, 0;
  stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
  guard.unlock();

  trigger_ = true;
  
  cout<< "Intermediate goals poses:" << endl;
  double mininum_height = edt_env_->getMapCurrentRange()[0](2);

  std::unique_lock<std::mutex> guard_goal(mutex_goal_poses);
  waypoints_list.clear();
  if(waypoints_list.size()>0){
    cout<< "No waypoints" << endl;
  }
  for(int i=0; i<(int)msg->poses.size(); i++){
    Eigen::Vector3d wp(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
    cout<< wp.transpose() << endl;
    if(msg->poses[i].pose.position.z< mininum_height){
      cout<< "z shuold be higher than "<< mininum_height << endl;
      waypoints_list.clear();
      return;
    }
    waypoints_list.push_back(wp);
  }

  if(waypoints_list.size()<1){
    cout<< "At least one way point is need, please tray again...!" << endl;
    waypoints_list.clear();
    return;
  }
  guard_goal.unlock();

  while(trajectroy_regulator->still_running){
    cout<< "Waitng till stopping the solver...!" << endl;
    granted_execution = false;
    trajectroy_regulator->force_terminate = true;
    usleep(5000);
  }

  have_goal_ = false;
}

void FSM_Trajectory_Point_Stabilizer::changeExecState(EXEC_STATE new_state, string pos_call)
{
  string state_str[2] = {"WAIT_GOAL", "EXEC_TRAJ" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void FSM_Trajectory_Point_Stabilizer::printExecState(){
  string state_str[2] = {"WAIT_GOAL", "EXEC_TRAJ"};
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

double FSM_Trajectory_Point_Stabilizer::getYawFromQuat(const geometry_msgs::Quaternion &data){
    tf::Quaternion quat(data.x, data.y, data.z, data.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void FSM_Trajectory_Point_Stabilizer::solverThread() {
  while(is_allowed_for_execution){
    std::cout<< "[FSM]: Waiting to start execution of solver..." << std::endl;
    std::unique_lock<std::mutex> lk(lock_on_solver);
    condition_on_solver.wait(lk, [&]{return granted_execution;});
    std::cout<< "[FSM]: Starting the solver thread..." << std::endl;
    trajectroy_regulator->still_running = true;
    trees.clear();
    trees_real.clear();

    nav_msgs::Odometry current_vehicle_pose;
    std::unique_lock<std::mutex> guard(mutex_odom);
    current_vehicle_pose = odom;
    guard.unlock();

    start_pt_(0) = current_vehicle_pose.pose.pose.position.x;
    start_pt_(1) = current_vehicle_pose.pose.pose.position.y;
    start_pt_(2) = current_vehicle_pose.pose.pose.position.z;
   
    trajectroy_regulator->force_terminate = false;
    if(has_intermeditate_goal){
      trajectroy_regulator->xs = DM({inter_end_pt_(0), inter_end_pt_(1), inter_end_pt_(2)});
      has_intermeditate_goal =  false;
      had_intermeditate_goal = true;
    }else{
      trajectroy_regulator->xs = DM({end_pt_(0), end_pt_(1), end_pt_(2)});
    }
   
    trajectroy_regulator->x0 = DM(3,1);
    trajectroy_regulator->x0(0,0) = start_pt_(0);
    trajectroy_regulator->x0(1,0) = start_pt_(1);
    trajectroy_regulator->x0(2,0) = start_pt_(2);
   

    trajectroy_regulator->u0 = DM(3,1);
    trajectroy_regulator->u0(0,0) = current_vehicle_pose.twist.twist.linear.x;
    trajectroy_regulator->u0(1,0) = current_vehicle_pose.twist.twist.linear.y;
    trajectroy_regulator->u0(2,0) = current_vehicle_pose.twist.twist.linear.z;


    trajectroy_regulator->mpc_solver();
    
    guard.lock();
    current_vehicle_pose = odom;
    guard.unlock();
    
    if(trajectroy_regulator->force_terminate){
        had_intermeditate_goal =  false;
        has_intermeditate_goal = false;
        have_goal_ = false;
        avoidance_distance_intermediate = 0;
        std::cout<< "[FSM]: Solver has been interrupted..." << std::endl;
    }else if(trajectroy_regulator->early_stop){
      had_intermeditate_goal =  false;
      has_intermeditate_goal = false;
      have_goal_ = true;
      std::cout<< "[FSM]: Solver has been continued....." << std::endl;
    }else if(trajectroy_regulator->need_intermediate_goal){
      Eigen::Vector3d current_pt(current_vehicle_pose.pose.pose.position.x, current_vehicle_pose.pose.pose.position.y, current_vehicle_pose.pose.pose.position.z);
      Eigen::Vector3d goal;
      Eigen::Vector3d projected_pose;
      bool status = false;
      while(1){
        status = edt_env_->get_projected_point(current_pt, end_pt_,
               cone_inner_radius+avoidance_distance_intermediate,
               cone_outer_radius+avoidance_distance_intermediate, avoidance_distance,
                avoidance_distance_max, projected_pose);

        Eigen::MatrixXd poses;
        edt_env_->get_cone_points(poses);
        visualization_->drawConePoints(poses, 0.3, Eigen::Vector4d(1, 1, 0, 1.0), 567);
        if (!status)
        {
          avoidance_distance_intermediate += 0.4;
          if(avoidance_distance_intermediate>intermediate_goal_max_dis){
            avoidance_distance_intermediate = intermediate_goal_max_dis;
            break;
          }
        }else{
          break;
        }
      }
      if (status)
      {
        cout << "[FSM]: change goal, replan.: " << cone_inner_radius + avoidance_distance_intermediate << endl;
        inter_end_pt_ = projected_pose;
        has_intermeditate_goal = true;
        visualization_->drawGoal(inter_end_pt_, 0.3, Eigen::Vector4d(1, 1, 0, 1.0), 67);
      }else{
        had_intermeditate_goal =  false;
        has_intermeditate_goal = false;
        std::cout<< "[FSM]: Cant not find intermediate goal,...(; solver is finished..." << std::endl;
      }
    }else if(had_intermeditate_goal){
      std::cout<< "[FSM]: Solver is intermediate goal to final goal..." << std::endl;
      had_intermeditate_goal = false;
    }
    else{
      std::cout<< "[FSM]: Solver is finished..." << std::endl;
      have_goal_ = false;
      avoidance_distance_intermediate = 0;
    }
   
    stop_pose <<  current_vehicle_pose.pose.pose.position.x,  current_vehicle_pose.pose.pose.position.y,  current_vehicle_pose.pose.pose.position.z;
    stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
    intermediate_stop_pose << current_vehicle_pose.pose.pose.position.x,  current_vehicle_pose.pose.pose.position.y,  current_vehicle_pose.pose.pose.position.z
              , current_vehicle_pose.twist.twist.linear.x, current_vehicle_pose.twist.twist.linear.y, current_vehicle_pose.twist.twist.linear.z;
    granted_execution = false;
    lk.unlock();
    condition_on_solver.notify_all();
  }
  return;
}

std::thread FSM_Trajectory_Point_Stabilizer::startSolverThread() {
    return std::thread([&] { solverThread(); });
}

void FSM_Trajectory_Point_Stabilizer::fsmExecutor() {
  int fsm_num = 0;
  clock_t t2, t1 = clock();
  while(is_allowed_for_execution){
    t2 = clock();
    if((t2-t1) > 1.0){
      fsm_num++;
      if (fsm_num == 100)
      {
        printExecState();
        fsm_num = 0;
      }
      if (!edt_env_->odomValid()){
          cout << "FSM_Trajectory_Point_Stabilizer: no odom." << endl;
      }
      if (!edt_env_->mapValid()){
          cout << "FSM_Trajectory_Point_Stabilizer : no map." << endl;
      } else{ 
        switch (exec_state_)
        {
          case WAIT_GOAL:
          {
            std_msgs::Empty emt;
            stop_moving.publish(emt);
            std::unique_lock<std::mutex> guard(mutex_goal_poses);
            if(have_goal_){
              changeExecState(EXEC_TRAJ, "FSM");
            }else if(waypoints_list.size()>0 && !stop_execution){
              end_pt_ = waypoints_list.front();
              waypoints_list.pop_front();
              have_goal_ = true;
              changeExecState(EXEC_TRAJ, "FSM");
            }
            guard.unlock();
            break;
          }
          case EXEC_TRAJ:
          {
            if(!have_goal_){
              changeExecState(WAIT_GOAL, "FSM");
            }
            else if(!trajectroy_regulator->still_running && !granted_execution){
              std::lock_guard<std::mutex> lk(lock_on_solver);
              granted_execution = true;
              condition_on_solver.notify_one();
              std_msgs::Empty emt;
              stat_moving.publish(emt);
            }
            retry_generate_cout++;
            break;
          }
        }
      }
      t1 =  clock();
    }else{
      usleep(10000);
    }
  }
}

std::thread FSM_Trajectory_Point_Stabilizer::execFSMThread() {
    return std::thread([&] { fsmExecutor(); });
}

void FSM_Trajectory_Point_Stabilizer::cmdExecutor() {
  clock_t t2, t1 = clock();
  while(is_allowed_for_execution){
    t2 = clock();
    if((t2-t1) > 0.08){
      if (!stop_pose_init){
      }else{
        Eigen::Vector3d pos, vel, acc;
        ros::Time time_now = ros::Time::now();
        int traj_id = 1;
        nav_msgs::Odometry current_state;
        std::unique_lock<std::mutex> guard_pose(mutex_current_pose);
        current_state = current_pose;
        guard_pose.unlock();
        Eigen::VectorXd state_vec(4);
        if(!init_kf_yaw){
            Eigen::VectorXd k_x0(1);
            k_x0 << stop_yaw_angle;
            kf_yaw->init(0, k_x0);
            init_kf_yaw =  true;
        }
        if(trajectroy_regulator->still_running && !trajectroy_regulator->force_terminate){
          pos(0) = current_state.pose.pose.position.x;
          pos(1) = current_state.pose.pose.position.y;
          pos(2) = current_state.pose.pose.position.z;
          current_yaw = current_state.pose.pose.orientation.x;
          state_vec << current_state.twist.twist.linear.x, current_state.twist.twist.linear.y
                    , current_state.twist.twist.linear.z, current_state.twist.twist.angular.z;
          vel(0) = (double)(state_vec(0)*cos(current_yaw) - state_vec(1)*sin(current_yaw));
          vel(1) = (double)(state_vec(0)*sin(current_yaw) + state_vec(1)*cos(current_yaw));
          vel(2) = current_state.twist.twist.linear.z;
          if(std::abs(vel(0)) < 0.00005 || std::abs(vel(1)) < 0.00005){
            current_yaw =  kf_yaw->state()[0];
          }else{

            Eigen::Vector3d normalized_vector = (vel).normalized();
            double curr_yaw = std::atan2(normalized_vector[1], normalized_vector[0]);
            if(normalized_vector[1]<0){
              curr_yaw += 2*M_PI;
            }

            Eigen::VectorXd k_y(1);
            k_y << curr_yaw;
            kf_yaw->update(k_y);
            previous_yaw = current_yaw;
            current_yaw =  kf_yaw->state()[0];
          }
        } else if(waypoints_list.size()>0){
          pos = intermediate_stop_pose.head(3);
          vel = intermediate_stop_pose.tail(3);
          Eigen::VectorXd k_y(1);
          k_y <<  stop_yaw_angle;
          kf_yaw->update(k_y); 
          previous_yaw = current_yaw;
          current_yaw =  kf_yaw->state()[0];
        }else {
          pos = stop_pose;
          Eigen::VectorXd k_y(1);
          k_y <<  stop_yaw_angle;
          kf_yaw->update(k_y); 
          previous_yaw = current_yaw;
          current_yaw =  kf_yaw->state()[0];
          vel.setZero();
        }
        acc.setZero();

        cmd.yaw = current_yaw;
        cmd.header.stamp = time_now;
        cmd.header.frame_id = "map";
        cmd.trajectory_flag = hagen_msgs::PoseCommand::TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id;

        cmd.position.x = pos(0);
        cmd.position.y = pos(1);
        cmd.position.z = pos(2);

        cmd.state_vector.x = state_vec[0];
        cmd.state_vector.y = state_vec[1];
        cmd.state_vector.z = state_vec[2];
        cmd.yaw_dot = state_vec[3];

        cmd.velocity.x = vel(0);
        cmd.velocity.y = vel(1);
        cmd.velocity.z = vel(2);

        cmd.acceleration.x = acc(0);
        cmd.acceleration.y = acc(1);
        cmd.acceleration.z = acc(2);
        if(cmd.position.z > edt_env_->getMapCurrentRange()[0](2)){
          pos_cmd_pub.publish(cmd);
        }
        visualization_->drawState(pos, vel, 50, Eigen::Vector4d(0.6, 1, 0.8, 1));
        traj_cmd->push_back(pos);
      }
      t1 =  clock();
    }else{
      usleep(5000);
    }
  }
}


std::thread FSM_Trajectory_Point_Stabilizer::execCMDThread() {
    return std::thread([&] { cmdExecutor(); });
}


void FSM_Trajectory_Point_Stabilizer::safetyCallback(const ros::TimerEvent& e)
{
  visualization_->displayTrajWithColor(*traj_cmd, 0.03, Eigen::Vector4d(1, 1, 0, 1), 21);
  visualization_->displayTrajWithColor(*traj_real, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964, 1), 20);
}

}  // namespace hagen_planner


namespace backward
{
  backward::SignalHandling sh;
}

using namespace hagen_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hagen_planner_group_node");
  ros::AsyncSpinner spinner(1);
  // ros::MultiThreadedSpinner spinner(2);
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  FSM_Trajectory_Point_Stabilizer fsm;
  fsm.init(nh);
  // spinner.spin();
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
