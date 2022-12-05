
#include <state_machine/fsm_linear_trajectory_tracker.h>

namespace hagen_planner
{
void FSMLinearTrajectoryTracker::init(ros::NodeHandle& nh)
{

  /* ---------- init global param---------- */
  nh.param("bspline/limit_vel", NonUniformBspline::limit_vel_, -1.0);
  nh.param("bspline/limit_acc", NonUniformBspline::limit_acc_, -1.0);
  nh.param("bspline/limit_ratio", NonUniformBspline::limit_ratio_, -1.0);
  /* ---------- fsm param ---------- */
  nh.param("fsm/sampling_rate", sampling_rate, 30);
  nh.param("fsm/avoidance_distance", avoidance_distance, 0.3);
  nh.param<std::string>("fsm/waypoints_file", waypoints_file, "waypoints.csv");
  nh.param<std::string>("fsm/bspline_file", bspline_file, "bspline_s.csv");

  current_wp_ = 0;
  exec_state_ = EXEC_STATE::WAIT_GOAL;
  have_trajector_ = false;

  /* ---------- init edt environment ---------- */
  sdf_map_.reset(new EDTOctoMap);
  sdf_map_->init(nh);

  edt_env_.reset(new EDTEnvironment);
  edt_env_->setMap(sdf_map_);

  bspline_utils_.reset(new BSplineUtils);
  bspline_utils_->setParam(nh);
  bspline_utils_->visualization_.reset(new PlanningVisualization(nh));

  rebound_optimizer.reset(new hagen_planner::BsplineOptimizer);
  rebound_optimizer->setParam(nh);
  rebound_optimizer->setEnvironment(edt_env_);
  rebound_optimizer->jsp_planner.reset(new JPSPlanner(true));
  rebound_optimizer->jsp_planner->setEnvironment(edt_env_);

  rebound_optimizer->a_star_.reset(new AStar);
  rebound_optimizer->a_star_->initGridMap(edt_env_, Eigen::Vector3i(100, 100, 100));
  rebound_optimizer->visualization_.reset(new PlanningVisualization(nh));
  trajectroy_tracker.reset(new LinearMPCTrajectoryTracker);
  trajectroy_tracker->visualization_.reset(new PlanningVisualization(nh));
  trajectroy_tracker->passer.reset(new ParamPasser(nh));
  trajectroy_tracker->planner_saving.reset(new PlanningSaving(nh));
  trajectroy_tracker->setEnvironment(edt_env_);
  trajectroy_tracker->init(nh);
  trajectroy_tracker->setTrajectoryGenerator(bspline_utils_);
  trajectroy_tracker->setBoundaryChecker(rebound_optimizer);

  visualization_.reset(new PlanningVisualization(nh));

  traj_cmd = new boost::circular_buffer<Eigen::Vector3d>(10000);
  traj_real = new boost::circular_buffer<Eigen::Vector3d>(10000);

  safety_timer_ = node_.createTimer(ros::Duration(5.0), &FSMLinearTrajectoryTracker::safetyCallback, this);
  waypoint_sub_ = node_.subscribe("/planner/waypoints", 1, &FSMLinearTrajectoryTracker::waypointCallback, this);

  replan_pub_ = node_.advertise<std_msgs::Empty>("/planning/replan", 10);
  wait_for_goal = node_.advertise<std_msgs::Empty>("/planning/wait_for_goal", 10);
  stat_moving = node_.advertise<std_msgs::Empty>("/planning/start_moving", 10);
  stop_moving = node_.advertise<std_msgs::Empty>("/planning/stop_moving", 10);
  bspline_pub_ = node_.advertise<state_machine::Bspline>("/planning/bspline", 10);
  odometry_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 50, &FSMLinearTrajectoryTracker::odomCallback, this);
  obstacle_poses_sub_ = node_.subscribe<visualization_msgs::MarkerArray>("/planner/detected_obs", 50, &FSMLinearTrajectoryTracker::detectedObs, this);
  lidar_poses_2d_sub_ = node_.subscribe<sensor_msgs::LaserScan>("/scan", 50, &FSMLinearTrajectoryTracker::laserScan, this);
  vehicle_current_pose_sub_ = node_.subscribe<nav_msgs::Odometry>("/planning/current_state", 5, &FSMLinearTrajectoryTracker::currentPoseCallback, this);

  pos_cmd_pub = node_.advertise<hagen_msgs::PoseCommand>("/planning/pos_cmd", 50);

  Eigen::MatrixXd k_A(1, 1); // System dynamics matrix
  Eigen::MatrixXd k_C(1, 1); // Output matrix
  Eigen::MatrixXd k_Q(1, 1); // Process noise covariance
  Eigen::MatrixXd k_R(1, 1); // Measurement noise covariance
  Eigen::MatrixXd k_P(1, 1); // Estimate error covariance
  k_A << 1;
  k_C << 1;
  // Reasonable covariance matrices
  bool passed = trajectroy_tracker->passer->passing_matrix("covariance_matrix_for_yaw_angle_q", k_Q);
  if(!passed){
    double q_cov = 1;
    k_Q << q_cov;
  }
  passed = trajectroy_tracker->passer->passing_matrix("covariance_matrix_for_yaw_angle_r", k_R);
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
    //  // Construct the filter

  kf_yaw = new KalmanFilter(0, k_A, k_C, k_Q, k_R, k_P);

  Eigen::VectorXd inter_stop_pose(6);
  fsm_thread = execFSMThread();
  solver_thread = startSolverThread();
  solver_global_thread = startGlobalTrajectoryThread();
  cmd_thread = execCMDThread();

}

void FSMLinearTrajectoryTracker::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
    return;
  odom = *msg;
  traj_real->push_back(Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
  if(stop_pose_init == false){
    stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
    stop_pose_init = true;
    stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
  }
}

void FSMLinearTrajectoryTracker::detectedObs(const visualization_msgs::MarkerArray::ConstPtr& msg){
  
}

void FSMLinearTrajectoryTracker::laserScan(const sensor_msgs::LaserScan::ConstPtr& scan_in){
  projector_.projectLaser(*scan_in, laser_cloud);
}

void FSMLinearTrajectoryTracker::currentPoseCallback(const nav_msgs::OdometryConstPtr& msg){
  current_pose = *msg;
}

void FSMLinearTrajectoryTracker::stopExecutionCallback(const std_msgs::Empty msg){
}

void FSMLinearTrajectoryTracker::continueExecutionCallback(const std_msgs::Empty msg){
}


void FSMLinearTrajectoryTracker::waypointCallback(const nav_msgs::PathConstPtr& msg)
{
  std_msgs::Empty emt;
  stop_moving.publish(emt);
  granted_execution = false;
  trajectroy_tracker->force_terminate = true;

  waypoints_list.clear();
  if (msg->poses.size() < 0.0){
    cout<< "empty waypoints are detected.." << endl;
    return;
  }
    
  cout << "Triggered!" << endl;
  stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
  outfile_waypoints <<stop_pose[0] << "," << stop_pose[1] << ","<< stop_pose[2];
  stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
  trigger_ = true;

  waypoints_list.push_back(stop_pose);
  cout<< "Intermediate goals poses:" << endl;
  double mininum_height = edt_env_->getMapCurrentRange()[0](2);
  for(int i=0; i<(int)msg->poses.size(); i++){
    Eigen::Vector3d wp(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
    cout<< wp.transpose() << endl;
    if(msg->poses[i].pose.position.z< mininum_height){
      cout<< "z shuold be higher than "<< mininum_height << endl;
      waypoints_list.clear();
      return;
    }
    outfile_waypoints<< "\n" << wp[0] << "," << wp[1] << ","<< wp[2];
    waypoints_list.push_back(wp);
  }
  if(waypoints_list.size()<4){
    cout<< "At least three way point is need, please tray again...!" << endl;
    waypoints_list.clear();
    return;
  }

  while(trajectroy_tracker->still_running){
    cout<< "Waiting till stopping the solver...!" << endl;
    trajectroy_tracker->force_terminate = true;
    granted_execution = false;
    usleep(5000);
  }

  end_pt_ = waypoints_list.at(waypoints_list.size()-1);

  have_trajector_ = bspline_utils_->generateTrajectory(waypoints_list, 1);
  outfile_waypoints<<"\n";

  if(!have_trajector_){
    cout<< "Trajectory can not be generated, please regenerate again..." << endl;
  }else{
    double tm, tmp;
    auto trajectory = bspline_utils_->traj_pos_;
    auto traj_vel_ = trajectory.getDerivative();
    auto traj_acc_ = traj_vel_.getDerivative();
    trajectory.getTimeSpan(tm, tmp);
    std::cout<< "time intervel of the estimaton: " << tmp-tm << std::endl;
    for (double t = tm; t <= tmp; t += 0.1)
    {
      Eigen::Vector3d pt = trajectory.evaluateDeBoor(t);
      outfile_bspline << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
    traj_vel_.getTimeSpan(tm, tmp);
    std::cout<< "time intervel of the estimaton: " << tmp-tm << std::endl;
    for (double t = tm; t <= tmp; t += 0.1)
    {
      Eigen::Vector3d pt = traj_vel_.evaluateDeBoor(t);
      outfile_bspline << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }

    traj_acc_.getTimeSpan(tm, tmp);
    std::cout<< "time intervel of the estimaton: " << tmp-tm << std::endl;
    for (double t = tm; t <= tmp; t += 0.1)
    {
      Eigen::Vector3d pt = traj_acc_.evaluateDeBoor(t);
      outfile_bspline << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
  }
}


void FSMLinearTrajectoryTracker::changeExecState(EXEC_STATE new_state, string pos_call)
{
  string state_str[2] = {"WAIT_GOAL", "EXEC_TRAJ" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void FSMLinearTrajectoryTracker::printExecState(){
  string state_str[2] = {"WAIT_GOAL", "EXEC_TRAJ"};
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

double FSMLinearTrajectoryTracker::getYawFromQuat(const geometry_msgs::Quaternion &data){
    tf::Quaternion quat(data.x, data.y, data.z, data.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void FSMLinearTrajectoryTracker::solverThread() {
  while(is_allowed_for_execution){
    std::cout<< "Waiting to start execution of solver..." << std::endl;
    std::unique_lock<std::mutex> lk(lock_on_solver);
    condition_on_solver.wait(lk, [&]{return granted_execution;});
    std::cout<< "Starting the solver thread..." << std::endl;
    trees.clear();
    trees_real.clear();
    stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
    start_pt_(0) = odom.pose.pose.position.x;
    start_pt_(1) = odom.pose.pose.position.y;
    start_pt_(2) = odom.pose.pose.position.z;
    trajectroy_tracker->force_terminate = false;
    trajectroy_tracker->xs.resize(4);
     
    trajectroy_tracker->x0 = Eigen::MatrixXd::Zero(3, 1);
    trajectroy_tracker->u0.resize(4);
    trajectroy_tracker->xs << end_pt_[0], end_pt_[1], end_pt_[2], 0;
    trajectroy_tracker->x0 << start_pt_[0], start_pt_[1], start_pt_[2];
    trajectroy_tracker->u0 << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, odom.twist.twist.angular.z;
    granted_execution_global = true;
    trajectroy_tracker->still_running = true;
    condition_on_global_solver.notify_all();
    trajectroy_tracker->mpc_solver();

    
    std::cout<< "Solver is finished..." << std::endl;
    have_trajector_ = false;
    granted_execution = false;
  
    stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
    lk.unlock();
    condition_on_solver.notify_all();

  }
  return;
}

std::thread FSMLinearTrajectoryTracker::startSolverThread() {
    return std::thread([&] { solverThread(); });
}

void FSMLinearTrajectoryTracker::GlobalTrajectoryThread() {
  while(is_allowed_for_execution){
    std::cout<< "Waiting to start execution of global trajectory..." << std::endl;
    std::unique_lock<std::mutex> lgk(lock_on_global_solver);
    condition_on_global_solver.wait(lgk, [&]{return granted_execution_global;});
    std::cout<< "Starting the global trajectory thread..." << std::endl;
    granted_execution_global = false;
    trajectroy_tracker->global_solver();
    std::cout<< "Global trajectory is finished..." << std::endl;
    lgk.unlock();
    condition_on_global_solver.notify_all();
  }
  return;
}

std::thread FSMLinearTrajectoryTracker::startGlobalTrajectoryThread() {
    return std::thread([&] { GlobalTrajectoryThread(); });
}

void FSMLinearTrajectoryTracker::fsmExecutor(){
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
            // cout << "FSMLinearTrajectoryTracker: no odom." << endl;
        }
        if (!edt_env_->mapValid()){
            // cout << "FSMLinearTrajectoryTracker : no map." << endl;
        }else{
          switch (exec_state_)
          {
            case WAIT_GOAL:
            {
              std_msgs::Empty emt;
              // wait_for_goal.publish(emt);
              stop_moving.publish(emt);
              if(have_trajector_){
                changeExecState(EXEC_TRAJ, "FSM");
              }
              break;
            }

            case EXEC_TRAJ:
            {
              if(!have_trajector_){
                changeExecState(WAIT_GOAL, "FSM");
              }
              else if(!trajectroy_tracker->still_running){
                std::lock_guard<std::mutex> lk(lock_on_solver);
                granted_execution = true;
                condition_on_solver.notify_one();
              }
              std_msgs::Empty emt;
              stat_moving.publish(emt);
              break;
            }
          }
      }
      t1 = clock();
    }else{
      usleep(10000);
    }
  }
}

std::thread FSMLinearTrajectoryTracker::execFSMThread() {
    return std::thread([&] { fsmExecutor(); });
}


void FSMLinearTrajectoryTracker::cmdExecutor() {
  clock_t t2, t1 = clock();
  while(is_allowed_for_execution){
    t2 = clock();
      if (!stop_pose_init){
        cout << "FSMLinearTrajectoryTracker: no odom" << endl;
      }else{
        Eigen::Vector3d pos, vel, acc;
        ros::Time time_now = ros::Time::now();
        int traj_id = 1;

        auto current_state = current_pose;
        Eigen::VectorXd state_vec(4);
        if(!init_kf_yaw){
            Eigen::VectorXd k_x0(1);
            k_x0 << stop_yaw_angle;
            kf_yaw->init(0, k_x0);
            init_kf_yaw =  true;
        }
        if(trajectroy_tracker->still_running && !trajectroy_tracker->force_terminate){
          pos(0) = current_state.pose.pose.position.x;
          pos(1) = current_state.pose.pose.position.y;
          pos(2) = current_state.pose.pose.position.z;
          current_yaw = current_state.pose.pose.orientation.x;
          state_vec << current_state.twist.twist.linear.x, current_state.twist.twist.linear.y
                    , current_state.twist.twist.linear.z, current_state.twist.twist.angular.z;
          vel(0) = (double)(state_vec(0)*cos(current_yaw) - state_vec(1)*sin(current_yaw));
          vel(1) = (double)(state_vec(0)*sin(current_yaw) + state_vec(1)*cos(current_yaw));
          vel(2) = current_state.twist.twist.linear.z;

          Eigen::Vector3d normalized_vector = (vel).normalized();
          double curr_yaw = std::atan2(normalized_vector[1], normalized_vector[0]);

          if(normalized_vector[1]<0){
            curr_yaw += 2*M_PI;
          }

          Eigen::VectorXd k_y(1);
          k_y <<  curr_yaw*yaw_sign;
          kf_yaw->update(k_y);

          previous_yaw = current_yaw;
          current_yaw =  kf_yaw->state()[0];
        }else {
            init_kf_yaw =  false;
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
        }else{
          // cout<< "Pose less than the minimum hight" << endl;
        }

        visualization_->drawState(pos, vel, 50, Eigen::Vector4d(0.7, 0.8, 0.9, 1));
        traj_cmd->push_back(pos);
        }
        t1 = clock();
  }
}

std::thread FSMLinearTrajectoryTracker::execCMDThread() {
    return std::thread([&] { cmdExecutor(); });
}

void FSMLinearTrajectoryTracker::safetyCallback(const ros::TimerEvent& e)
{
  visualization_->displayTrajWithColor(*traj_cmd, 0.03, Eigen::Vector4d(1, 1, 0, 1), 21);
  visualization_->displayTrajWithColor(*traj_real, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964, 1), 20);
}


// FSMLinearTrajectoryTracker::
}  // namespace hagen_planner


namespace backward
{
  backward::SignalHandling sh;
}

using namespace hagen_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hagen_planner_group_node");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  FSMLinearTrajectoryTracker fsm;
  fsm.init(nh);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
