
#include <state_machine/fsm_linearized_trajectory_tracker.h>

namespace hagen_planner
{
void FSMLinearizedTrajectoryTracker::init(ros::NodeHandle& nh)
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

  trajectroy_tracker.reset(new LinearizedMPCTrajectoryTracker);
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

  previous_wp << 0, 0;

  safety_timer_ = node_.createTimer(ros::Duration(5.0), &FSMLinearizedTrajectoryTracker::safetyCallback, this);
  waypoint_sub_ = node_.subscribe("/planner/waypoints", 1, &FSMLinearizedTrajectoryTracker::waypointCallback, this);

  replan_pub_ = node_.advertise<std_msgs::Empty>("/planning/replan", 10);
  wait_for_goal = node_.advertise<std_msgs::Empty>("/planning/wait_for_goal", 10);
  stat_moving = node_.advertise<std_msgs::Empty>("/planning/start_moving", 10);
  stop_moving = node_.advertise<std_msgs::Empty>("/planning/stop_moving", 10);
  bspline_pub_ = node_.advertise<state_machine::Bspline>("/planning/bspline", 10);
  odometry_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 50, &FSMLinearizedTrajectoryTracker::odomCallback, this);
  obstacle_poses_sub_ = node_.subscribe<visualization_msgs::MarkerArray>("/planner/detected_obs", 50, &FSMLinearizedTrajectoryTracker::detectedObs, this);
  lidar_poses_2d_sub_ = node_.subscribe<sensor_msgs::LaserScan>("/scan", 50, &FSMLinearizedTrajectoryTracker::laserScan, this);
  vehicle_current_pose_sub_ = node_.subscribe<nav_msgs::Odometry>("/planning/current_state", 5, &FSMLinearizedTrajectoryTracker::currentPoseCallback, this);
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

void FSMLinearizedTrajectoryTracker::odomCallback(const nav_msgs::OdometryConstPtr& msg){
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

void FSMLinearizedTrajectoryTracker::detectedObs(const visualization_msgs::MarkerArray::ConstPtr& msg){
  
}

void FSMLinearizedTrajectoryTracker::laserScan(const sensor_msgs::LaserScan::ConstPtr& scan_in){
  // laser_geometry::LaserProjection projector_;
  projector_.projectLaser(*scan_in, laser_cloud);
}

void FSMLinearizedTrajectoryTracker::currentPoseCallback(const nav_msgs::OdometryConstPtr& msg){
  current_pose = *msg;
}

void FSMLinearizedTrajectoryTracker::stopExecutionCallback(const std_msgs::Empty msg){
 
}

void FSMLinearizedTrajectoryTracker::continueExecutionCallback(const std_msgs::Empty msg){
 
}

void FSMLinearizedTrajectoryTracker::waypointCallback(const nav_msgs::PathConstPtr& msg)
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
  stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
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
  trigger_ = true;
  
  double tm, tmp;
  auto trajectory = bspline_utils_->traj_pos_;
  trajectory.getTimeSpan(tm, tmp);
  std::cout<< "time intervel of the estimaton: " << tmp-tm << std::endl;
  std::vector<Eigen::Vector2d> poses;
  std::cout<< "time intervel of the estimaton: " << tmp-tm << std::endl;
}


void FSMLinearizedTrajectoryTracker::changeExecState(EXEC_STATE new_state, string pos_call)
{
  string state_str[2] = {"Waiting for an initial trajectory", "Tracking" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void FSMLinearizedTrajectoryTracker::printExecState(){
  string state_str[2] = {"Waiting for an initial trajectory", "Tracking"};
  cout << "[Tracker]: " + state_str[int(exec_state_)] << endl;
}

double FSMLinearizedTrajectoryTracker::getYawFromQuat(const geometry_msgs::Quaternion &data){
    tf::Quaternion quat(data.x, data.y, data.z, data.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void FSMLinearizedTrajectoryTracker::solverThread() {
  while(is_allowed_for_execution){
    std::cout<< "Waiting to start execution of solver..." << std::endl;
    std::unique_lock<std::mutex> lk(lock_on_solver);
    condition_on_solver.wait(lk, [&]{return granted_execution;});
    std::cout<< "Starting the solver thread..." << std::endl;
    trees.clear();
    trees_real.clear();
    start_pt_(0) = odom.pose.pose.position.x;
    start_pt_(1) = odom.pose.pose.position.y;
    start_pt_(2) = odom.pose.pose.position.z;
    trajectroy_tracker->force_terminate = false;
   
     trajectroy_tracker->x0.resize(3,1);
    trajectroy_tracker->x0.coeffRef(0,0) = start_pt_(0);
    trajectroy_tracker->x0.coeffRef(1,0) = start_pt_(1);
    trajectroy_tracker->x0.coeffRef(2,0) = 0;

    trajectroy_tracker->xs.resize(3,1);
    trajectroy_tracker->xs.coeffRef(0,0) = end_pt_(0);
    trajectroy_tracker->xs.coeffRef(1,0) = end_pt_(1);
    trajectroy_tracker->xs.coeffRef(2,0) = 0;

    trajectroy_tracker->u0.resize(2,1);
    trajectroy_tracker->u0.coeffRef(0,0) = std::sqrt(std::pow(odom.twist.twist.linear.x, 2) + std::pow(odom.twist.twist.linear.y, 2));
    trajectroy_tracker->u0.coeffRef(1,0) = 0;
    
    trajectroy_tracker->still_running = true;
    granted_execution_global = true;
    condition_on_global_solver.notify_all();
    trigger_ = false;
    trajectroy_tracker->mpc_solver();
    
    std::cout<< "Solver is finished..." << std::endl;
    have_trajector_ = false;
    granted_execution = false;
  
    stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
    stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
    lk.unlock();
    condition_on_solver.notify_all();
  }
  return;
}

void FSMLinearizedTrajectoryTracker::GlobalTrajectoryThread() {
  while(is_allowed_for_execution){
    std::cout<< "Waiting to start execution of global trajectory..." << std::endl;
    std::unique_lock<std::mutex> lgk(lock_on_global_solver);
    condition_on_global_solver.wait(lgk, [&]{return granted_execution_global;});
    std::cout<< "Starting the global trajectory thread..." << std::endl;
    granted_execution_global = false;
    trajectroy_tracker->global_solver();
    lgk.unlock();
    condition_on_global_solver.notify_all();
  }
  return;
}

std::thread FSMLinearizedTrajectoryTracker::startSolverThread() {
    return std::thread([&] { solverThread(); });
}

std::thread FSMLinearizedTrajectoryTracker::startGlobalTrajectoryThread() {
    return std::thread([&] { GlobalTrajectoryThread(); });
}

void FSMLinearizedTrajectoryTracker::fsmExecutor(){
  int fsm_num = 0;
  clock_t t2, t1 = clock();
  while(is_allowed_for_execution){
    t2 = clock();
    if((t2-t1) > 1.0){
        fsm_num++;
        if (fsm_num == 100)
        {
          if(edt_env_->mapValid()){
              printExecState();
          }
          
          fsm_num = 0;
        }
        if (!edt_env_->odomValid()){
            cout << "FSMLinearizedTrajectoryTracker: no odom." << endl;
        }
        else{
          switch (exec_state_)
          {
            case WAIT_GOAL:
            {
              std_msgs::Empty emt;
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
              else if(trigger_){
                std::cout<< "Solver start running again.....------------->" << std::endl;
                std::lock_guard<std::mutex> lk(lock_on_solver);
                granted_execution = true;
                condition_on_solver.notify_one();
                trigger_ = false;
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

std::thread FSMLinearizedTrajectoryTracker::execFSMThread() {
    return std::thread([&] { fsmExecutor(); });
}

void FSMLinearizedTrajectoryTracker::cmdExecutor() {
  clock_t t2, t1 = clock();
  bool station_keeping = false;
  while(is_allowed_for_execution){
    t2 = clock();
    if((t2-t1) > 1.0){
      if (!stop_pose_init){
        cout << "FSMLinearizedTrajectoryTracker: no odom" << endl;
      }else{
        Eigen::Vector3d pos, vel, acc;
        ros::Time time_now = ros::Time::now();
        int traj_id = 1;

        // std::cout<< "trajectroy_tracker->still_running" << trajectroy_tracker->still_running << std::endl;
        auto current_state = current_pose;
        Eigen::VectorXd state_vec(4);
        if(!init_kf_yaw){
            Eigen::VectorXd k_x0(1);
            k_x0 << stop_yaw_angle;
            kf_yaw->init(0, k_x0);
            init_kf_yaw =  true;
        }
        double current_yaw_dot =0;
        if(trajectroy_tracker->still_running && !trajectroy_tracker->force_terminate){
          current_yaw_dot = current_state.twist.twist.angular.x;
          current_yaw = current_state.twist.twist.angular.z;
          pos(0) = current_state.pose.pose.position.x;
          pos(1) = current_state.pose.pose.position.y;
          pos(2) = current_state.pose.pose.position.z;
          
          if(!std::isnan(pos(0)) && !std::isnan(pos(1))){
              double dis = (pos.head(2) - previous_wp.head(2)).norm();
              if(dis>2.0 && !std::isnan(current_yaw)){
                previous_wp = pos.head(2);
              }
          }
          vel(0) = (double)(current_state.twist.twist.linear.x);
          vel(1) = (double)(current_state.twist.twist.linear.y);
          vel(2) = 0.0;
          station_keeping = false;
        } else {
            init_kf_yaw =  false;
            pos = stop_pose;
            Eigen::VectorXd k_y(1);
            k_y <<  0;
            kf_yaw->update(k_y); 
            previous_yaw = current_yaw;
            current_yaw =  0.0;
            station_keeping = true;
            vel.setZero();
        }

        cmd.yaw = current_yaw;
        cmd.yaw_dot = current_yaw_dot;
        cmd.header.stamp = time_now;
        cmd.header.frame_id = "map";
        cmd.trajectory_flag = hagen_msgs::PoseCommand::TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id;

        cmd.position.x = pos(0);
        cmd.position.y = pos(1);
        cmd.position.z = pos(2);

        cmd.velocity.x = vel(0);
        cmd.velocity.y = vel(1);
        cmd.velocity.z = vel(2);

        cmd.acceleration.x = acc(0);
        cmd.acceleration.y = acc(1);
        cmd.acceleration.z = acc(2);
        pos_cmd_pub.publish(cmd);

        visualization_->drawState(pos, vel, 50, Eigen::Vector4d(0.7, 0.8, 0.9, 1));
        traj_cmd->push_back(pos);
       
        }
        t1 = clock();
    }else{
      usleep(5000);
    }
  }
}

std::thread FSMLinearizedTrajectoryTracker::execCMDThread() {
    return std::thread([&] { cmdExecutor(); });
}

void FSMLinearizedTrajectoryTracker::safetyCallback(const ros::TimerEvent& e)
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
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  FSMLinearizedTrajectoryTracker fsm;
  fsm.init(nh);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
