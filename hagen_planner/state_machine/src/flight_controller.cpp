#include <state_machine/flight_controller.h>

namespace hagen_planner
{

    FlightController::FlightController(){

    }

    bool FlightController::drone_wait_till_auto_mode(){
      std::cout << "Waiting for turn on auto control model ..." << std::endl;
      while(!obtain_control()){
          std::cout << "Obtain control failed... make sure drone is in F mode..." << std::endl;
          // std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      std::cout << "Activated auto control model ..." << std::endl;
      return true;
    }

    void FlightController::localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                            sensor_msgs::NavSatFix& target,
                            sensor_msgs::NavSatFix& origin)
    {
      double deltaLon = target.longitude - origin.longitude;
      double deltaLat = target.latitude - origin.latitude;

      deltaNed.y = deltaLat * deg2rad * C_EARTH;
      deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
      deltaNed.z = target.altitude - origin.altitude;
    }

    bool FlightController::obtain_control()
    {
      dji_sdk::SDKControlAuthority authority;
      authority.request.control_enable=1;
      sdk_ctrl_authority_service.call(authority);
      if(!authority.response.result)
      {
          std::cout << "obtain control failed!" << std::endl;
        return false;
      }
      return true;
    }

    bool FlightController::release_control()
    {
      dji_sdk::SDKControlAuthority authority;
      authority.request.control_enable=0;
      sdk_ctrl_authority_service.call(authority);
      if(!authority.response.result)
      {
        std::cout <<  "releasing control failed!" << std::endl;
        return false;
      }
      return true;
    }

    bool FlightController::is_M100()
    {
      dji_sdk::QueryDroneVersion query;
      query_version_service.call(query);
      if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
      {
        return true;
      }
      return false;
    }

    void FlightController::stop_drone(){
        sensor_msgs::Joy controlVelYawRate;
        uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE            |
                    DJISDK::HORIZONTAL_GROUND   |
                    DJISDK::STABLE_ENABLE);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(flag);
        ///ctrlBrakePub.publish(controlVelYawRate);
        return;
    }

    bool FlightController::set_local_position()
    {
      dji_sdk::SetLocalPosRef localPosReferenceSetter;
      set_local_pos_reference.call(localPosReferenceSetter);
      return localPosReferenceSetter.response.result;
    }

    //  void FlightController::set_initial_position(float height){
    //   start_altitude = height;
    // }

    //  bool FlightController::takeoff_land(float height)
    // {
    //   dji_sdk::DroneTaskControl droneTaskControl;
    //   droneTaskControl.request.task = (int)height;
    //   drone_task_service.call(droneTaskControl);
    //   if(!droneTaskControl.response.result)
    //   {
    //     std::cout << "takeoff_land fail" << std::endl;
    //     return false;
    //   }
    //   return true;
    // }

    // geometry_msgs::Vector3 FlightController::toEulerAngle(Eigen::VectorXf quat)
    // {
    //   geometry_msgs::Vector3 ans;
    //   tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
    //   R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    //   return ans;
    // }

    // void FlightController::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    // {
    //   current_gps = *msg;
    // }

    // bool FlightController::drone_take_off(){

    //     bool takeoff_result;
    //     if (!set_local_position())
    //     {
    //       std::cout << "GPS health insufficient - No local frame reference for height. Exiting." << std::endl;
    //       return false;
    //     }

    //     if(is_M100())
    //     {
    //       std::cout << "M100 taking off!" << std::endl;
    //       takeoff_result = M100monitoredTakeoff();
    //     }
    //     else
    //     {
    //       std::cout << "A3/N3 taking off!" << std::endl;
    //       takeoff_result = monitoredTakeoff();
    //     }
    //     return takeoff_result;
    // }

    // void FlightController::initFlightController(float object_avoidance_zone_){
    //   object_avoidance_zone = object_avoidance_zone_;
    // }


    // bool FlightController::route_drone(Eigen::Vector4f desired_state
    //                                                 , Eigen::VectorXf local_current_pose){

    //   sensor_msgs::Joy ctrlVelYawRate;
    //   float xCmd, yCmd, zCmd;
    //   float yawDesiredRad;
    //   yawDesiredRad = 0;
    //   xCmd = desired_state[0];
    //   yCmd = desired_state[1];
    //   zCmd = desired_state[2];
    //   uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
    //             DJISDK::HORIZONTAL_VELOCITY |
    //             DJISDK::YAW_RATE            |
    //             DJISDK::HORIZONTAL_GROUND   |
    //             DJISDK::STABLE_ENABLE);

    //   ctrlVelYawRate.axes.push_back(xCmd);
    //   ctrlVelYawRate.axes.push_back(yCmd);
    //   ctrlVelYawRate.axes.push_back(zCmd);
    //   ctrlVelYawRate.axes.push_back(yawDesiredRad);
    //   ctrlVelYawRate.axes.push_back(flag);
    //   std::cout<< FCYN("Drone position: ")<< local_current_pose;
    //   if(local_current_pose[2]<drone_min_height){
    //       BOOST_LOG_TRIVIAL(fatal)<< FRED("Drone height can not be less than")<< drone_min_height << " m." << "Current height " << local_current_pose[2];
    //       return false;
    //   }
    //   ctrlVelYawRatePub.publish(ctrlVelYawRate);
    // }

    // bool FlightController::route_drone(Eigen::VectorXf next_target
    //           , Eigen::VectorXf local_current_pose, Eigen::VectorXf local_current_orientation)
    // {

    //     auto target_position = next_target;
    //     float xCmd, yCmd, zCmd;

    //     double xOffsetRemaining = target_position[0] - local_current_pose[0];
    //     double yOffsetRemaining = target_position[1] - local_current_pose[1];

    //     // TODO check how to reduce giving more yaw angle...
    //     double yawInRad          = local_current_orientation[2];

    //     if (abs(xOffsetRemaining) >= max_speed)
    //       xCmd = (xOffsetRemaining>0) ? max_speed : -1 * max_speed;
    //     else
    //       xCmd = xOffsetRemaining;

    //     if (abs(yOffsetRemaining) >= max_speed)
    //       yCmd = (yOffsetRemaining>0) ? max_speed : -1 * max_speed;
    //     else
    //       yCmd = yOffsetRemaining;

    //     zCmd =  target_position[2];

    //     std::cout<< FCYN("Drone position: ")<< xCmd << "," << yCmd << ","<< zCmd;
    //     if(zCmd<1.0){
    //         BOOST_LOG_TRIVIAL(fatal)<< FRED("Drone height can not be less than 1 m...!");
    //         return false;
    //     }
    //     std::cout<< FCYN("Yaw angle: ") << yawInRad;
    //     sensor_msgs::Joy controlPosYaw;
    //     controlPosYaw.axes.push_back(xCmd);
    //     controlPosYaw.axes.push_back(yCmd);
    //     controlPosYaw.axes.push_back(zCmd);
    //     controlPosYaw.axes.push_back(yawInRad);
    //     ctrlPosYawPub.publish(controlPosYaw);
    //     return true;
    // }

    // void FlightController::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
    // {
    //   flight_status = msg->data;
    // }

    // void FlightController::display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
    // {
    //   display_mode = msg->data;
    // }

    // bool FlightController::monitoredTakeoff()
    // {
    //   ros::Time start_time = ros::Time::now();

    //   if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    //     return false;
    //   }

    //   ros::Duration(0.01).sleep();
    //   ros::spinOnce();

    //   // Step 1.1: Spin the motor
    //   while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
    //         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
    //         ros::Time::now() - start_time < ros::Duration(5)) {
    //     ros::Duration(0.01).sleep();
    //     ros::spinOnce();
    //   }

    //   if(ros::Time::now() - start_time > ros::Duration(5)) {
    //     ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    //     return false;
    //   }
    //   else {
    //     start_time = ros::Time::now();
    //     ROS_INFO("Motor Spinning ...");
    //     ros::spinOnce();
    //   }

      // Step 1.2: Get in to the air
      // while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
      //         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
      //         ros::Time::now() - start_time < ros::Duration(20)) {
      //   ros::Duration(0.01).sleep();
      //   ros::spinOnce();
      // }

      // if(ros::Time::now() - start_time > ros::Duration(20)) {
      //   ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
      //   return false;
      // }
      // else {
      //   start_time = ros::Time::now();
      //   ROS_INFO("Ascending...");
      //   ros::spinOnce();
      // }

      // // Final check: Finished takeoff
      // while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
      //         ros::Time::now() - start_time < ros::Duration(20)) {
      //   ros::Duration(0.01).sleep();
      //   ros::spinOnce();
      // }

      // if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
      // {
      //   ROS_INFO("Successful takeoff!");
      //   start_time = ros::Time::now();
      // }
      // else
      // {
      //   ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
      //   return false;
      // }

    //   return true;
    // }

    // bool FlightController::M100monitoredTakeoff()
    // {
    //   ros::Time start_time = ros::Time::now();
    //   float home_altitude = current_gps.altitude;
    //   if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
    //   {
    //     return false;
    //   }

    //   ros::Duration(0.01).sleep();
    //   ros::spinOnce();

    //   // Step 1: If M100 is not in the air after 10 seconds, fail.
    //   while (ros::Time::now() - start_time < ros::Duration(10))
    //   {
    //     ros::Duration(0.01).sleep();
    //     ros::spinOnce();
    //   }

    //   if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
    //       current_gps.altitude - home_altitude < 1.0)
    //   {
    //     ROS_ERROR("Takeoff failed.");
    //     return false;
    //   }
    //   else
    //   {
    //     start_time = ros::Time::now();
    //     ROS_INFO("Successful takeoff!");
    //     ros::spinOnce();
    //   }
    //   return true;
    // }
}

