#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

using namespace visualization_msgs;
using namespace geometry_msgs;

// Global variables
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::Publisher goal_pub;

PoseStamped goal;
Marker marker_text;

double getYawFromQuat(geometry_msgs::Quaternion quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void updateGoal(const Pose &pose)
{
  goal.pose = pose;
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

  updateGoal(feedback->pose);
  server->applyChanges();

  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose = feedback->pose;
  goal_pub.publish(goal);
}

Marker arrowMarker( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale * 0.75;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 0.5;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;
  return marker;
}

Marker textMarker(PoseStamped msg)
{

  double yaw = getYawFromQuat(msg.pose.orientation);
  std::string text_msg = "           x:" +std::to_string(msg.pose.position.x)+ "\n";
              text_msg += "           y:" +std::to_string(msg.pose.position.y)+ "\n";
              text_msg += "           z:" +std::to_string(msg.pose.position.z)+ "\n";
              text_msg += "           yaw:" +std::to_string(yaw * 57.2958)+ "\n";

  float _scale = 0.08;
  marker_text.header.frame_id = "map";
//  marker_text.header.stamp = ros::Time::now();
  marker_text.ns = "goal_text";
  marker_text.type = Marker::TEXT_VIEW_FACING;
  marker_text.text =  text_msg;
  // marker_text.scale.x = _scale;
  // marker_text.scale.y = _scale;
  marker_text.scale.z = _scale;
  marker_text.pose.position = msg.pose.position;
  marker_text.pose.position.z += 0.15;
  marker_text.pose.orientation.x = 0.;
  marker_text.pose.orientation.y = 0.;
  marker_text.pose.orientation.z = 0.;

  marker_text.pose.orientation.w = 1.;

  marker_text.color.r = 0.8;
  marker_text.color.g = 0.0;
  marker_text.color.b = 0.0;
  marker_text.color.a = 1.0;
  return marker_text;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( arrowMarker(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void makeQuadrocopterMarker( const tf::Vector3& position)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "goal_pose";
  int_marker.description = "GOAL";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void goalUpdateCb(const PoseStamped &goal_)
{
  goal = goal_;
  server->setPose("goal_pose", goal.pose);
  server->applyChanges();
}


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "marker_server");
  ros::NodeHandle n;


  // Init marker server
  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
  ros::Duration(0.1).sleep();
  makeQuadrocopterMarker(tf::Vector3(0, 0, 0));
  server->applyChanges();


  goal_pub = n.advertise<PoseStamped>("/goal", 10);
  ros::Publisher marker_text_pub = n.advertise<Marker>("/goal/marker_text", 10);
  ros::Subscriber goal_sub = n.subscribe("/goal", 10, &goalUpdateCb);

  ros::Rate loop_rate(15);

  while (ros::ok())
  {
      marker_text_pub.publish(textMarker(goal));
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
