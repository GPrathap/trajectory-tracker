#include <traj_common/planning_visualization.h>

using std::cout;
using std::endl;
namespace hagen_planner
{
PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh)
{
    node = nh;
    traj_pub = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 10);
    search_space_publisher = node.advertise<visualization_msgs::Marker>("/planning_vis/search_space", 10);
    pub_rviz_markers_ = node.advertise<visualization_msgs::Marker>("/planning_vis/cone", 10);
    traj_whole_pub = node.advertise<visualization_msgs::Marker>("/planning_vis/traj", 10);
    state_pub = node.advertise<visualization_msgs::Marker>("planning_vis/state", 10);
    free_space_pub = node.advertise<jsk_recognition_msgs::PolygonArray>("planning_vis/free_space", 10);

    cmd_pub = node.advertise<hagen_msgs::PoseCommand>("/planning_mpc/drone_1/position_cmd",10);
    refer_pub = node.advertise<nav_msgs::Path>("/planning_mpc/refer_path", 1000);
    drone_pub = node.advertise<visualization_msgs::Marker>("/planning_mpc/drone_pose", 1000);
    global_pub = node.advertise<visualization_msgs::Marker>("/planning_mpc/global_pose", 1000);
    vis_polytope_pub  = node.advertise<decom_ros_msgs::PolyhedronArray>("/planning_mpc/polyhedron_corridor_mesh", 1000, true);
    flight_tunnel_pub = node.advertise<decom_ros_msgs::PolyhedronArray>("/planning_mpc/flight_tunnel", 1000, true);
    predict_pub = node.advertise<nav_msgs::Path>("/planning_mpc/predict_path", 1000);
    globalOdom_pub = node.advertise<nav_msgs::Odometry>("/planning_mpc/globalOdom", 1000);
}

void PlanningVisualization::displayFreeSpace(std::vector<std::vector<Eigen::Vector3d>>& verticies){
  jsk_recognition_msgs::PolygonArray array_msg;
  array_msg.header.frame_id = "map";
  array_msg.header.stamp = ros::Time::now();
  int index_poly= 0;
  for(auto poly: verticies){
    geometry_msgs::PolygonStamped polygon_;
    polygon_.header.frame_id = "map";
    // geometry_msgs::Polygon poly_points;
    for(int i=0; i< poly.size(); i++){
      geometry_msgs::Point32 point;
      point.x = poly[i][0];
      point.y = poly[i][1];
      point.z = poly[i][2];
      // poly_points.points.push_back(point);
      polygon_.polygon.points.push_back(point);
    }
    geometry_msgs::Point32 point;
    point.x = poly[0][0];
    point.y = poly[0][1];
    point.z = poly[0][2];
      // poly_points.points.push_back(point);
    polygon_.polygon.points.push_back(point);

    array_msg.polygons.push_back(polygon_);
    array_msg.labels.push_back(index_poly);
    array_msg.likelihood.push_back(0.4);
    index_poly++;
  }
  free_space_pub.publish(array_msg);
}

void PlanningVisualization::displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color,
                                              int id)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0, mk.pose.orientation.w = 1.0;
  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0), pt.y = list[i](1), pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
}


void PlanningVisualization::displayTrajWithColor(boost::circular_buffer<Eigen::Vector3d>& path, double resolution,
                          Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_whole_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for(boost::circular_buffer<Eigen::Vector3d>::iterator it = path.begin(); it< path.end(); it++){
    Eigen::Vector3d pose = *it;
    pt.x = pose(0);
    pt.y = pose(1);
    pt.z = pose(2);
    mk.points.push_back(pt);
  }
  traj_whole_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void PlanningVisualization::drawBspline(NonUniformBspline bspline, double size, Eigen::Vector4d color,
                                        bool show_ctrl_pts, double size2, Eigen::Vector4d color2, int id1, int id2)
{
  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);
  for (double t = tm; t <= tmp; t += 0.01)
  {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);

  // draw the control point
  if (!show_ctrl_pts)
    return;


  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();

  vector<Eigen::Vector3d> ctp;
  for (int i = 0; i < int(ctrl_pts.rows()); ++i)
  {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }
  displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
}

void PlanningVisualization::publish_marker(visualization_msgs::Marker marker, int id_, std::string name_space){
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = name_space;
        marker.id = id_;
        search_space_publisher.publish(marker);
        return;
}

void PlanningVisualization::drawObsMap(visualization_msgs::MarkerArray marker_array,double resolution, Eigen::Vector4d color, int id){
  // marker.header.frame_id = "map";
  // marker.header.stamp = ros::Time();
  // marker.ns = name_space;
  // marker.id = id_;
}

void PlanningVisualization::drawConePoints(Eigen::MatrixXd poses, double resolution, Eigen::Vector4d color, int id){

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pub_rviz_markers_.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0, mk.pose.orientation.w = 1.0;
  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;
  geometry_msgs::Point pt;
  int poses_index = (int)poses.cols()/3;
  for (int i = 0; i < (int)poses.rows(); i++)
  {
    // for (int j = 0; j < poses_index; j++)
    // {
      // pt.x = poses(i,j), pt.y = poses(i, poses_index+j), pt.z = poses(i,poses_index*2+j);
      pt.x = poses(i,0), pt.y = poses(i, 1), pt.z = poses(i,2);
      mk.points.push_back(pt);
    // }
  }
  pub_rviz_markers_.publish(mk);
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution, Eigen::Vector4d color, int id)
{
  vector<Eigen::Vector3d> goal_vec = { goal };

  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawPath(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id, Eigen::Vector4d color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "map";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;
  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2) ;
  mk_state.points.push_back(pt);
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);
  state_pub.publish(mk_state);
}

// PlanningVisualization::
}  // namespace hagen_planner