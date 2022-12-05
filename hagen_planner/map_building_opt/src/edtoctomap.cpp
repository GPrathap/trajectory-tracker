#include "map_building_opt/edtoctomap.h"

namespace hagen_planner
{

  EDTOctoMap::EDTOctoMap(Eigen::Vector3d ori, double resolution, Eigen::Vector3d size)
  {
    this->origin_ = ori;
    this->resolution_sdf_ = resolution;
    this->resolution_inv_ = 1 / resolution_sdf_;
    this->map_size_ = size;
    for (int i = 0; i < 3; ++i){
        grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
    }
    min_range_ = origin_;
    max_range_ = origin_ + map_size_;
    min_vec_ = Eigen::Vector3i::Zero();
    max_vec_ = grid_size_ - Eigen::Vector3i::Ones();
  }

  void EDTOctoMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
  {
    min_pos(0) = max(min_pos(0), min_range_(0));
    min_pos(1) = max(min_pos(1), min_range_(1));
    min_pos(2) = max(min_pos(2), min_range_(2));
    max_pos(0) = min(max_pos(0), max_range_(0));
    max_pos(1) = min(max_pos(1), max_range_(1));
    max_pos(2) = min(max_pos(2), max_range_(2));
    Eigen::Vector3i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos - Eigen::Vector3d(resolution_sdf_ / 2, resolution_sdf_ / 2, resolution_sdf_ / 2), max_id);
  }

  bool EDTOctoMap::isInMap(Eigen::Vector3d pos)
  {
    if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4)
    {
      return false;
    }
    if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4)
    {
      return false;
    }
    return true;
  }

  void EDTOctoMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
  {
    for (int i = 0; i < 3; ++i)
      id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
  }

  void EDTOctoMap::indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos)
  {
    for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + 0.5) * resolution_sdf_ + origin_(i);
  }

  std::vector<Eigen::Vector3d> EDTOctoMap::getMapCurrentRange(){
    std::vector<Eigen::Vector3d> current_ranges;
    current_ranges.push_back(min_range_);
    current_ranges.push_back(max_range_);
    return current_ranges;
  }

  void EDTOctoMap::setUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
  {
    min_pos(0) = max(min_pos(0), min_range_(0));
    min_pos(1) = max(min_pos(1), min_range_(1));
    min_pos(2) = max(min_pos(2), min_range_(2));
    max_pos(0) = min(max_pos(0), max_range_(0));
    max_pos(1) = min(max_pos(1), max_range_(1));
    max_pos(2) = min(max_pos(2), max_range_(2));
    posToIndex(min_pos, min_vec_);
    posToIndex(max_pos - Eigen::Vector3d(resolution_sdf_ / 2, resolution_sdf_ / 2, resolution_sdf_ / 2), max_vec_);
  }

  void EDTOctoMap::loadStaticMap(pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_){
    tree->clear();
    for (int i = 0; i < (int)latest_cloud_->points.size(); i++) {
      octomap::point3d endpoint(latest_cloud_->points[i].x,latest_cloud_->points[i].y, latest_cloud_->points[i].z);
      tree->updateNode(endpoint, true);
    }
    distmap->update(); 
  }

  void EDTOctoMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!have_odom_)
    {
      cout << "EDTOctoMap: no odom_" << endl;
      return;
    }
    new_map_ = false;
    pcl::fromROSMsg(*msg, latest_cloud_);

    Eigen::Vector3d center(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
    if (isnan(center(0)) || isnan(center(1)) || isnan(center(2)))
      return;
    Eigen::Vector3d disp(update_range_, update_range_, update_range_ / 2.0);
    this->resetBuffer(center - disp, center + disp);
    cloud_inflate_vis_.clear();
    pcl::PointXYZ pt, pt_inf;
    Eigen::Vector3d p3d, p3d_inf;
    const int ifn = ceil(inflate_ * resolution_inv_);
    tree->clear();
    octomap::point3d max_(max_range_[0], max_range_[1], max_range_[2]);
    octomap::point3d min_(min_range_[0], min_range_[1], min_range_[2]);
    std::shared_ptr<DynamicEDTOctomap> temp_map(new DynamicEDTOctomap(update_range_, tree, min_, max_, false));
    for (int i = 0; i < (int)latest_cloud_.points.size(); i++) {
      pt = latest_cloud_.points[i];
      p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
      double dist_to_obs = (center - p3d).norm();
      if ( dist_to_obs < update_range_)
      {
        for (int x = -ifn; x <= ifn; ++x){
          for (int y = -ifn; y <= ifn; ++y){
            for (int z = -ifn; z <= ifn; ++z)
            {
              p3d_inf(0) = pt_inf.x = pt.x + x * resolution_sdf_;
              p3d_inf(1) = pt_inf.y = pt.y + y * resolution_sdf_;
              p3d_inf(2) = pt_inf.z = pt.z + 0.5 * z * resolution_sdf_;
              cloud_inflate_vis_.push_back(pt_inf);
            }
          }
        }
        octomap::point3d endpoint(latest_cloud_.points[i].x,latest_cloud_.points[i].y, latest_cloud_.points[i].z);
        tree->updateNode(endpoint, true);
      }
    }
    temp_map->update();
    distmap = temp_map;
    Eigen::Vector3d pose(3, 5,1);
    double dis1 = get_free_distance(pose);
    new_map_ = true;
    map_valid_ = true;
    cloud_inflate_vis_.width = cloud_inflate_vis_.points.size();
    cloud_inflate_vis_.height = 1;
    cloud_inflate_vis_.is_dense = true;
    cloud_inflate_vis_.header.frame_id = "map";
    cloud_inflate_vis_.header.seq = latest_cloud_.header.seq;
    cloud_inflate_vis_.header.stamp = latest_cloud_.header.stamp;
    sensor_msgs::PointCloud2 map_inflate_vis;
    pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);
    inflate_cloud_pub_.publish(map_inflate_vis);
    this->setUpdateRange(center - disp, center + disp);

    converetd_obs = DecompROS::cloud_to_vec(latest_cloud_);
    converetd_obs_previous = converetd_obs;
  }

  std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> EDTOctoMap::getFilteredMap(){
    return converetd_obs_previous;
  }

  void EDTOctoMap::publishPath(nav_msgs::Path path_msg){
    path_msg.header.frame_id = "map";
    path_pub.publish(path_msg);
  }

  void EDTOctoMap::publishEllipsoids(decom_ros_msgs::EllipsoidArray path_msg){
    path_msg.header.frame_id = "map";
    es_pub.publish(path_msg);
  }

  void EDTOctoMap::publishPolyhedron(decom_ros_msgs::PolyhedronArray path_msg){
    path_msg.header.frame_id = "map";
    poly_pub.publish(path_msg);
  }

  void EDTOctoMap::publishPolyhedronCorridor(decom_ros_msgs::PolyhedronArray path_msg){
    path_msg.header.frame_id = "map";
    poly_corri_pub.publish(path_msg);
  }

  void EDTOctoMap::publishEllipsoids1(decom_ros_msgs::EllipsoidArray path_msg){
    path_msg.header.frame_id = "map";
    es_pub1.publish(path_msg);
  }

  void EDTOctoMap::publishPolyhedron1(decom_ros_msgs::PolyhedronArray path_msg){
    path_msg.header.frame_id = "map";
    poly_pub1.publish(path_msg);
  }

  void EDTOctoMap::publishPolyhedronCorridor1(decom_ros_msgs::PolyhedronArray path_msg){
    path_msg.header.frame_id = "map";
    poly_corri_pub1.publish(path_msg);
  }

  void EDTOctoMap::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
      return;
    odom_ = *msg;
    odom_.header.frame_id = "map";
    have_odom_ = true;
  }

  std::vector<std::array<double, 6>> EDTOctoMap::getObsMap(){
    std::vector<std::array<double, 6>> map;
    return map;
  }

  std::vector<Eigen::Vector3d> EDTOctoMap::nearest_obstacles_to_current_pose(Eigen::Vector3d x
                  , int max_neighbours){
    
  std::vector<Eigen::Vector3d> neighbour_points;
    if(!isInMap(x)){
      std::cout<< "Point outside of the map" << x.transpose() << std::endl;
      return neighbour_points;
    }
    octomap::point3d p(x[0],x[1],x[2]);
    octomap::point3d closestObst;
    float distance;
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    Eigen::Vector3d obs(closestObst.x(), closestObst.y(), closestObst.z());
    neighbour_points.push_back(obs);
    return neighbour_points;
  }

  double EDTOctoMap::get_free_distance(Eigen::Vector3d x){
    octomap::point3d p(x[0],x[1],x[2]);
    double dis = distmap->getDistance(p);
    return dis;
  }

  void EDTOctoMap::get_close_obstacle(Eigen::Vector3d x, Eigen::Vector3d& close_obs, double& dis){
    octomap::point3d p(x[0],x[1],x[2]);
    octomap::point3d closestObst;
    float distance;
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    if(distance <0){
       distance = 20;
       closestObst.x() = x[0]+100;
       closestObst.y() = x[1]+100;
       closestObst.z() = x[2]+100;
    }
    Eigen::Vector3d obs_pose(closestObst.x(), closestObst.y(), closestObst.z());
    close_obs = obs_pose;
    dis = distance;
  }

  void EDTOctoMap::get_close_obstacles(Eigen::Vector3d pos, std::vector<Eigen::Vector3d>& obs_vec){

  }

  void EDTOctoMap::init(ros::NodeHandle& nh)
  {
    node_ = nh;
    /* ---------- param ---------- */
    node_.param("sdf_map/origin_x", origin_(0), -20.0);
    node_.param("sdf_map/origin_y", origin_(1), -20.0);
    node_.param("sdf_map/origin_z", origin_(2), 0.0);

    node_.param("sdf_map/map_size_x", map_size_(0), 40.0);
    node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
    node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

    node_.param("sdf_map/resolution_sdf", resolution_sdf_, 0.2);
    node_.param("sdf_map/ceil_height", ceil_height_, 2.0);
    node_.param("sdf_map/update_rate", update_rate_, 10.0);
    node_.param("sdf_map/update_range", update_range_, 5.0);
    node_.param("sdf_map/inflate", inflate_, 0.2);
    node_.param("sdf_map/radius_ignore", radius_ignore_, 0.2);
    node_.param("sdf_map/max_avoidance_distance", max_avoidance_distance_, 0.2);
    node_.param("sdf_map/min_avoidance_distance", min_avoidance_distance_, 0.2);

    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_.transpose() << endl;
    cout << "resolution: " << resolution_sdf_ << endl;

    /* ---------- sub and pub ---------- */
    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 10, &EDTOctoMap::odomCallback, this);
    cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1, &EDTOctoMap::cloudCallback, this);
    inflate_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/inflate_cloud", 1);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/planning/cloud", 1, true);
    path_pub = nh.advertise<nav_msgs::Path>("/planning/jsppath", 1, true);
    es_pub = nh.advertise<decom_ros_msgs::EllipsoidArray>("/planning/ellipsoid_array", 1, true);
    poly_pub = nh.advertise<decom_ros_msgs::PolyhedronArray>("/planning/polyhedron_array", 1, true);
    poly_corri_pub = nh.advertise<decom_ros_msgs::PolyhedronArray>("/planning/corrior", 1, true);
    es_pub1 = nh.advertise<decom_ros_msgs::EllipsoidArray>("/planning/ellipsoid_array_odom", 1, true);
    poly_pub1 = nh.advertise<decom_ros_msgs::PolyhedronArray>("/planning/polyhedron_array_odom", 1, true);
    poly_corri_pub1 = nh.advertise<decom_ros_msgs::PolyhedronArray>("/planning/corrior_odom", 1, true);
    have_odom_ = false;
    new_map_ = false;
    map_valid_ = false;

    resolution_inv_ = 1 / resolution_sdf_;
    for (int i = 0; i < 3; ++i)
      grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
    min_range_ = origin_;
    max_range_ = origin_ + map_size_;
    min_vec_ = Eigen::Vector3i::Zero();
    max_vec_ = grid_size_ - Eigen::Vector3i::Ones();
    tree = new octomap::OcTree(resolution_sdf_);
    octomap::point3d max_(max_range_[0], max_range_[1], max_range_[2]);
    octomap::point3d min_(min_range_[0], min_range_[1], min_range_[2]);
    tree->setBBXMax(max_);
    tree->setBBXMin(min_);
    std::shared_ptr<DynamicEDTOctomap> distmap_init(new DynamicEDTOctomap(update_range_, tree, min_, max_, false));
    distmap = distmap_init;
  }
}  // namespace hagen_planner
