#ifndef _EDTOACTO_MAP_H
#define _EDTOACTO_MAP_H

#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <chrono>
#include <fstream>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <boost/timer.hpp>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <decom_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>


using namespace std;
namespace hagen_planner
{
    class EDTOctoMap
    {
        private:
            // data are saved in vector
            std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
            std::vector<double> distance_buffer_;
            std::vector<double> no_cloud_buffer_;
            std::vector<double> distance_buffer_neg_;
            std::vector<double> tmp_buffer1_, tmp_buffer2_;

            // map property
            Eigen::Vector3d min_range_, max_range_;  // map range in pos
            Eigen::Vector3i grid_size_;              // map range in index
            Eigen::Vector3i min_vec_, max_vec_;      // the min and max updated range, unit is 1

            void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id);
            void indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos);

            template <typename F_get_val, typename F_set_val>
            void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

            std::mutex lock_on_map; 
            std::condition_variable condition_on_map;
            bool allow_updating_map = false;


        public:
            EDTOctoMap() {}
            EDTOctoMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size);
            ~EDTOctoMap() {}
            void init(ros::NodeHandle& nh);

            std::vector<Eigen::Vector3d> getMapCurrentRange();
            bool isInMap(Eigen::Vector3d pos);
            std::vector<std::array<double, 6>> getObsMap();
            /* get state */
            bool odomValid() { return have_odom_; }
            bool mapValid() { return map_valid_; }
            nav_msgs::Odometry getOdom() { return odom_; }
            void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) { ori = origin_, size = map_size_; }
            double getResolution() { return resolution_sdf_; }
            double getIgnoreRadius() { return radius_ignore_; }
            std::vector<Eigen::Vector3d> nearest_obstacles_to_current_pose(Eigen::Vector3d x
                        , int max_neighbours);
            double get_free_distance(Eigen::Vector3d x);
            void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
            void setUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos);
            bool collision_free(Eigen::Vector3d start, Eigen::Vector3d end);
            void get_close_obstacle(Eigen::Vector3d x, Eigen::Vector3d& close_obs, double& dis);
            void get_close_obstacles(Eigen::Vector3d pos, std::vector<Eigen::Vector3d>& obs_vec);
            void get_polyhedral_eql(Eigen::MatrixXd vertices, Eigen::MatrixXd& set_enql);
            void loadStaticMap(pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_);
            double mapMinAlt(){
                return min_range_[2];
            }
            double mapHighAlt(){
                return max_range_[2];
            }
            std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> getFilteredMap();
            typedef shared_ptr<EDTOctoMap> Ptr;
            double resolution_sdf_, resolution_inv_;
            double max_avoidance_distance_ = 1.0;
            double min_avoidance_distance_ = 1.0;
            double inflate_, update_range_, radius_ignore_;
            Eigen::Vector3d origin_, map_size_;
            
            double ceil_height_;
            double update_rate_;

            nav_msgs::Odometry odom_;
            bool have_odom_;

            pcl::PointCloud<pcl::PointXYZ> latest_cloud_, cloud_inflate_vis_;
            bool new_map_, map_valid_;

            ros::NodeHandle node_;
            ros::Subscriber odom_sub_, cloud_sub_;
            ros::Publisher inflate_cloud_pub_, octomap_publisher_;
            ros::Timer update_timer_;
            ros::Publisher cloud_pub, path_pub, es_pub, poly_pub, poly_corri_pub,
                        es_pub1, poly_pub1, poly_corri_pub1;

            void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
            void cloudCallback(const octomap_msgs::OctomapConstPtr& msg);
            void odomCallback(const nav_msgs::OdometryConstPtr& msg);
            void updateCallback(const ros::TimerEvent& e);
            void publishPath(nav_msgs::Path path_msg);

            void publishEllipsoids(decom_ros_msgs::EllipsoidArray path_msg);
            void publishPolyhedron(decom_ros_msgs::PolyhedronArray path_msg);
            void publishPolyhedronCorridor(decom_ros_msgs::PolyhedronArray path_msg);
            void publishEllipsoids1(decom_ros_msgs::EllipsoidArray path_msg);
            void publishPolyhedron1(decom_ros_msgs::PolyhedronArray path_msg);
            void publishPolyhedronCorridor1(decom_ros_msgs::PolyhedronArray path_msg);

            std::vector<std::array<double, 6>> _objects_map;

            octomap::OcTree *tree = NULL;
            std::shared_ptr<DynamicEDTOctomap> distmap;
            std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> converetd_obs;
            std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> converetd_obs_previous;
    };
}
#endif