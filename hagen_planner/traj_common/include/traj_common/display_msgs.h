#ifndef PROJECT_DISPLAT_MSGS_H
#define PROJECT_DISPLAT_MSGS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <decom_ros_msgs/PolyhedronArray.h>
#include <decom_ros_msgs/Polyhedron.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Sparse>
#include <motion_model/model.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <traj_common/corridor.h>

namespace hagen_planner{
    class DisplayMsgs
    {
    private:
        
        geometry_msgs::Point pt, nm;
        decom_ros_msgs::Polyhedron poly_msg;
    public:
        int horizon;
        nav_msgs::Path refTraj_msg;
        nav_msgs::Path trajPred_msg;
        decom_ros_msgs::PolyhedronArray corridor_array_msg;
        decom_ros_msgs::PolyhedronArray tunnel_array_msg;
        decom_ros_msgs::PolyhedronArray empty_poly_msg;
        visualization_msgs::Marker drone_msg;
        visualization_msgs::Marker theta_msg;
        std::shared_ptr<Corridor> corridor;

        DisplayMsgs();
        void displayRefTraj(std::vector<Eigen::Vector3d> path);
        void displayCorridors();
        void displayOneTunnel(int horizon_, double theta_, Eigen::Vector3d position, Eigen::Vector3d tangent);
        void displayDrone(Eigen::SparseMatrix<double> &state);
        void displayTheta(Eigen::SparseMatrix<double> &state, Eigen::Vector3d pos, Eigen::Vector3d vel);
        void displayPredict(Eigen::SparseMatrix<double> &statePredict);
        void clearTunnels();
        void pubTunnels(ros::Publisher& pub);
    };
    
}//namespace hagen_planner

#endif