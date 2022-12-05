#include <traj_common/display_msgs.h>


using namespace std;
namespace hagen_planner{
    DisplayMsgs::DisplayMsgs(){
        // initialize msgs:
        refTraj_msg.header.frame_id = "map";
        trajPred_msg.header.frame_id = "map";
        corridor_array_msg.header.frame_id = "map";
        tunnel_array_msg.header.frame_id = "map";
        empty_poly_msg.header.frame_id = "map";
        drone_msg.header.frame_id = "map";
        drone_msg.type = visualization_msgs::Marker::ARROW;
        drone_msg.action = visualization_msgs::Marker::ADD;
        drone_msg.scale.x = 0.06;
        drone_msg.scale.y = 0.1;
        drone_msg.scale.z = 0;
        drone_msg.color.a = 1;
        drone_msg.color.r = 1;
        drone_msg.color.g = 0;
        drone_msg.color.b = 0;
        drone_msg.pose.orientation.w = 1;
        theta_msg = drone_msg;
        theta_msg.color.r = 0;
        theta_msg.color.b = 1;

        // pre malloc for trajPred_msg
        trajPred_msg.poses.resize(horizon);
        empty_poly_msg.polyhedrons.clear();
        // empty_poly_msg.ids.clear();
    }   

    void DisplayMsgs::displayRefTraj(std::vector<Eigen::Vector3d> path){
        geometry_msgs::PoseStamped tmpPose;
        double theta = 0;
        for(auto pos : path){
            tmpPose.pose.position.x = pos(0);
            tmpPose.pose.position.y = pos(1);
            tmpPose.pose.position.z = pos(2);
            refTraj_msg.poses.push_back(tmpPose);
            theta += 0.01;
        }
    }

    void DisplayMsgs::displayCorridors(){
        corridor_array_msg.polyhedrons.clear();
        for ( int i = 0; i < corridor->polys.size(); i++ )
        {
            auto poly = corridor->polys[i];
            poly_msg.points.clear();
            poly_msg.normals.clear();
            for ( auto point : poly.points ){
                pt.x = point(0);
                pt.y = point(1);
                pt.z = point(2);
                poly_msg.points.push_back(pt);
            }
            for ( auto normal : poly.normals ){
                nm.x = normal(0);
                nm.y = normal(1);
                nm.z = normal(2);
                poly_msg.normals.push_back(nm);
            }
            corridor_array_msg.polyhedrons.push_back(poly_msg);
            // corridor_array_msg.ids.push_back(i);
            // tunnel_array_msg.polyhedrons.push_back(poly_msg);
            // tunnel_array_msg.ids.push_back(i);
        }
    }
    void DisplayMsgs::clearTunnels(){
        tunnel_array_msg.polyhedrons.clear();
        // tunnel_array_msg.ids.clear();
    };
    void DisplayMsgs::displayOneTunnel(int horizon_, double theta_, Eigen::Vector3d position, Eigen::Vector3d tangent){
        if (horizon_ == 0){
            tunnel_array_msg.polyhedrons.clear();
            // tunnel_array_msg.ids.clear();
        }
        poly_msg.points.clear();
        poly_msg.normals.clear();
        for ( int j = 0; j < corridor->tunnel.rows(); j++ ){
            Eigen::Vector3d normal = corridor->tunnel.block(j, 0, 1, 3).transpose().normalized();
            double D = corridor->tunnel(j,3) / corridor->tunnel.block(j, 0, 1, 3).transpose().norm();
            Eigen::Vector3d point_on_plane = - normal * D;
            pt.x = point_on_plane(0);
            pt.y = point_on_plane(1);
            pt.z = point_on_plane(2);
            poly_msg.points.push_back(pt);
            nm.x = normal(0);
            nm.y = normal(1);
            nm.z = normal(2);
            poly_msg.normals.push_back(nm);
        }
        Eigen::Vector3d extra_point, extra_normal = tangent.normalized();

        extra_point = position + extra_normal * 1e-1;
        pt.x = extra_point(0);
        pt.y = extra_point(1);
        pt.z = extra_point(2);
        poly_msg.points.push_back(pt);
        nm.x = extra_normal(0);
        nm.y = extra_normal(1);
        nm.z = extra_normal(2);
        poly_msg.normals.push_back(nm);

        extra_point = position - extra_normal * 1e-1;
        pt.x = extra_point(0);
        pt.y = extra_point(1);
        pt.z = extra_point(2);
        poly_msg.points.push_back(pt);
        nm.x = -extra_normal(0);
        nm.y = -extra_normal(1);
        nm.z = -extra_normal(2);
        poly_msg.normals.push_back(nm);
        tunnel_array_msg.polyhedrons.push_back(poly_msg);
        // tunnel_array_msg.ids.push_back(horizon_);
    }
    void DisplayMsgs::displayDrone(Eigen::SparseMatrix<double> &state){
       
        drone_msg.points.clear();
        drone_msg.header.frame_id = "map";
        drone_msg.header.stamp = ros::Time::now();
        drone_msg.type = visualization_msgs::Marker::SPHERE_LIST;
        // drone_msg.action = visualization_msgs::Marker::DELETE;
        drone_msg.id = 45;

        drone_msg.action = visualization_msgs::Marker::ADD;
        drone_msg.pose.orientation.x = 0.0;
        drone_msg.pose.orientation.y = 0.0;
        drone_msg.pose.orientation.z = 0.0;
        drone_msg.pose.orientation.w = 1.0;

        drone_msg.color.r = 0.6;
        drone_msg.color.g = 0.7;
        drone_msg.color.b = 0.6;
        drone_msg.color.a = 1;

        drone_msg.scale.x = 0.6;
        drone_msg.scale.y = 0.6;
        drone_msg.scale.z = 1;
  
        pt.x = state.coeffRef(0,0);
        pt.y = state.coeffRef(Model::numOrder,0);
        pt.z = state.coeffRef(2*Model::numOrder,0);
        drone_msg.points.push_back(pt);
        pt.x += state.coeffRef(1,0);
        pt.y += state.coeffRef(1+Model::numOrder,0);
        pt.z += state.coeffRef(1+2*Model::numOrder,0);
        drone_msg.points.push_back(pt);
        // drone_msg.header.stamp = ros::Time::now();
    }
    void DisplayMsgs::displayTheta(Eigen::SparseMatrix<double> &state,  Eigen::Vector3d pos, Eigen::Vector3d vel){
        double theta = state.coeffRef(Model::numState-Model::numOrder,0);
        theta_msg.points.clear();
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        theta_msg.points.push_back(pt);
        pt.x += vel(0);
        pt.y += vel(1);
        pt.z += vel(2);
        theta_msg.points.push_back(pt);      
    }
    void DisplayMsgs::displayPredict(Eigen::SparseMatrix<double> &statePredict){
        geometry_msgs::PoseStamped tmpPose;
        trajPred_msg.poses.clear();
        for (unsigned int i=0; i<horizon; ++i){
            // std::cout<< "============displayPredict===========" << i << " " << statePredict.coeffRef(0,i) << std::endl;
            tmpPose.pose.position.x = statePredict.coeffRef(0,i);
            // std::cout<< "============displayPredict===========" << i << " " << statePredict.coeffRef(Model::numOrder,i) << std::endl;
            tmpPose.pose.position.y = statePredict.coeffRef(Model::numOrder,i);
            // std::cout<< "============displayPredict===========" << i << " " << statePredict.coeffRef(2*Model::numOrder,i) << std::endl;
            tmpPose.pose.position.z = statePredict.coeffRef(2*Model::numOrder,i);
            trajPred_msg.poses.push_back(tmpPose);
        }
    }

    void DisplayMsgs::pubTunnels(ros::Publisher& pub){
        pub.publish(empty_poly_msg);
        pub.publish(tunnel_array_msg);
    }
}
