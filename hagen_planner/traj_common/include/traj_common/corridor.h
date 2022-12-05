#ifndef PROJECT_CORRIDOR_H
#define PROJECT_CORRIDOR_H
#include <Eigen/Core>
#include <decom_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>

namespace hagen_planner{
    struct Poly{
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Vector3d> normals;
        std::vector<double> starter, ender;
    };

    struct PolyLine{
        Eigen::Matrix<double, 1, 6> line;
        std::vector<Eigen::Vector3d> points;  // intersecting point
        std::vector<int> intersecting_line_id;  // 
    };

    typedef Eigen::MatrixX4d Polyhedron1;

    class Corridor{
    public:
        std::vector<Poly> polys;
        std::vector<Eigen::MatrixXd> Cn;
        std::vector<Eigen::MatrixXd> dn;
        std::vector<Eigen::MatrixXd> dn_neg;

        // tmp variables:
        Polyhedron1 tunnel;
        double tunnelArea;
        Corridor()=default;
        Corridor(vec_E<Polyhedron<3>> polyhedrons);

        void FindPolygon(const Eigen::Vector3d position, const Eigen::Vector3d tangent_line, const int corridor_id);
    };

}//namespace hagen_planner
#endif
