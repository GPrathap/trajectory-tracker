#ifndef LINEAR_MPC_OPT
#define LINEAR_MPC_OPT
#include "model.h"
#include "corridor.h"
#include <map_building_opt/edt_environment.h>

#include <ros/ros.h>
#include <traj_common/param_passer.h>
#include <decom_rviz_plugins/data_ros_utils.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <decom_rviz_plugins/polyhedron.h>
#include "display_msgs.h"
#include <unsupported/Eigen/KroneckerProduct>
#include "mpc_opt.h"

namespace hagen_planner{
    class LinearMPCOpt : public MPCOpt
    {
    private:
    
    public:
        
        int corridorConstrains;
        double qVTheta;
        Eigen::VectorXd stateUpper;
        Eigen::VectorXd stateLower;
        Eigen::VectorXd inputUpper;
        Eigen::VectorXd inputLower;
        int numState = Model::numState; // ns
        int numInput = Model::numInput; // ni
       
        // matrices for qpsolver: 
        Eigen::SparseMatrix<double> P ; // N*ns * N*ns
        Eigen::SparseMatrix<double> q ;
        Eigen::SparseMatrix<double> Ax;
        Eigen::SparseMatrix<double> Bx;
        Eigen::SparseMatrix<double> Aineq;
        Eigen::SparseMatrix<double> lineq;
        Eigen::SparseMatrix<double> uineq;
        Eigen::SparseMatrix<double> Aeq;
        Eigen::SparseMatrix<double> A ;

        Eigen::SparseMatrix<double> xu; // N*ns *    1 : state upper
        Eigen::SparseMatrix<double> xl; // N*ns *    1 : state lower
        Eigen::SparseMatrix<double> uu; // N*ns *    1 : input upper
        Eigen::SparseMatrix<double> ul; // N*ns *    1 : input lower

        Eigen::SparseMatrix<double> Cn; // n?   *   ns
        Eigen::SparseMatrix<double> Cnb;// n?   *    1 
        Eigen::SparseMatrix<double> Ck; // n??  *   ns : blkdiag(C1,C2,C3,...)
        Eigen::SparseMatrix<double> Ckb;// n??  *    1

        // osqp::OSQPInterface osqpInterface; // qpSolver interface
        bool initStatus;
        // hagen_planner::Map map;
        Model model;
        // solution: 
        Eigen::SparseMatrix<double> inputPredict;
        Eigen::SparseMatrix<double> statePredict;
        // DisplayMsgs * displayPtr;
        EDTEnvironment map;
        Corridor corridor_ref;
        DisplayMsgs display_msgs;
        vec_E<Polyhedron<3>> _filtered_visu;
        int corridor_constrains = 1;

        LinearMPCOpt();
        ~LinearMPCOpt();
        void init(ros::NodeHandle& nh);
        void solver_init();
        void printMatrices(); 
        void setEnvironment(const EDTEnvironment::Ptr& env);
        
    };
}//namespace hagen_planner

#endif