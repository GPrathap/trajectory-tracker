#ifndef LINEARIZED_MPC_OPT
#define LINEARIZED_MPC_OPT
#include <motion_model/model.h>
#include <traj_common/corridor.h>
#include <traj_common/display_msgs.h>
#include <map_building_opt/edt_environment.h>
#include <traj_common/param_passer.h>
#include <ros/ros.h>
#include <decom_rviz_plugins/data_ros_utils.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <decom_rviz_plugins/polyhedron.h>
#include <unsupported/Eigen/KroneckerProduct>
#include "mpc_opt.h"
#include <motion_model/spatial_bicycle_models.h>

namespace hagen_planner{
    class LinearizedMPCOpt : public MPCOpt
    {
    private:
    
    public:
        
        int corridorConstrains;
        double qVTheta;
        Eigen::VectorXd stateUpper;
        Eigen::VectorXd stateLower;
        Eigen::VectorXd inputUpper;
        Eigen::VectorXd inputLower;
        int numState = BicycleModel::numState; // ns
        int numInput = BicycleModel::numInput; // ni
       
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
        Eigen::SparseMatrix<double> ur; // N*ns *    1 : state upper
        Eigen::SparseMatrix<double> xr; // N*ns *    1 : state lower
        Eigen::SparseMatrix<double> uu; // N*ns *    1 : input upper
        Eigen::SparseMatrix<double> ul; // N*ns *    1 : input lower
        // Eigen::SparseMatrix<double> uq; // N*ns *    1 : input lower
        Eigen::MatrixXd uq; // N*ns *    1 : input lower

        Eigen::SparseMatrix<double> Cn; // n?   *   ns
        Eigen::SparseMatrix<double> Cnb;// n?   *    1 
        Eigen::SparseMatrix<double> Ck; // n??  *   ns : blkdiag(C1,C2,C3,...)
        Eigen::SparseMatrix<double> Ckb;// n??  *    1
        Eigen::SparseMatrix<double>umax_dyn;
        // osqp::OSQPInterface osqpInterface; // qpSolver interface
        bool initStatus;
        // hagen_planner::Map map;
       
        // solution: 
        Eigen::SparseMatrix<double> inputPredict;
        Eigen::SparseMatrix<double> statePredict;
        // DisplayMsgs * displayPtr;
        EDTEnvironment map;
        Corridor corridor_ref;
        DisplayMsgs display_msgs;
        vec_E<Polyhedron<3>> _filtered_visu;
        int corridor_constrains = 1;

        int set_solver_ = 0;
        int infeasibility_counter = 0;
        bool set_init_solver = false;

        Eigen::MatrixXd u_max;
        Eigen::MatrixXd u_min;
        Eigen::MatrixXd x_max, x_min;

        Eigen::MatrixXd current_control;
        Eigen::MatrixXd kappa_pred;
        Eigen::MatrixXd predicted_states;

        LinearizedMPCOpt();
        ~LinearizedMPCOpt();
        void init(ros::NodeHandle& nh);
        void solver_init();
        void printMatrices(); 
        void setEnvironment(const EDTEnvironment::Ptr& env);
        void castMPCToQPConstraintMatrix(Eigen::MatrixXd &dynamicMatrix
                        , Eigen::MatrixXd &controlMatrix, Eigen::SparseMatrix<c_float> &constraintMatrix);
        void castMPCToQPConstraintMatrix(Eigen::MatrixXd &dynamicMatrix
                                                              , Eigen::MatrixXd &controlMatrix, Eigen::SparseMatrix<double> &constraintMatrix, Eigen::MatrixXd& a1);
        void castMPCToQPConstraintVectors(Eigen::MatrixXd &xMax
          , Eigen::MatrixXd &xMin,
            Eigen::MatrixXd &uMax, Eigen::MatrixXd &uMin,
            Eigen::MatrixXd &x0, Eigen::MatrixXd& uq,
            Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
        void castMPCToQPConstraintVectors(Eigen::MatrixXd &xMax, Eigen::MatrixXd &xMin, Eigen::MatrixXd &uMax, Eigen::MatrixXd &uMin,
                                                Eigen::MatrixXd &x0, Eigen::MatrixXd& uq, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound, Eigen::MatrixXd& b1);
        void tile(int n, Eigen::MatrixXd& in_, Eigen::MatrixXd& cost_, Eigen::MatrixXd& out_);

        void save_model(Eigen::SparseMatrix<double> inputs, std::string name_);
        void save_model(Eigen::MatrixXd inputs, std::string name_);
        void save_model(Eigen::VectorXd inputs, std::string name_);
        
    };
}//namespace hagen_planner

#endif