#ifndef PROJECT_MPC_SOLVER_H
#define PROJECT_MPC_SOLVER_H
#include <map_building_opt/edt_environment.h>
#include <ros/ros.h>
#include <traj_common/param_passer.h>
#include <decom_rviz_plugins/data_ros_utils.h>
#include <decom_rviz_plugins/ellipsoid_decomp.h>
#include <decom_rviz_plugins/polyhedron.h>
#include <traj_common/corridor.h>
#include <motion_model/model.h>
#include <traj_common/display_msgs.h>
#include <unsupported/Eigen/KroneckerProduct>

namespace hagen_planner{
    class MpcSolver
    {
    private:
        int corridorConstrains; // set to 1 if considering corridor constrains
        double qVTheta; //cost for v_theta : set in config/qVTheta.yaml
        Eigen::VectorXd stateUpper;
        Eigen::VectorXd stateLower;
        Eigen::VectorXd inputUpper;
        Eigen::VectorXd inputLower;
        Eigen::SparseMatrix<double> Inu;
        int numState = Model::numState; // ns
        int numInput = Model::numInput; // ni
        /**
         * Q = BB'(Qk)BB
         * c = BB'(Qk'AA x0 + qk)
         * X = BB * U + AA * x0
         * **/
        // matrices for qpsolver: 
        Eigen::SparseMatrix<double> P ; // N*ns * N*ns
        Eigen::SparseMatrix<double> q ;
        Eigen::SparseMatrix<double> Ax;
        Eigen::SparseMatrix<double> Bx;
        Eigen::SparseMatrix<double> Aineq;
        Eigen::SparseMatrix<double> lineq;
        Eigen::SparseMatrix<double> uineq;
        Eigen::SparseMatrix<double> Aeq;
        Eigen::SparseMatrix<double> Q ; // N*ns * N*ns
        Eigen::SparseMatrix<double> c ; // N*ni * 1
        Eigen::SparseMatrix<double> A ;
        Eigen::SparseMatrix<double> b ;
        Eigen::SparseMatrix<double> C ;
        Eigen::SparseMatrix<double> clow;
        Eigen::SparseMatrix<double> cupp;
        // xlow : ul; xupp : uu;
        // const matrices: 
        Eigen::SparseMatrix<double> AA; // N*ns * ns
        Eigen::SparseMatrix<double> BB; // N*ns * N*ni
        Eigen::SparseMatrix<double> AAT;//AA.transpose()
        Eigen::SparseMatrix<double> BBT;//BB.transpose()
        Eigen::SparseMatrix<double> In; // ns   *   ns : tiny diag
        Eigen::SparseMatrix<double> xu; // N*ns *    1 : state upper
        Eigen::SparseMatrix<double> xl; // N*ns *    1 : state lower
        Eigen::SparseMatrix<double> uu; // N*ns *    1 : input upper
        Eigen::SparseMatrix<double> ul; // N*ns *    1 : input lower
        // temp matrices for each horizon: 
        Eigen::SparseMatrix<double> Qn; // ns   *   ns
        Eigen::SparseMatrix<double> qn; // ns   *    1
        Eigen::SparseMatrix<double> Cn; // n?   *   ns
        Eigen::SparseMatrix<double> Cnb;// n?   *    1 
        Eigen::SparseMatrix<double> Qk; // N*ns * N*ns : blkdiag(Q1,Q2,Q3,...)
        Eigen::SparseMatrix<double> qk; // N*ns *    1 : [q1;q2;q3;...]
        Eigen::SparseMatrix<double> Ck; // n??  *   ns : blkdiag(C1,C2,C3,...)
        Eigen::SparseMatrix<double> Ckb;// n??  *    1

        // osqp::OSQPInterface osqpInterface; // qpSolver interface
// 
    public:
        bool initStatus;
        int horizon; // N : set in config/qVTheta.yaml
        // hagen_planner::Map map;
        hagen_planner::Model model;
        // solution: 
        Eigen::SparseMatrix<double> inputPredict;
        Eigen::SparseMatrix<double> statePredict;
        // DisplayMsgs * displayPtr;
        hagen_planner::EDTEnvironment map;
        Corridor corridor_ref;
        DisplayMsgs display_msgs;
        
        MpcSolver();
        void solverInit(hagen_planner::ParamPasser::Ptr& passer);
        ~MpcSolver();
        void calculateCost(double theta, int horizon_, Eigen::Vector3d pos, Eigen::Vector3d vel);
        void calculateConstrains(Eigen::SparseMatrix<double> &stateTmp, int horizon_
                                                    , Eigen::Vector3d position, Eigen::Vector3d tangentLine);
        int solveMpcQp(Eigen::SparseMatrix<double> &stateTmp, Eigen::SparseMatrix<double> &end_state, std::vector<Eigen::Vector3d>& projected_path
                                                            ,  vec_E<Polyhedron<3>>& polyhedrons);
        void createTunnel(const Eigen::Vector3d& position
        , const Eigen::Vector3d& tangent_line_, Polyhedron<3>& poly_tunnel, int index);
        void printMatrices(); // just for debug
        vec_E<Polyhedron<3>> _filtered_visu;
    };
}//namespace hagen_planner

#endif