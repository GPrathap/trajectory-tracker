#include "mpc_opt/mpc_solver.h"
#include <iostream>
#include <ros/package.h>
#include <ctime>

using namespace Eigen;
using namespace std;
namespace hagen_planner{
    MpcSolver::MpcSolver()
    {
        
    }
    
    void MpcSolver::solverInit(hagen_planner::ParamPasser::Ptr& passer){

        Eigen::MatrixXd horizon_(1,1);
        passer->passing_vector("horizon", horizon_);
        horizon = horizon_(0,0);
        Eigen::MatrixXd corridor_constrains_(1,1);
        passer->passing_vector("corridor_constrains", corridor_constrains_);
        corridorConstrains = corridor_constrains_(0,0);
        Eigen::MatrixXd thetaMax_(1,1);
        passer->passing_vector("thetaMax", thetaMax_);
        map.thetaMax = thetaMax_(0,0);
        Eigen::MatrixXd qVTheta_(1,1);
        passer->passing_vector("qVTheta", qVTheta_);
        qVTheta = qVTheta_(0,0);
        Eigen::MatrixXd stateUpper_(1, numState);
        passer->passing_vector("stateUpper", stateUpper_);
        stateUpper = stateUpper_.row(0);
        // std::cout<< "upper: " << stateUpper_ << std::endl;
        Eigen::MatrixXd stateLower_(1,numState);
        passer->passing_vector("stateLower", stateLower_);
        stateLower = stateLower_.row(0);

        // std::cout<< "lower: " << stateLower_ << std::endl;

        Eigen::MatrixXd inputUpper_(1,numInput);
        passer->passing_vector("inputUpper", inputUpper_);
        inputUpper = inputUpper_.row(0);

        Eigen::MatrixXd inputLower_(1,numInput);
        passer->passing_vector("inputLower", inputLower_);
        inputLower = inputLower_.row(0);

        initStatus = true;

        A.resize(0,0);
        b.resize(0,0);
        xl.resize(horizon*numState, 1);
        xu.resize(horizon*numState, 1);
        ul.resize(horizon*numInput, 1);
        uu.resize(horizon*numInput, 1);
        P.resize(horizon*(numState+numInput)+numState, horizon*(numState+numInput)+numState);
        P.setIdentity();
        
        inputPredict.resize(numInput, horizon);
        statePredict.resize(numState, horizon);
        // initialize xl,xu,ul,uu: 
        Eigen::SparseMatrix<double> stateUs(numState,1);
        Eigen::SparseMatrix<double> stateLs(numState,1);
        Eigen::SparseMatrix<double> inputUs(numInput,1);
        Eigen::SparseMatrix<double> inputLs(numInput,1);
        for (int i=0; i<numState; ++i){
            stateUs.coeffRef(i,0) = stateUpper[i];
            stateLs.coeffRef(i,0) = stateLower[i];
        }
        for (int i=0; i<numInput; ++i){
            inputUs.coeffRef(i,0) = inputUpper[i];
            inputLs.coeffRef(i,0) = inputLower[i];
        }

        Eigen::SparseMatrix<double> eye_n(model.numState, model.numState);
        eye_n.setIdentity();
        eye_n = -1*eye_n;
        Eigen::SparseMatrix<double> eye_h(horizon+1, horizon+1);
        eye_h.setIdentity();
        Eigen::SparseMatrix<double> eye_h_o(horizon+1, horizon+1);
        for(int j=1; j<eye_h.rows(); j++){
            eye_h_o.coeffRef(j,j-1) = 1;
        }
        Ax = Eigen::kroneckerProduct(eye_h, eye_n).eval() 
                                + Eigen::kroneckerProduct(eye_h_o, model.Ad).eval();

        Eigen::SparseMatrix<double> b_i(horizon+1, horizon);
        for(int i=0;i<horizon; i++){
            b_i.coeffRef(i+1, i) = 1;
        }
        Bx = Eigen::kroneckerProduct(b_i, model.Bd).eval();

        Aineq.resize((horizon+1)*numState + horizon*numInput, (horizon+1)*numState + horizon*numInput);
        Aineq.setIdentity();

        Eigen::SparseMatrix<double> ones_i(horizon+1, 1);
        for(int i=0; i<horizon+1; i++){
            ones_i.coeffRef(i,0) = 1.0;
        }

        Eigen::SparseMatrix<double> ones_(horizon, 1);
        for(int i=0; i<horizon; i++){
                ones_.coeffRef(i,0) = 1.0;
        }

    }

    MpcSolver::~MpcSolver(){
    }

    void MpcSolver::calculateCost(double theta, int horizon_, Eigen::Vector3d pos, Eigen::Vector3d vel){
        double theta_ = std::fmod(theta, map.thetaMax);
        double x_v = pos(0);
        double y_v = pos(1);
        double z_v = pos(2);
        double dx_v__dtheta = vel(0);
        double dy_v__dtheta = vel(1);
        double dz_v__dtheta = vel(2);
        double r_x = x_v - dx_v__dtheta * theta;
        double r_y = y_v - dy_v__dtheta * theta;
        double r_z = z_v - dz_v__dtheta * theta;
        Eigen::SparseMatrix<double> grad_x(numState, 1);
        Eigen::SparseMatrix<double> grad_y(numState, 1);
        Eigen::SparseMatrix<double> grad_z(numState, 1);
        grad_x.coeffRef(0,0) = 1;
        grad_y.coeffRef(Model::numOrder,0) = 1;
        grad_z.coeffRef(2*Model::numOrder,0) = 1;
        grad_x.coeffRef(numState-Model::numOrder,0) = -dx_v__dtheta;
        grad_y.coeffRef(numState-Model::numOrder,0) = -dy_v__dtheta;
        grad_z.coeffRef(numState-Model::numOrder,0) = -dz_v__dtheta;
        Qn = grad_x * Eigen::SparseMatrix<double>(grad_x.transpose()) + 
             grad_y * Eigen::SparseMatrix<double>(grad_y.transpose()) + 
             grad_z * Eigen::SparseMatrix<double>(grad_z.transpose());
        qn = -r_x*grad_x - r_y*grad_y - r_z*grad_z;
        qn.coeffRef(numState-Model::numOrder+1,0) = -qVTheta;
    }

    void MpcSolver::calculateConstrains(Eigen::SparseMatrix<double> &stateTmp, int horizon_, Eigen::Vector3d position
                        , Eigen::Vector3d tangentLine){
        assert(stateTmp.rows() == numState);
        double theta_ = stateTmp.coeffRef(numState-Model::numOrder,0);
        double theta = std::fmod(theta_+map.thetaMax, map.thetaMax);
        int numConstrains = 0;
        Polyhedron1 chosenPoly;
        if (theta < 1e-6){
            theta = 1e-6;
        }
        // find the max Polyhedron intersecting theta face
        double maxArea = 0;
        
        corridor_ref.FindPolygon(position, tangentLine, 0);
        if(maxArea < corridor_ref.tunnelArea){
            maxArea = corridor_ref.tunnelArea;
            chosenPoly = corridor_ref.tunnel;
        }
        Polyhedron<3> poly_tunnel;
        createTunnel(position, tangentLine, poly_tunnel, horizon_);
        Cn.resize(numConstrains, numState);
        Cnb.resize(numConstrains, 1);
        for (int i=0; i<numConstrains; ++i){
            for (int j=0; j<Model::numDimention; ++j){
                Cn.coeffRef(i,Model::numOrder*j) = chosenPoly(i,j);
            }
            Cnb.coeffRef(i,0) = chosenPoly(i,3);
        }
    }

    void MpcSolver::createTunnel(const Eigen::Vector3d& position
        , const Eigen::Vector3d& tangent_line_, Polyhedron<3>& poly_tunnel, int index){
        
        if(index ==0){
            _filtered_visu.clear();
        }
        for ( int j = 0; j < corridor_ref.tunnel.rows(); j++ ){
            Eigen::Vector3d normal = corridor_ref.tunnel.block(j, 0, 1, 3).transpose().normalized();
            double D = corridor_ref.tunnel(j,3) / corridor_ref.tunnel.block(j, 0, 1, 3).transpose().norm();
            Eigen::Vector3d point_on_plane = - normal * D;
            Eigen::Matrix<double, 3, 1> p_(point_on_plane(0), point_on_plane(1), point_on_plane(2));
            Eigen::Matrix<double, 3, 1> n_(normal(0), normal(1), normal(2));
            Hyperplane3D hyper_planeyu(p_, n_);
            poly_tunnel.add(hyper_planeyu);
        }
        Eigen::Vector3d extra_point, extra_normal = tangent_line_.normalized();
        extra_point = position + extra_normal * 1e-1;
        const Eigen::Matrix<double, 3, 1> p_(extra_point(0), extra_point(1), extra_point(2));
        const Eigen::Matrix<double, 3, 1> n_(extra_normal(0), extra_normal(1), extra_normal(2));
        Hyperplane3D hyper_planer(p_, n_);
        poly_tunnel.add(hyper_planer);
        extra_point = position - extra_normal * 1e-1;
        const Eigen::Matrix<double, 3, 1> p1_(extra_point(0), extra_point(1), extra_point(2));
        const Eigen::Matrix<double, 3, 1> n1_(-extra_normal(0), -extra_normal(1), -extra_normal(2));
        Hyperplane3D hyper_plane1(p1_, n1_);
        poly_tunnel.add(hyper_plane1);
        _filtered_visu.push_back(poly_tunnel);
        return;
    }

    int MpcSolver::solveMpcQp(Eigen::SparseMatrix<double> &stateTmp, Eigen::SparseMatrix<double> &endTmp,std::vector<Eigen::Vector3d>& projected_path
                                , vec_E<Polyhedron<3>>& polyhedrons){
    };
    
    void MpcSolver::printMatrices(){
        cout << "Q:\n" << Q << endl;
        cout << "c:\n" << c << endl;
        cout << "A:\n" << A << endl;
        cout << "b:\n" << b << endl;
        cout << "C:\n" << C << endl;
        cout << "clow:\n" << clow << endl;
        cout << "cupp:\n" << cupp << endl;
        cout << "ul:\n" << ul << endl;
        cout << "uu:\n" << uu << endl;
    }
} // namespace hagen_planner
