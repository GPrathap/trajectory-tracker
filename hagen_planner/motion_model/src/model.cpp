#include <motion_model/model.h>
#include <iostream>

using namespace Eigen;
using namespace std;
namespace hagen_planner{
    Model::Model (double T){
        Ad = Eigen::MatrixXd::Identity(numState, numState);
        Bd = Eigen::MatrixXd::Identity(numState, numState);
        Q = Eigen::MatrixXd::Zero(numState, numState);
        R = Eigen::MatrixXd::Zero(numInput, numInput);
        QN = Eigen::MatrixXd::Zero(numInput, numInput);
        std::vector<Eigen::Triplet<double>> triplets;
        for (int i=0; i<numState; ++i){
        //    Ad.coeffRef(i,i) = 1.0;
        //    Bd.coeffRef(i,i) = 1.0;
           Q(i,i) = 1.0;
           R(i,i) = 1.0;
           QN(i,i) = 1.0;
        }
    }

}// namespace hagen_planner
