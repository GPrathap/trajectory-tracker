#include "mpc_opt/linear_mpc_opt.h"
#include <iostream>
#include <ros/package.h>
#include <ctime>

using namespace Eigen;
using namespace std;
namespace hagen_planner
{
    LinearMPCOpt::LinearMPCOpt()
    {
    }

    LinearMPCOpt::~LinearMPCOpt(){
    }

    void LinearMPCOpt::setEnvironment(const EDTEnvironment::Ptr& env){
        edt_env_ = env;
    }

    void LinearMPCOpt::init(ros::NodeHandle& nh){
        node_ = nh;
        MPCOpt::init(nh);
        node_.param("mpc_opt/corridor_constrains", corridor_constrains, 1);
    }

    void LinearMPCOpt::solver_init(){

    }

    void LinearMPCOpt::printMatrices(){
        cout << "A:\n" << A << endl;
    }

}  // namespace hagen_planner

