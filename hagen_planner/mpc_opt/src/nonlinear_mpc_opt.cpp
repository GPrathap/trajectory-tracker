#include "mpc_opt/nonlinear_mpc_opt.h"
#include <ros/ros.h>

namespace hagen_planner
{
  NonLinearMPCOpt::NonLinearMPCOpt()
  {
    
  }

  void NonLinearMPCOpt::solver_init(){
  }

  void NonLinearMPCOpt::getCollocationPoints(std::vector<double>& B, std::vector<std::vector<double>>& C, std::vector<double>& D){
    
    int d = collocation_degree;
    // Choose collocation points
    vector<double> tau_root = collocation_points(d, "legendre");
    tau_root.insert(tau_root.begin(), 0);

    // For all collocation points
    for(int j=0; j<d+1; ++j){
      // Construct Lagrange polynomials to get the polynomial basis at the collocation point
      Polynomial p = 1;
      for(int r=0; r<d+1; ++r){
        if(r!=j){
          p *= Polynomial(-tau_root[r],1)/(tau_root[j]-tau_root[r]);
        }
      }

      // Evaluate the polynomial at the final time to get the coefficients of the continuity equation
      D[j] = p(1.0);

      // Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
      Polynomial dp = p.derivative();
      for(int r=0; r<d+1; ++r){
        C[j][r] = dp(tau_root[r]);
      }

      Polynomial ip = p.anti_derivative();
      B[j] = ip(1.0);

    }
    return;
  }


  double NonLinearMPCOpt::getYawFromQuat(const geometry_msgs::Quaternion &data)
  {
      tf::Quaternion quat(data.x, data.y, data.z, data.w);
      tf::Matrix3x3 m(quat);
      double_t roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
  }

  void NonLinearMPCOpt::setEnvironment(const EDTEnvironment::Ptr& env){
    edt_env_ = env;
  }


  void NonLinearMPCOpt::init(ros::NodeHandle& nh){
    MPCOpt::init(nh);
  }

  tuple<double, SX, SX> NonLinearMPCOpt::shift(double T, double t0, SX x0, SX u, Function f){
      SX st = x0;
      SX con = u(Slice(0), Slice(0, u.size2())).T();
      SXDict f_in = {{"x", st}, {"u", con}};
      SXDict f_value = f(f_in);
      st = st + T*f_value["rhs"];
      x0 = st;
      t0 = t0 + T;
      SX u_rest = u(Slice(1, u.size1()), Slice(0, u.size2()));
      SX u_last = u(Slice(u.size1()-1, u.size1()), Slice(0, u.size2()));
      SX ui = vertcat(u_rest, u_last);
      tuple<double, SX, SX> next_state(t0, x0, ui);
      return next_state;
  }

}  // namespace hagen_planner
