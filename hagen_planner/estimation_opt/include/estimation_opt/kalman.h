#ifndef _KALMAN_FILTER_
#define _KALMAN_FILTER_

#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

namespace hagen_planner
{
  class KalmanFilter {

  public:

    /**
    * Create a Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   B - Control dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P
    );

    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P
    );

    KalmanFilter();
    void init();
    void init(double t0, const Eigen::VectorXd& x0);
    void update(const Eigen::VectorXd& y);
    void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);
    void update(const Eigen::VectorXd& y, const Eigen::MatrixXd B, const Eigen::VectorXd u);
    Eigen::VectorXd state() { return x_hat; };
    double time() { return t; };

  private:
    Eigen::MatrixXd A, B, C, Q, R, P, K, P0;
    int m, n;
    double t0, t;
    double dt;
    bool initialized;
    Eigen::MatrixXd I;
    Eigen::VectorXd x_hat, x_hat_new;
  };
}

#endif
