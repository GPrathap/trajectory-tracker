#ifndef PROJECT_MODEL_H
#define PROJECT_MODEL_H

#include <Eigen/Sparse>

namespace hagen_planner{

// class Model
    class Model{
        public:

        static constexpr unsigned long freq = 20; // frequency of discretization
        static constexpr double ts = 1.0/freq;
        static constexpr int numOrder = 3;        // number of orders
        static constexpr int numDimention = 3;    // number of dimentions
        static constexpr int numState = 3; // number of states
        static constexpr int numInput = 3; // number of inputs

        Eigen::MatrixXd Ad;
        Eigen::MatrixXd Bd;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd QN;
        Eigen::MatrixXd R;

        Model (double T = ts);
    };

}//namespace hagen_planner -> flight_tunnel



#endif //PROJECT_MODEL_H
