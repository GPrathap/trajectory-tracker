#include "mpc_opt/linearized_mpc_opt.h"
#include <iostream>
#include <ros/package.h>
#include <ctime>

using namespace Eigen;
using namespace std;
namespace hagen_planner
{
    LinearizedMPCOpt::LinearizedMPCOpt()
    {
        
    }

    LinearizedMPCOpt::~LinearizedMPCOpt(){

    }

    void LinearizedMPCOpt::setEnvironment(const EDTEnvironment::Ptr& env){
        edt_env_ = env;
    }

    void LinearizedMPCOpt::init(ros::NodeHandle& nh){
        node_ = nh;
        MPCOpt::init(nh);
        node_.param("mpc_opt/corridor_constrains", corridor_constrains, 1);
    }

    void LinearizedMPCOpt::solver_init(){
        
    }

    void LinearizedMPCOpt::printMatrices(){
        cout << "A:\n" << A << endl;
    }

    void LinearizedMPCOpt::castMPCToQPConstraintMatrix(Eigen::MatrixXd &dynamicMatrix
                                                              , Eigen::MatrixXd &controlMatrix, Eigen::SparseMatrix<double> &constraintMatrix, Eigen::MatrixXd& a1){
       
      constraintMatrix.resize(numState*(prediction_horizon+1)  + numState*(prediction_horizon+1) + (prediction_horizon+1)*a1.rows()+ numInput * prediction_horizon, numState*(prediction_horizon+1) + numInput * prediction_horizon);
      // populate linear constraint matrix
      for(int i = 0; i<numState*(prediction_horizon+1); i++){
          constraintMatrix.insert(i,i) = -1;
      }

      for(int i = 0; i < numState*(prediction_horizon+1); i++){
        for(int j = 0; j < numState*(prediction_horizon+1); j++){
          float value = dynamicMatrix(i,j);
          if(value != 0){
              constraintMatrix.insert(i, j) = value;
          }
        }
      }

      for(int j = 0; j < numState*(prediction_horizon+1); j++){
        for(int i = numState*(prediction_horizon+1); i < numState*(prediction_horizon+1) + numInput*(prediction_horizon); i++){
          float value = controlMatrix(j,i-numState*(prediction_horizon+1));
          if(value != 0){
              constraintMatrix.insert(j, i) = value;
          }
        }
      }

      for(int i = 0; i<numState*(prediction_horizon+1) + numInput*prediction_horizon; i++){
        constraintMatrix.insert(i+(prediction_horizon+1)*numState,i) = 1;
      }

      int index_i = numState*(prediction_horizon+1)  + numState*(prediction_horizon+1) + numInput * prediction_horizon;
      int index_j = numState*(prediction_horizon+1)  + numState*(prediction_horizon+1) + numState*a1.rows()+ numInput * prediction_horizon;
      int col_index = 0;
      int step_ = 0;
      if(a1.rows()>0){
        for(int j=0; j< prediction_horizon+1; j++){
           for(int k = 0; k < a1.rows(); k++){
            for(int l = 0; l < a1.cols(); l++){
                float value = a1(k, l);
                int row_ = index_i + k + step_;
                int col_ = col_index + l ;
                std::cout<< row_ << "," << col_ << std::endl;
                if(value != 0){
                    constraintMatrix.insert(row_, col_) = value;
                }
            }
          }
          col_index += numState;
          step_ += a1.rows();
        }
      }
    }

    
    void LinearizedMPCOpt::castMPCToQPConstraintMatrix(Eigen::MatrixXd &dynamicMatrix
        , Eigen::MatrixXd &controlMatrix, Eigen::SparseMatrix<c_float> &constraintMatrix){
       
      constraintMatrix.resize(numState*(prediction_horizon+1)  + numState*(prediction_horizon+1) + numInput * prediction_horizon, numState*(prediction_horizon+1) + numInput * prediction_horizon);
      // populate linear constraint matrix
      for(int i = 0; i<numState*(prediction_horizon+1); i++){
          constraintMatrix.insert(i,i) = -1;
      }

      for(int i = 0; i < numState*(prediction_horizon+1); i++){
        for(int j = 0; j < numState*(prediction_horizon+1); j++){
          float value = dynamicMatrix(i,j);
          if(value != 0){
              constraintMatrix.insert(i, j) = value;
          }
        }
      }

      for(int j = 0; j < numState*(prediction_horizon+1); j++){
        for(int i = numState*(prediction_horizon+1); i < numState*(prediction_horizon+1) + numInput*(prediction_horizon); i++){
          float value = controlMatrix(j,i-numState*(prediction_horizon+1));
          if(value != 0){
              constraintMatrix.insert(j, i) = value;
          }
        }
      }

      for(int i = 0; i<numState*(prediction_horizon+1) + numInput*prediction_horizon; i++){
        constraintMatrix.insert(i+(prediction_horizon+1)*numState,i) = 1;
      }
   }

   void LinearizedMPCOpt::castMPCToQPConstraintVectors(Eigen::MatrixXd &xMax, Eigen::MatrixXd &xMin, Eigen::MatrixXd &uMax, Eigen::MatrixXd &uMin,
        Eigen::MatrixXd &x0, Eigen::MatrixXd& uq, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound, Eigen::MatrixXd& b1){
          
        // evaluate the lower and the upper inequality vectors
        Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1) +  numInput * prediction_horizon + b1.rows()*(prediction_horizon+1), 1);
        Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1) +  numInput * prediction_horizon + b1.rows()*(prediction_horizon+1), 1);

        for(int i=0; i<prediction_horizon+1; i++){
            lowerInequality.block(numState*i,0,numState,1) = xMin;
            upperInequality.block(numState*i,0,numState,1) = xMax;
        }
        for(int i=0; i<prediction_horizon; i++){
            lowerInequality.block(numInput * i + numState * (prediction_horizon + 1), 0, numInput, 1) = uMin;
            upperInequality.block(numInput * i + numState * (prediction_horizon + 1), 0, numInput, 1) = uMax;
        }
        if(b1.rows()>0){
          Eigen::MatrixXd b_lower(b1.rows(), 1);
          for(int v=0; v< b1.rows(); v++){
            b_lower(v,0) = -100000;
          }
          for(int i=0; i<prediction_horizon+1; i++){
              lowerInequality.block(i*b1.rows() + numInput * prediction_horizon + numState * (prediction_horizon + 1), 0, b1.rows(),1) = b_lower;
              upperInequality.block(i*b1.rows() + numInput * prediction_horizon + numState * (prediction_horizon + 1), 0, b1.rows(),1) = b1;
          }
        }

        Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1),1 );
        Eigen::VectorXd upperEquality;
        lowerEquality.block(0,0,numState,1) = -x0;
        lowerEquality.block(numState, 0, numState*(prediction_horizon), 1) = uq;
        upperEquality = lowerEquality;
        lowerBound = Eigen::MatrixXd::Zero(2*numState*(prediction_horizon+1) +  numInput*prediction_horizon + b1.rows()*(prediction_horizon+1), 1);
        lowerBound << lowerEquality, lowerInequality;

        upperBound = Eigen::MatrixXd::Zero(2*numState*(prediction_horizon+1) +  numInput*prediction_horizon + b1.rows()*(prediction_horizon+1), 1);
        upperBound << upperEquality, upperInequality;
    }


    void LinearizedMPCOpt::castMPCToQPConstraintVectors(Eigen::MatrixXd &xMax
          , Eigen::MatrixXd &xMin,
            Eigen::MatrixXd &uMax, Eigen::MatrixXd &uMin,
            Eigen::MatrixXd &x0, Eigen::MatrixXd& uq,
            Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound){
        // evaluate the lower and the upper inequality vectors
        Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1) +  numInput * prediction_horizon, 1);
        Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1) +  numInput * prediction_horizon, 1);
        for(int i=0; i<prediction_horizon+1; i++){
            lowerInequality.block(numState*i,0,numState,1) = xMin;
            upperInequality.block(numState*i,0,numState,1) = xMax;
        }
        for(int i=0; i<prediction_horizon; i++){
            lowerInequality.block(numInput * i + numState * (prediction_horizon + 1), 0, numInput, 1) = uMin;
            upperInequality.block(numInput * i + numState * (prediction_horizon + 1), 0, numInput, 1) = uMax;
        }

        // evaluate the lower and the upper equality vectors
        Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(numState*(prediction_horizon+1),1 );
        Eigen::VectorXd upperEquality;
        lowerEquality.block(0,0,numState,1) = -x0;
        lowerEquality.block(numState, 0, numState*(prediction_horizon), 1) = uq;
        upperEquality = lowerEquality;
        // merge inequality and equality vectors
        lowerBound = Eigen::MatrixXd::Zero(2*numState*(prediction_horizon+1) +  numInput*prediction_horizon,1 );
        lowerBound << lowerEquality, lowerInequality;

        upperBound = Eigen::MatrixXd::Zero(2*numState*(prediction_horizon+1) +  numInput*prediction_horizon,1 );
        upperBound << upperEquality, upperInequality;
    }

    void LinearizedMPCOpt::tile(int n, Eigen::MatrixXd& in_, Eigen::MatrixXd& cost_, Eigen::MatrixXd& out_){
      out_.resize(in_.rows()*n, 1);
      out_.setZero();
      for(int i=0; i<in_.rows()*n; i+=in_.rows()){
        for(int j=0; j<in_.rows(); j++){
          out_(i+j, 0) = in_(j,0)*cost_(i+j, 0);
        }
      }
    }

    void LinearizedMPCOpt::save_model(Eigen::SparseMatrix<c_float> inputs, std::string name_){    
    std::string path_name = "/home/geesara/catkin_workspace/src/planner/Hagen-Planner/hagen_planner/state_machine/data/" + name_ + "_" + std::to_string(inputs.rows()) + "_" + std::to_string(inputs.cols()) + "_.npy";
      std::vector<double> data_;
      for (int i = 0; i < inputs.rows(); i++)
      {
        for (int j = 0; j < inputs.cols(); j++)
        {
          data_.push_back(inputs.coeffRef(i,j));
        }
      }
      cnpy::npy_save(path_name, &data_[0], {data_.size()}, "w");
  }

  void LinearizedMPCOpt::save_model(Eigen::MatrixXd inputs, std::string name_){    
    std::string path_name = "/home/geesara/catkin_workspace/src/planner/Hagen-Planner/hagen_planner/state_machine/data/" + name_ + "_" + std::to_string(inputs.rows()) + "_" + std::to_string(inputs.cols()) + "_.npy";
      std::vector<double> data_;
      for (int i = 0; i < inputs.rows(); i++)
      {
        for (int j = 0; j < inputs.cols(); j++)
        {
          data_.push_back(inputs(i,j));
        }
      }
      cnpy::npy_save(path_name, &data_[0], {data_.size()}, "w");
  }

  void LinearizedMPCOpt::save_model(Eigen::VectorXd inputs, std::string name_){    
    std::string path_name = "/home/geesara/catkin_workspace/src/planner/Hagen-Planner/hagen_planner/state_machine/data/" + name_ + "_" + std::to_string(inputs.rows()) + "_" + std::to_string(inputs.cols()) + "_.npy";
      std::vector<double> data_;
      for (int i = 0; i < inputs.size(); i++)
      {
        data_.push_back(inputs(i));
      }
      cnpy::npy_save(path_name, &data_[0], {data_.size()}, "w");
  }
}  // namespace hagen_planner

