#ifndef _PLANNING_SAVING_H_
#define _PLANNING_SAVING_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <spline_opt/non_uniform_bspline.h>
#include <casadi/casadi.hpp>
#include <cnpy.h>
#include <fstream>


using std::vector;
using namespace casadi;
namespace hagen_planner
{
class PlanningSaving
{
private:
  enum DRAW_ID
  {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    PREDICTION = 500
  };

  /* data */
  ros::NodeHandle node;

public:
  PlanningSaving(/* args */)
  {
  }
  ~PlanningSaving()
  {
  }

  PlanningSaving(ros::NodeHandle& nh);

  std::string root_dir ="/";
  std::ofstream outfile_training;
  typedef std::shared_ptr<PlanningSaving> Ptr;

  void save_vector(vector<double> trees, std::string file_name, int index);
  void save_dm(DM trees, std::string file_name, int index);
  void save_double_vector(vector<vector<double>> trees_, std::string file_name, int index);
  void save_training_set(std::vector<double> state_in, std::vector<double> state_out);

};
}  // namespace hagen_planner
#endif