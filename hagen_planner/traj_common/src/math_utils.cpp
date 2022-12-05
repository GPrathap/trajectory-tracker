#include <traj_common/math_utils.h>

using std::cout;
using std::endl;
namespace hagen_planner
{

std::vector<Eigen::Vector3d> MathUtils::next_poses(Eigen::VectorXd start_position, Eigen::VectorXd end_position
      , double distance)
    {
        std::vector<Eigen::Vector3d> poses;
        Eigen::VectorXd next_pose(3);
        auto position_vector = end_position - start_position;
        auto x = position_vector[0];
        auto y = position_vector[1];
        auto z = position_vector[2];
        double diff = position_vector.norm();
        if( diff <= 0.0){
            std::cout<< "Next pose of the cant be equal or less than zero..." << next_pose.transpose() << std::endl;
        }
        if(diff < distance){
            poses.push_back(end_position);
            return poses;
        }
        auto theta = std::atan2(y, x);
        auto phi = std::atan2(std::sqrt(x*x + y*y), z);
        // std::cout<< "theta: "<< theta << " phi: " << phi << std::endl;
        while(true){
            auto target_z = distance*std::cos(phi) + start_position[2];
            auto target_x = distance*std::sin(phi)*std::cos(theta) + start_position[0];
            auto target_y = distance*std::sin(phi)*std::sin(theta) + start_position[1];
            next_pose<<target_x, target_y, target_z;
            poses.push_back(next_pose);
            distance += distance;
            if((next_pose.head(3)-end_position.head(3)).norm() <= distance){
                break;
            }
            // std::cout<< "Next pose:: inside:: not"<< next_pose << std::endl;
            // return next_pose;
        }
        return poses;
}

std::deque<double> MathUtils::linsplit(double start_in, double end_in, double number_of_steps){
  std::deque<double> linspaced;
  double start = start_in;
  double end = end_in;
  double num = number_of_steps;
  if (num == 0) {
        return linspaced;
  }
  if (num == 1)
  {
      linspaced.push_back(start);
      return linspaced;
  }
  double delta = (end - start) / (num - 1);
  for(int i=0; i < num-1; ++i)
  {
      linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
  return linspaced;
}

std::vector<double> MathUtils::linspace(double start_in, double end_in, double step_size){
  std::vector<double> linspaced;
  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);

  if (step_size == 0) { return linspaced; }
  
  double num = (end - start) / (step_size);
  for(int i=0; i < num; ++i)
  {
    linspaced.push_back(start + step_size * i);
  }
  linspaced.push_back(end);
  return linspaced;
}

void MathUtils::get_segments(double cx, double cy, double min_dis, double division, std::vector<double>& pose_x, std::vector<double>& pose_y){
    // double radius = 3;
    // double cx = 0.0, cy = 0.0;
    // double division = M_PI/18.0;

    std::vector<double> x_pose = linspace(0.0, 2.0*M_PI, division);
    std::vector<double> y_pose = linspace(0.0, 2.0*M_PI, division);
    std::for_each(x_pose.begin(), x_pose.end(), [&](double& f) { f = min_dis*std::cos(f) + cx;});
    std::for_each(y_pose.begin(), y_pose.end(), [&](double& f) { f = min_dis*std::sin(f) + cy;});
    pose_x = x_pose;
    pose_y = y_pose;
}

void MathUtils::get_squar_segments(Eigen::VectorXd info, std::vector<Eigen::Vector3d>& points_on_squar){
  double center_x = info(0);
  double center_y = info(1);
  double x_pose = info(2);
  double y_pose = info(3);
  double division = info(4);
  double z_val = info(5);
  std::vector<double> points_y = linspace(-y_pose, y_pose, division);
  std::vector<double> points_x = linspace(-x_pose, x_pose, division);
  for(auto y_i : points_y){
    Eigen::Vector3d pose_r(x_pose + center_x, y_i + center_y, z_val);
    Eigen::Vector3d pose_l(-x_pose + center_x, y_i + center_y, z_val);
    points_on_squar.push_back(pose_r);
    points_on_squar.push_back(pose_l);
  }
  for(auto x_i : points_x){
    Eigen::Vector3d pose_r(x_i + center_x, y_pose + center_y, z_val);
    Eigen::Vector3d pose_l(x_i + center_x, -y_pose + center_y, z_val);
    points_on_squar.push_back(pose_r);
    points_on_squar.push_back(pose_l);
  }
  return;
}

  

}  // namespace hagen_planner