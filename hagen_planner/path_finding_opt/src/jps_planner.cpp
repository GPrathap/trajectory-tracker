#include <path_finding_opt/jps_planner.h>

JPSPlanner::JPSPlanner(bool verbose): planner_verbose_(verbose) {
  planner_verbose_ = verbose;
  if(planner_verbose_)
    printf("hagen_planner PLANNER VERBOSE ON\n");
}

void JPSPlanner::setEnvironment(const hagen_planner::EDTEnvironment::Ptr& env){
  graph_search_ = std::make_shared<hagen_planner::GraphSearch>(env, 1, true);
}

int JPSPlanner::status() { 
  return status_; 
}

double JPSPlanner::total_distance(const std::vector<Eigen::Matrix<double,3, 1>>& vs){
  double dist = 0;
  for(unsigned int i = 1; i < vs.size(); i++)
    dist += (vs[i] - vs[i-1]).norm();
  return dist;
}

std::vector<Eigen::Vector3d> JPSPlanner::getPath() { return path_; }

std::vector<Eigen::Vector3d> JPSPlanner::getRawPath() { return raw_path_; }

bool JPSPlanner::checkOccupiedConvex( Eigen::Vector3i& pn){
  for(Polyhedron<3> poly : free_space){
    Eigen::Matrix<double, 3, 1> pts(pn[0], pn[1], pn[2]);
    if(poly.inside(pts)){
      return 0;
    }
  }
  return 1;
}

bool JPSPlanner::plan(const Eigen::Vector3d&start, const Eigen::Vector3d &goal, double eps, bool use_jps) {
  if(planner_verbose_){
    std::cout <<"Start: " << start.transpose() << std::endl;
    std::cout <<"Goal:  " << goal.transpose() << std::endl;
    std::cout <<"Epsilon:  " << eps << std::endl;
  }

  path_.clear();
  raw_path_.clear();
  status_ = 0;
  graph_search_->center_ = (start + goal)/2;
  Eigen::Vector3i start_int;
  Eigen::Vector3i goal_int;
  if (!graph_search_->ConvertToIndexAndAdjustStartEndPoints(start, goal, start_int, goal_int))
  {
      ROS_ERROR("Unable to handle the initial or end point, force return!");
      return false;
  }

  if(planner_verbose_){
    std::cout <<"Start ids: " << start_int.transpose() << std::endl;
    std::cout <<"Goal ids :  " << goal_int.transpose() << std::endl;
  }

  graph_search_->plan(start_int, goal_int, true);
  const auto path = graph_search_->getPath();
  if (path.size() < 1) {
    if(planner_verbose_)
      std::cout <<  "Cannot find a path from " << start.transpose() <<" to " << goal.transpose() << " Abort! tried" << std::endl;
    status_ = -1;
    return false;
  }

  //**** raw path, s --> g
  std::vector<Eigen::Vector3d> ps;
  std::cout<< "=========jsp path===========" << std::endl;
  for (int i=path.size()-1; i>0; i--) {
    Eigen::Vector3i pn;
    pn << path[i]->x, path[i]->y, path[i]->z;
    std::cout<< graph_search_->Index2Coord(pn).transpose() << std::endl;
    ps.push_back(graph_search_->Index2Coord(pn));
  }

  raw_path_ = ps;
  return true;
}
