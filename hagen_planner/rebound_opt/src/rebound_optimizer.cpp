#include "rebound_opt/rebound_optimizer.h"
#include "rebound_opt/gradient_descent_optimizer.h"

// using namespace std;
namespace hagen_planner
{

  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    nh.param("optimization/lambda_collision", lambda2_, -1.0);
    nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    nh.param("optimization/lambda_fitness", lambda4_, -1.0);

    nh.param("optimization/dist0", dist0_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_points", max_points, 6);
    nh.param<std::string>("traj_common/root_dir", root_dir, "");

    nh.param("optimization/order", order_, 3);
  }

  void BsplineOptimizer::setEnvironment(const hagen_planner::EDTEnvironment::Ptr &env)
  {
    this->grid_map_ = env;
    map_range =  grid_map_->getMapCurrentRange();
    min_map_range = map_range[0];
    max_map_range = map_range[1];
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.clearance = dist0_;
    cps_.resize(points.cols());
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) {
     bspline_interval_ = ts; 
  }

  void BsplineOptimizer::getJSPPath(Eigen::Vector3d in, Eigen::Vector3d out,  std::vector<Eigen::Vector3d>& path){
      Timer time_jps(true);
      vec_E<Polyhedron<3>> polyhedrons;
      jsp_planner->planner_verbose_ = true;
      // jsp_planner->updateMap(polyhedrons);
      bool valid_jps = jsp_planner->plan(in, out, 1, true); 
      std::cout<< valid_jps << std::endl;
      double dt_jps = time_jps.Elapsed().count();
      printf("JPS Planner takes2: %f ms\n", dt_jps);
      printf("JPS Path Distance2: %f\n", jsp_planner->total_distance(jsp_planner->getRawPath()));
      printf("JPS Path2: \n");
      std::vector<Eigen::Vector3d> path_jps;
      if(valid_jps){
        path_jps = jsp_planner->getRawPath();
      }
    
      std::vector<Eigen::Vector3d> jsp_path;
      if(valid_jps){
        jsp_path.push_back(in);
        for(const auto& it: path_jps){
          std::cout << it.transpose() << std::endl;
          Eigen::Vector3d next_pose(it(0), it(1), it(2));
          jsp_path.push_back(next_pose);
        }
        jsp_path.push_back(out);
      }
      path = jsp_path;
  }

bool BsplineOptimizer::MoveBack(Eigen::Vector3d start_pt, Eigen::Vector3d &start_free, int index_, int& final_index){
    ROS_WARN("Start point moving back...");
    for(int g=seg_init+index_; g>0; g--){
      if (!grid_map_->getInflateOccupancy(horizon_trajectory.col(g))){
        start_free = horizon_trajectory.col(g);
        final_index = (g<=seg_init)? 0: g-seg_init;
        return true;
      }
    }
    return false;
  }

  bool BsplineOptimizer::MoveForward(Eigen::Vector3d start_pt, Eigen::Vector3d& start_free, int index_, int& final_index){
    ROS_WARN("Final point moving forward...");
    for(int g=seg_init+index_; g<horizon_trajectory.cols(); g++){
      if (!grid_map_->getInflateOccupancy(horizon_trajectory.col(g))){
        start_free = horizon_trajectory.col(g);
        final_index = (g>=seg_end)? cps_.points.cols()-1: g-seg_init;
        return true;
      }
    }
    return false;
  }

  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init)
  {
    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }
    // std::cout<< init_points <<std::endl;
    
    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    // std::cout<< "--------------->" << step_size << std::endl;
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; 
    // int i_end = (int)init_points.cols() - order_; 
    
    int index = 3;
    bool start_seg = false;
    bool end_seg_index_set = false; 
    std::vector<std::pair<int, int>> segment_ids1;
    int start_seg_index = 0;
    int end_seg_index = 0;
    int free_space_index = 0;
    int max_space = 4;

    for (int i = 2; i <= i_end; ++i)
    {
        occ = grid_map_->getInflateOccupancy(init_points.col(i));
        if(occ){
            if(start_seg==false && (free_space_index >= max_space || i<4)){
                start_seg = true;
                start_seg_index = index-1;
            }
            end_seg_index_set =  false;
            free_space_index = 0;
        }else{
            free_space_index++;
            if(start_seg && (free_space_index >= max_space || i<4)){
                if(start_seg_index < end_seg_index){
                  std::pair<int, int> segment(start_seg_index, end_seg_index);
                  // ROS_WARN("segment_ids push back init -> : %d, %d", start_seg_index, end_seg_index);
                  segment_ids.push_back(segment);
                }
                start_seg = false;
            }
            if(!end_seg_index_set){
                end_seg_index = index;
                end_seg_index_set = true;
            }
        }
        index++;
    }

    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      Eigen::MatrixXd refined_points;
      std::vector<Eigen::Vector3d> rebound_array_dead_refine;
      bool set_path = false;
      if(set_path){
          for(int jkl=0; jkl<refined_points.cols(); jkl++){
            rebound_array_dead_refine.push_back(refined_points.col(jkl).head(3));
          }
      }
      
      if(set_path){
        a_star_pathes.push_back(rebound_array_dead_refine);
      }else if (a_star_->AstarSearch(0.1, in, out)){
        a_star_pathes.push_back(a_star_->getPath());
      }
      else
      {
        ROS_ERROR("Can not find a path segment (;");
        force_stop_type_ = STOP_FOR_ERROR;
        return a_star_pathes;
      }
      
    }

    /*** calculate bounds ***/
    int seg_low_bound, seg_high_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    int num_points;

    for (size_t i = 0; i < segment_ids.size(); i++){
        if(i==0){
          seg_low_bound = order_;
          seg_high_bound = (segment_ids.size() > 1) ? (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2) 
                                  : init_points.cols() - order_ - 1;
        }else if(i=segment_ids.size()-1){
          seg_low_bound = (int)(((segment_ids[i].first + segment_ids[i-1].second) + 1.0f) / 2); 
          seg_high_bound = init_points.cols() - order_ - 1;
          bounds[i] = std::pair<int, int>(seg_low_bound, seg_high_bound);
        }else{
          seg_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); 
          seg_high_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2); 
        }
        bounds[i] = std::pair<int, int>(seg_low_bound, seg_high_bound);

        num_points = segment_ids[i].second - segment_ids[i].first + 1;
        //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
        int modified_seg_is_begin = segment_ids[i].first;
        int modified_seg_is_end = segment_ids[i].second;
        if (num_points < 0)
        {
          double add_points_each_side = (int)(((-num_points) + 1.0f) / 2);
          modified_seg_is_begin = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;
          modified_seg_is_end = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
        }
        for (int j = modified_seg_is_begin; j <= modified_seg_is_end; ++j){
          cps_.flag_temp[j] = false;
        }   

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j){
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                  (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );
              got_intersection_id = j;
              break;
            }
          }
        
          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              Eigen::Vector3d normalized_direction = (intersection_point - cps_.points.col(j)).normalized();
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                Eigen::Vector3d pose = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                occ = grid_map_->getInflateOccupancy(pose);
                if (occ || a < grid_map_->getResolution()){
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back(normalized_direction);
                  break;
                }
              }
            }
          }
      }
    }

    return a_star_pathes;
  }

  
  void BsplineOptimizer::save_matrix(std::vector<std::vector<Eigen::Vector3d>> data, std::string file_name){
    std::vector<double> data_map; 
    int count = 0;
    for(auto dd : data){
      for(auto indexi : dd){
        data_map.push_back(indexi(0));
        data_map.push_back(indexi(1));
        data_map.push_back(indexi(2));
      }
    }
    cnpy::npy_save(file_name, &data_map[0],{(unsigned int)1, (unsigned int)1, (unsigned int)data_map.size()},"w");
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          // cost += a * dist_err * dist_err + b * dist_err + c;
          // gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {
    cost = 0.0;
    Eigen::Vector3d acc, temp_acc;
    for (int i = 0; i < q.cols() - 2; i++)
    {
      acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
      cost += acc.squaredNorm();
      temp_acc = 2.0 * acc;
      gradient.col(i + 0) += temp_acc;
      gradient.col(i + 1) += -2.0 * temp_acc;
      gradient.col(i + 2) += temp_acc;
    }
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    double ts, ts_inv2;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2;
          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;
          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
      }
      if(i<q.cols()-2){
        Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;
        for (int j = 0; j < 3; j++)
        {
          if (ai(j) > max_acc_)
          {
            cost += pow(ai(j) - max_acc_, 2);
            gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
            gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
            gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
          }
          else if (ai(j) < -max_acc_)
          {
            cost += pow(ai(j) + max_acc_, 2);
            gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
            gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
            gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
          }
        }
      }
    }
  }

  void BsplineOptimizer::calPoly(vec_E<Polyhedron<3>> polyhedrons,  std::vector<Eigen::MatrixXd>& filtered_){
    Eigen::Vector3d point = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d normal = Eigen::MatrixXd::Zero(3,1);
    int numPolys = polyhedrons.size();
    std::vector<Eigen::MatrixXd> selected_poly;
    std::vector<double> point_normal;
    for(int i=0; i < numPolys; ++i){
        auto hperplanes = polyhedrons[i].vs_;
        int j=0;
        Eigen::MatrixXd set_a_b(hperplanes.size(), 4);
        for(auto po : hperplanes){
            point << po.p_[0], po.p_[1], po.p_[2];
            normal << po.n_[0], po.n_[1], po.n_[2];
            
            double d = normal.transpose() * point;
            set_a_b.block(j,0, 1, 3) = normal.transpose();
            set_a_b(j,3) = 1.0*d;
            j++;
        }
        selected_poly.push_back(set_a_b);
    }

    filtered_ = selected_poly;
    return;
  }


  bool BsplineOptimizer::check_collision_and_rebound(void)
  {
    int end_idx = cps_.size - order_;
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {
      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; 
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;
        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          in_id = 0;
          force_stop_type_ = STOP_FOR_ERROR;
          ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
          current_pose_obs = 0;
        }else{
          current_pose_obs = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        if((previous_index_in == in_id) && (previous_index_out == out_id)){
          times_segs++;
        }else{
          times_segs = 0;
        }

        previous_index_in = in_id;
        previous_index_out = out_id;
        
        if(times_segs<3){
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
       
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        Eigen::MatrixXd refined_points;
        std::vector<Eigen::Vector3d> rebound_array_dead_refine;
        bool set_path = false;
        if(set_path){
            for(int jkl=0; jkl<refined_points.cols(); jkl++){
              rebound_array_dead_refine.push_back(refined_points.col(jkl).head(3));
            }
            visualization_->drawPath(rebound_array_dead_refine, 0.2, Eigen::Vector4d(07, 0.9 ,0.7, 1), 128);
        }
        if(set_path){
          a_star_pathes.push_back(rebound_array_dead_refine);
        }
        else if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out)){
          a_star_pathes.push_back(a_star_->getPath());
        }
        else
        {
          ROS_ERROR("Can not find a path segment, remove the corresponding segment... %d  %d", segment_ids[i].first, segment_ids[i].second);
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }


      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );
              got_intersection_id = j;
              break;
            }
          }
          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              Eigen::Vector3d normalized_direction = (intersection_point - cps_.points.col(j)).normalized();
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                Eigen::Vector3d pose = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                bool occ = grid_map_->getInflateOccupancy(pose);
                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back(normalized_direction);
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }
        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
      }
      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  bool BsplineOptimizer::calculateOptimalPoints(Eigen::MatrixXd& a, Eigen::MatrixXd& b
                    , Eigen::MatrixXd& p_init, Eigen::MatrixXd& refined_points){
    return false;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts, Eigen::Vector3d start_odom_, int& current_pose_obs_){
    setBsplineInterval(ts);
    bool flag_success = rebound_optimize_g1();
    optimal_points = cps_.points;
    start_odom_set = true;
    start_odom = start_odom_;
    return flag_success;
  }

  double BsplineOptimizer::operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad){
    double cost;
    combineCostRebound(x, grad, cost);
    iter_num_ += 1;
    return cost;
  }

  bool BsplineOptimizer::rebound_optimize_g1()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.size - order_;
    variable_num_ = 3 * (end_id - start_id);
    double final_cost;

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {

      const int n = 10;
      LBFGSpp::LBFGSBParam<double> param;
      param.epsilon = 0.01;
      param.max_iterations = 100;
      LBFGSpp::LBFGSBSolver<double> solver(param);
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;
      
      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      Eigen::VectorXd x_o(Eigen::Map<Eigen::VectorXd>(cps_.points.data(), cps_.points.cols()*cps_.points.rows()));
      lb = Eigen::VectorXd::Constant(cps_.points.cols()*3, 0.0);
      ub = Eigen::VectorXd::Constant(cps_.points.cols()*3, 20.0);
      
      for(int v=0; v< cps_.points.cols(); v++){
        int index = v*3;
        lb[index] = min_map_range[0];
        lb[index+1] = min_map_range[1];
        lb[index+2] = min_map_range[2];
        ub[index] = max_map_range[0];
        ub[index+1] = max_map_range[1];
        ub[index+2] = max_map_range[2];
      }
      
      double fx;
      int result = 0;
      try{
        result = solver.minimize(*this, x_o, fx, lb, ub);
      }catch (const std::runtime_error& error){
        std::string str = error.what(); 
        string str1 = "maximum"; 
        size_t found = str.find(str1); 
        if (found != string::npos){
          result = param.max_iterations;
        }
      } catch (const std::logic_error& error){
        result = param.max_iterations;
      }

    
      /* ---------- success temporary, check collision again ---------- */
      if (result <= param.max_iterations){
        flag_force_return = false;

        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
          if (flag_occ){
            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              return false;
            }
            break;
          }
        }

        if (!flag_occ)
        {
          success = true;
        }
        else // restart
        {
          restart_nums++;
          new_lambda2_ *= 2;
        }
      }else
      {
        rebound_times++;
      }
    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 10));

    return success;
  }

  void BsplineOptimizer::combineCostRebound(const Eigen::VectorXd& x, Eigen::VectorXd& grad, double &f_combine){

    cps_.points = Eigen::Map<const Eigen::MatrixXd>(x.data(), cps_.points.rows(), cps_.points.cols());

    double f_smoothness, f_distance, f_feasibility;
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    ros::Time t0 = ros::Time::now(), t1, t2;
    t1 = ros::Time::now();
    t0 = ros::Time::now();
    double time_ms = (t2 - t1).toSec() * 1000;
    double total_time_ms = (t2 - t0).toSec() * 1000;
    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    t2 = ros::Time::now();
    time_ms = (t2 - t1).toSec() * 1000;
    total_time_ms = (t2 - t0).toSec() * 1000;
    t1 = ros::Time::now();
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    t2 = ros::Time::now();
    time_ms = (t2 - t1).toSec() * 1000;
    total_time_ms = (t2 - t0).toSec() * 1000;
    t1 = ros::Time::now();
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    t2 = ros::Time::now();
    time_ms = (t2 - t1).toSec() * 1000;
    total_time_ms = (t2 - t0).toSec() * 1000;
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
    Eigen::VectorXd grad_1d(Eigen::Map<Eigen::VectorXd>(grad_3D.data(), grad_3D.cols()*grad_3D.rows()));
    grad = grad_1d;
  }

} // namespace hagen_planner