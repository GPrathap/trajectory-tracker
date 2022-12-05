#include <spline_opt/reference_path.h>


namespace hagen_planner
{
    Waypoint::Waypoint(double x, double y, double psi, double kappa){
        x_ = x;
        y_ = y;
        psi_ = psi;
        kappa_ = kappa;
    }


    Waypoint::Waypoint(double x, double y, double z, double psi, double kappa){
        x_ = x;
        y_ = y;
        z_ = z;
        psi_ = psi;
        kappa_ = kappa;
    }


    double Waypoint::sub(Waypoint other){
        return std::sqrt(std::pow((x_-other.x_), 2) + std::pow((y_-other.y_), 2));
    }

    double Waypoint::sub3d(Waypoint other){
        return std::sqrt(std::pow((x_-other.x_), 2) + std::pow((y_-other.y_), 2) + std::pow((z_-other.z_), 2));
    }

    Waypoint ReferencePath::get_waypoint(int index){
        if(index >= waypoints.size()){
            index = waypoints.size()-1;
        }
        return waypoints[index];
    }
      
    ReferencePath::ReferencePath(){
        osqpInterface = new OsqpEigen::Solver();
    }
    void ReferencePath::construct_waypoints(Eigen::MatrixXd points, bool use_3d){
            waypoints.clear();
            for(int wp=0; wp<points.cols()-1; wp++){
                // std::cout<< "==================================================" << std::endl;
                Eigen::Vector2d current_wp = points.col(wp).head(2);
                Eigen::Vector2d next_wp = points.col(wp+1).head(2);
                Eigen::Vector2d dif_ahead = next_wp - current_wp;
                double psi = std::atan2(dif_ahead[1], dif_ahead[0]);
                double dist_ahead = dif_ahead.norm();
                double kappa;
                if(wp==0){
                    kappa = 0;
                }else{
                    Eigen::Vector2d prev_wp = points.col(wp-1).head(2);
                    // std::cout<< prev_wp.transpose() << std::endl;
                    Eigen::Vector2d dif_behind = current_wp - prev_wp;
                    // std::cout<< dif_behind.transpose() << std::endl;
                    double angle_behind = std::atan2(dif_behind[1], dif_behind[0]);
                    double angle_diff = std::fmod((psi - angle_behind + M_PI), 2.0*M_PI) - M_PI; 
                    // std::cout<< angle_behind << std::endl;
                    // std::cout<< angle_diff << std::endl;
                    kappa = (angle_diff / dist_ahead + 1e-12);
                }
               
                // std::cout<< kappa << std::endl;
                if(use_3d){
                    Waypoint way_p(current_wp[0], current_wp[1], points.col(wp)[2], psi, kappa);
                    waypoints.push_back(way_p);
                }else{
                    Waypoint way_p(current_wp[0], current_wp[1], psi, kappa);
                    waypoints.push_back(way_p);
                    // std::cout<< current_wp[0] << "," << current_wp[1] << "," << psi << "," << kappa << std::endl;
                    // std::cout<< points.col(wp)[0] << "," << points.col(wp)[1] << "," << points.col(wp)[3] << "," << points.col(wp)[2] << std::endl;
                }
            }
            compute_length(segment_lengths, length);
    }

    void ReferencePath::update_path_constraints(int index, int N, double min_width
                        , double safety_margin, Eigen::MatrixXd& ub_, Eigen::MatrixXd& lb_, bool use_3d){
        
        ub_.resize(1, N);
        lb_.resize(1, N);
        Eigen::VectorXd ub_ls, lb_ls;
        if(use_3d){
            ub_ls.resize(3);
            lb_ls.resize(3);
        }else{
            ub_ls.resize(2);
            lb_ls.resize(2);
        }
        for (int n = 0; n < N; n++)
        {  
            Waypoint wp = get_waypoint(index+n);
            if(use_3d){
                ub_ls << wp.x_, wp.y_, wp.z_;
                lb_ls << wp.x_, wp.y_, wp.z_;
            }else{
                ub_ls << wp.x_, wp.y_;
                lb_ls << wp.x_, wp.y_;
            }
            
            double angle_ub = std::fmod(std::atan2(ub_ls[1]-wp.y_, ub_ls[0]-wp.x_)-wp.psi_ + M_PI, 2*M_PI)-M_PI;
            double angle_lb = std::fmod(std::atan2(lb_ls[1]-wp.y_, lb_ls[0]-wp.x_)-wp.psi_ + M_PI, 2*M_PI)-M_PI;
            double sign_ub = angle_ub > 0.0 ? 1: -1;
            double sign_lb = angle_lb > 0.0 ? 1: -1;
            double ub = sign_ub * std::sqrt(std::pow((ub_ls[0]-wp.x_), 2) + std::pow((ub_ls[1]-wp.y_), 2));
            double lb = sign_lb * std::sqrt(std::pow((lb_ls[0]-wp.x_), 2) + std::pow((lb_ls[1]-wp.y_), 2));
            // double lb = sign_lb * std::sqrt(std::pow((lb_ls[0]-wp.x_), 2) + std::pow((lb_ls[1]-wp.y_), 2));
            ub -= safety_margin;
            lb += safety_margin;
            if(ub < lb){
                ub = 0.0;
                lb = 0.0;
            }
            ub_.col(n) << ub;
            lb_.col(n) << lb;
        }
    }
    
    void ReferencePath::compute_length(std::vector<double>& segment_lengths, double& distance){
        distance = 0;
        segment_lengths.clear();
        segment_lengths.push_back(0.0);
        for (int wp_id = 0; wp_id < waypoints.size()-1; wp_id++){
            double dis = waypoints[wp_id+1].sub(waypoints[wp_id]);
            distance += dis;
            segment_lengths.push_back(dis); 
        }
    }

    void ReferencePath::refine_trajectory(double smoothing_dis, double resolution , Eigen::MatrixXd& waypoints
                            , Eigen::MatrixXd& smooth_wps, bool use_3d){
        
        std::vector<int> n_wp;
        int total_wps = 0;
        int init_size = waypoints.cols();
        // std::cout<< "=====1"<< std::endl;
        if(use_3d){
            for (int i = 0; i < init_size-1; i++)
            {
                int count = (int)((waypoints.col(i+1).head(3) - waypoints.col(i).head(3)).norm()/resolution);
                total_wps += count;
                n_wp.push_back(count);
                // std::cout<< "=====2: "<< count << "," << i << std::endl;
            }
        
            Eigen::Vector3d  gp_wp = waypoints.col(init_size-1).head(3);
            Eigen::MatrixXd new_wps(3, total_wps+1);
            int wp_count = 0;
            // std::cout<< "=====g "<< gp_wp << std::endl;
            for (int i = 0; i < init_size-1; i++){
                std::vector<double> inter_set_x = linspace(waypoints.col(i)(0), waypoints.col(i+1)(0), n_wp[i]);
                std::vector<double> inter_set_y = linspace(waypoints.col(i)(1), waypoints.col(i+1)(1), n_wp[i]);
                std::vector<double> inter_set_z = linspace(waypoints.col(i)(2), waypoints.col(i+1)(2), n_wp[i]);
                int count_split = std::min(inter_set_z.size(), std::min(inter_set_x.size(), inter_set_y.size()));
                for (int k = 0; k < count_split; k++){
                    Eigen::Vector3d wp_i(inter_set_x[k], inter_set_y[k], inter_set_z[k]);
                    //  std::cout<< wp_i.transpose() << std::endl;
                    // wp_i += gp_wp;
                    new_wps.col(wp_count) = wp_i;
                    wp_count++;
                } 
            }
            new_wps.col(total_wps) = gp_wp;
          
            std::cout<< "=====4: "<< wp_count << "," << new_wps.cols() << std::endl;
            std::vector<Eigen::Vector3d> wp_xs;
            for (int wp_id = smoothing_dis; wp_id < new_wps.cols() - smoothing_dis; wp_id++){
                // std::cout<< "=====4: " << wp_id-smoothing_dis << " ==== " << wp_id+smoothing_dis+1 -(wp_id-smoothing_dis) << std::endl;
                // std::cout<< "=====x4: " << new_wps.block(0, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)) << std::endl;
                // std::cout<< "=====y4: " << new_wps.block(1, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)) << std::endl;
                double next_wp_xs = new_wps.block(0, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)).mean();
                // std::cout<< next_wp_xs << std::endl;
                double next_wp_ys = new_wps.block(1, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)).mean();
                double next_wp_zs = new_wps.block(2, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)).mean();
                // std::cout<< "=====y4: " << next_wp_xs << "," << next_wp_ys << std::endl;
                Eigen::Vector3d smoothed_wp(next_wp_xs, next_wp_ys, next_wp_zs);
                wp_xs.push_back(smoothed_wp);
            }
            // std::cout<< "=====5"<< std::endl;
            smooth_wps.resize(3, wp_xs.size());
            for (int i = 0; i < wp_xs.size(); i++){
                smooth_wps.col(i) = wp_xs[i];
            }
        }else{
            for (int i = 0; i < init_size-1; i++)
            {
                int count = (int)((waypoints.col(i+1).head(2) - waypoints.col(i).head(2)).norm()/resolution);
                total_wps += count;
                n_wp.push_back(count);
                // std::cout<< "=====2: "<< count << "," << i << std::endl;
            }
        
            Eigen::Vector2d  gp_wp = waypoints.col(init_size-1).head(2);
            Eigen::MatrixXd new_wps(2, total_wps+1);
            int wp_count = 0;
            // std::cout<< "=====g "<< gp_wp << std::endl;
            for (int i = 0; i < init_size-1; i++){
                std::vector<double> inter_set_x = linspace(waypoints.col(i)(0), waypoints.col(i+1)(0), n_wp[i]);
                std::vector<double> inter_set_y = linspace(waypoints.col(i)(1), waypoints.col(i+1)(1), n_wp[i]);
                for (int k = 0; k < inter_set_x.size(); k++){
                    Eigen::Vector2d wp_i(inter_set_x[k], inter_set_y[k]);
                    //  std::cout<< wp_i.transpose() << std::endl;
                    // wp_i += gp_wp;
                    new_wps.col(wp_count) = wp_i;
                    wp_count++;
                } 
            }
            new_wps.col(total_wps) = gp_wp;
          
            std::cout<< "=====4: "<< wp_count << "," << new_wps.cols() << std::endl;
            std::vector<Eigen::Vector2d> wp_xs;
            for (int wp_id = smoothing_dis; wp_id < new_wps.cols() - smoothing_dis; wp_id++){
                // std::cout<< "=====4: " << wp_id-smoothing_dis << " ==== " << wp_id+smoothing_dis+1 -(wp_id-smoothing_dis) << std::endl;
                // std::cout<< "=====x4: " << new_wps.block(0, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)) << std::endl;
                // std::cout<< "=====y4: " << new_wps.block(1, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)) << std::endl;
                double next_wp_xs = new_wps.block(0, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)).mean();
                // std::cout<< next_wp_xs << std::endl;
                double next_wp_ys = new_wps.block(1, wp_id-smoothing_dis, 1, wp_id+smoothing_dis+1 -(wp_id-smoothing_dis)).mean();
                // std::cout<< "=====y4: " << next_wp_xs << "," << next_wp_ys << std::endl;
                Eigen::Vector2d smoothed_wp(next_wp_xs, next_wp_ys);
                wp_xs.push_back(smoothed_wp);
            }
            // std::cout<< "=====5"<< std::endl;
            smooth_wps.resize(2, wp_xs.size());
            for (int i = 0; i < wp_xs.size(); i++){
                smooth_wps.col(i) = wp_xs[i];
            }
        }
       
    }

    void ReferencePath::calculate_velocity_profile(double a_max_, double a_min_, double v_max_, double v_min_, double ay_max_){

        int N  = waypoints.size()-1;
        Eigen::MatrixXd a_min = Eigen::MatrixXd::Ones(1, N-1)*a_min_;
        Eigen::MatrixXd a_max = Eigen::MatrixXd::Ones(1, N-1)*a_max_;
        Eigen::MatrixXd v_min = Eigen::MatrixXd::Ones(1, N)*v_min_;
        Eigen::MatrixXd v_max = Eigen::MatrixXd::Ones(1, N)*v_max_;
        double ay_max = ay_max_;
        Eigen::MatrixXd D1 = Eigen::MatrixXd::Zero(N-1, N);
        std::cout<< "======================ki=======================" << std::endl;
        for(int i=0; i<N; i++){
            Waypoint current_wp = waypoints[i];
            Waypoint next_wp = waypoints[i+1];
            // std::cout<< "======================ki=======================" << std::endl;
            double li =  next_wp.sub(current_wp);
            double ki =  current_wp.kappa_;
            if(i < N-1){
                D1.block(i, i, 1, 2)<< -1/(2*li), 1/(2*li);
                // std::cout<< -1/(2*li) << " ,"<< 1/(2*li)<< std::endl;
            }
            double v_max_dyn = std::sqrt(ay_max/(std::abs(ki)+1e-12));
            if(v_max_dyn < v_max(0, i)){
                v_max(0, i) = v_max_dyn;
            }
            // std::cout<< ki << std::endl;
            // std::cout<< li << std::endl;
            // std::cout<< v_max_dyn << std::endl;
           
        }

        // Eigen::SparseMatrix<c_float> D11;
        // D11.resize(D1.rows(), D1.cols());
        // for(size_t i=0; i< D1.rows(); i++){
        //     for (size_t j = 0; j < D1.cols(); j++){
        //         D11.coeffRef(i, j) = D1(i,j);
        //     }
        // }
        std::cout<< "======================ki= end======================" << std::endl;
        // Eigen::SparseMatrix<c_float> D2;
        Eigen::SparseMatrix<double> P;
        // D2.resize(N, N);
        P.resize(N,N);
        // D2.setZero();
        P.setIdentity();
        // D2.setIdentity();
        std::cout<< "====1" << std::endl;
        Eigen::SparseMatrix<c_float> D;
        D.resize(D1.rows() + N, N);
        for(int i=0; i<D1.rows(); i++){
            for (int j = 0; j < D1.cols(); j++)
            {
                float value = D1(i,j);
                if(value != 0){
                    D.insert(i, j) = value;
                }   
            }
        }
         std::cout<< "====2" << std::endl;
        // for(int i=D1.rows(); i<D1.rows()+N; i++){
        for (int j = 0; j < N; j++)
        {
            D.insert(D1.rows()+j, j) = 1; 
        }
        // }
        // sp::colMajor::addRows(D, D11);
        // sp::colMajor::addRows(D, D2);
         std::cout<< "====3" << std::endl;
        Eigen::SparseMatrix<c_float> q(N,1);
        Eigen::SparseMatrix<c_float> l;
        l.resize(N-1 + N, 1);
        Eigen::SparseMatrix<c_float> u;
        u.resize(N-1 + N, 1);

        // Eigen::SparseMatrix<c_float> a_min_sparse(N-1, 1);
        // Eigen::SparseMatrix<c_float> a_max_sparse(N-1, 1);
        // Eigen::SparseMatrix<c_float> v_min_sparse(N, 1);
        // Eigen::SparseMatrix<c_float> v_max_sparse(N, 1);


        for(int i=0; i<N-1; i++){
            l.insert(i,0) = a_min(0, i);
            u.insert(i,0) = a_max(0, i);
        }

        for(int i=N-1; i<N+N-1; i++){
            // v_min_sparse.coeffRef(i,0) = v_min(0, i);
            // v_max_sparse.coeffRef(i,0) = v_max(0, i);
            l.insert(i, 0) = v_min(0, i-(N-1));
            u.insert(i, 0) = v_max(0, i-(N-1));
            q.coeffRef(i-(N-1), 0) = v_max(0,i-(N-1))*-1.0;
        }

       
        // sp::colMajor::addRows(l, a_min_sparse);
        // sp::colMajor::addRows(l, v_min_sparse);

        // sp::colMajor::addRows(u, a_max_sparse);
        // sp::colMajor::addRows(u, v_max_sparse);
        

        Eigen::VectorXd lowerBound(l.rows());
        Eigen::VectorXd upperBound(u.rows());
        Eigen::VectorXd gradient(q.rows());
        for(int i=0; i< u.rows(); i++){
            lowerBound[i] = l.coeffRef(i, 0);
            upperBound[i] = u.coeffRef(i, 0);
        }

        for(int i=0; i< q.rows(); i++){
            gradient[i] = q.coeffRef(i, 0);
        }

        int solveStatus = 1;
        // std::cout << "----------------------PqDlu------------------------"<< P.rows() << "," << P.cols() << std::endl;
        // std::cout << "----------------------PqDlu------------------------"<< P << std::endl;
        // std::cout << Eigen::MatrixXd(P) << std::endl;
        // std::cout << Eigen::MatrixXd(q) << std::endl;
        // std::cout << D << std::endl;
        // std::cout << Eigen::MatrixXd(l) << std::endl;
        // std::cout << Eigen::MatrixXd(u) << std::endl;

        // P = P.triangularView<Eigen::Upper>();
        // qpSolver interface
        // std::cout<< P.rows() << "  " << P.cols() << " "<< D.cols() << " "<< D.rows() << std::endl;
        osqpInterface->settings()->setWarmStart(true);
        osqpInterface->data()->setNumberOfVariables(P.cols());
        osqpInterface->data()->setNumberOfConstraints(D.rows());
        bool set_correct = true;
        osqpInterface->data()->clearHessianMatrix();
        if(!osqpInterface->data()->setHessianMatrix(P)) set_correct = false;
        if(!osqpInterface->data()->setGradient(gradient)) set_correct = false;
        osqpInterface->data()->clearLinearConstraintsMatrix();
        if(!osqpInterface->data()->setLinearConstraintsMatrix(D)) set_correct = false;
        if(!osqpInterface->data()->setLowerBound(lowerBound)) set_correct = false;
        if(!osqpInterface->data()->setUpperBound(upperBound)) set_correct = false;

        // instantiate the solver
        osqpInterface->clearSolver();
        if(!osqpInterface->initSolver()) set_correct = false;
         // solve the QP problem
        if(!osqpInterface->solve()) set_correct = false;
        Eigen::VectorXd QPSolution;
        QPSolution = osqpInterface->getSolution();
        // std::cout<< "=================set_correct======================" << set_correct << std::endl;
        // if(set_correct){
            for(int j=0; j<N-1; j++){
                // std::cout<< QPSolution[j] << std::endl;
                // waypoints[j].v_ref = 0.45;
                waypoints[j].v_ref = QPSolution[j];
            }
            // waypoints[N].v_ref = QPSolution[N-1];
            waypoints[N].v_ref = QPSolution[N-1];
        // }
        // osqpInterface.updateMatrices(P, q, D, l, u);
        // osqpInterface.solveQP();
        // solveStatus = osqpInterface.solveStatus();
        // if (solveStatus == OSQP_PRIMAL_INFEASIBLE){
        //     std::cout << "SOLVER IS INFEASIBLE..." << std::endl;
        // }else if (solveStatus == OSQP_SOLVED){
        //     // std::cout << "Rerefence velocity i set..." << std::endl;
        //     auto solPtr = osqpInterface.solPtr();
        //     for(int j=0; j<N-1; j++){
        //         // std::cout<< solPtr->x[j] << std::endl;
        //         waypoints[j].v_ref = solPtr->x[j];
        //     }
        //     waypoints[N].v_ref = solPtr->x[N-1];
        // }
        // for(int j=0; j<N; j++){
        //         // std::cout<< solPtr->x[j] << std::endl;
        //         waypoints[j].v_ref = 0;
        // }
    }
}  // namespace hagen_planner
