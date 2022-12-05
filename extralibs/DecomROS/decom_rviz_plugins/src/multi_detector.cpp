#include <decom_rviz_plugins/multi_detector.h>


MultiDetector::MultiDetector(){


}

void MultiDetector::push_job(std::shared_ptr<LineSegment<3>>& worker) {
  ptask_t task = boost::make_shared<task_t>(boost::bind(&LineSegment<3>::dilate_parallel, std::move(worker)));
  boost::shared_future<free_info> fut(task->get_future());
  pending_data.push_back(fut);
  io_service.post(boost::bind(&task_t::operator(), task));
}

void MultiDetector::detect_freespace(std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>>& obs_, std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> path){
   const unsigned int N = path.size() - 1;
   lines_.resize(N);
   ellipsoids_.resize(N);
   polyhedrons_.resize(N);
   for (unsigned int i = 0; i < N; i++) {
        std::shared_ptr<LineSegment<3>> lin_segment(new LineSegment<3>(path[i], path[i+1]));
        lin_segment->set_local_bbox(Eigen::Matrix<decimal_t, 3, 1>(1, 2, 1));
        lin_segment->set_obs(obs_);
        push_job(lin_segment);
   }
   boost::wait_for_all((pending_data).begin(), (pending_data).end());
   ellipsoids_.clear();
   polyhedrons_.clear();
   int cout1 = 0;
   for(auto result : pending_data){
        free_info free_space = result.get();
        ellipsoids_.push_back(free_space.elliposids_);
        polyhedrons_.push_back(free_space.polyhedron_);
        // std::cout<< cout1 << std::endl;
        cout1++;
   }
   pending_data.clear();
}

void MultiDetector::getEllipsoids(vec_E<Ellipsoid<3>>& ellis){
    ellis = ellipsoids_;
}

void MultiDetector::getPolyhedrons(vec_E<Polyhedron<3>>& polyhe){
    polyhe = polyhedrons_;
}

bool MultiDetector::init(){
    service_work = boost::make_unique<boost::asio::io_service::work>(io_service);
    std::cout<< "Number of threads that can support this system: " << boost::thread::hardware_concurrency() << std::endl;
    for (int i = 0; i < boost::thread::hardware_concurrency(); ++i)
    {
        threads.create_thread(boost::bind(&boost::asio::io_service::run,
        &io_service));
    }
    std::cout<< "Thread pool has been initialized..." << boost::thread::hardware_concurrency() << std::endl;
    return true;
}