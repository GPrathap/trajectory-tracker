/**
 * @file line_segment.h
 * @brief LineSegment Class
 */
#ifndef MULTI_DETECTOR
#define MULTI_DETECTOR


#include <ros/console.h>
#include <ros/ros.h>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <Eigen/Core>
#include <boost/filesystem.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/move/move.hpp>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <numeric>   
#include <algorithm> 
#include "ellipsoid.h"
#include "line_segment.h"


namespace asio = boost::asio; 

class MultiDetector{
    public:
        MultiDetector();
        ~MultiDetector() = default;
        bool init();
        void detect_freespace(std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>>& obs, std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>> path);
        void getEllipsoids(vec_E<Ellipsoid<3>>& ellis);
        void getPolyhedrons(vec_E<Polyhedron<3>>& polyhe);
    private:
        void push_job(std::shared_ptr<LineSegment<3>>& worker);
        typedef boost::packaged_task<free_info> task_t;
        typedef boost::shared_ptr<task_t> ptask_t;
        std::vector<boost::shared_future<free_info>> pending_data;
        boost::asio::io_service io_service;
        boost::thread_group threads;
        std::unique_ptr<boost::asio::io_service::work> service_work;

        vec_E<Ellipsoid<3>> ellipsoids_;
        vec_E<Polyhedron<3>> polyhedrons_;
        std::vector<std::shared_ptr<LineSegment<3>>> lines_;

        Vecf<3> local_bbox_{Vecf<3>::Zero()};
        Vecf<3> global_bbox_min_{Vecf<3>::Zero()}; // bounding box params
        Vecf<3> global_bbox_max_{Vecf<3>::Zero()};
};

#endif 
  