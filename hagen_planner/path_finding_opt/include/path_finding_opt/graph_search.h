#ifndef JPS_GRAPH_SEARCH_H
#define JPS_GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map
#include <iostream>
#include <map_building_opt/edt_environment.h>

namespace hagen_planner
{
  ///Heap element comparison
  template <class T>
  struct compare_state
  {
    bool operator()(T a1, T a2) const
    {
      double f1 = a1->g + a1->h;
      double f2 = a2->g + a2->h;
      if( ( f1 >= f2 - 0.000001) && (f1 <= f2 +0.000001) )
        return a1->g < a2->g; // if equal compare gvals
      return f1 > f2;
    }
  };


  struct State; // forward declaration
  using StatePtr = std::shared_ptr<State>;
  using priorityQueue = boost::heap::d_ary_heap<StatePtr, boost::heap::mutable_<true>,
                        boost::heap::arity<2>, boost::heap::compare< compare_state<StatePtr> >>;

  struct State
  {
    /// ID
    int id;
    /// Coord
    int x, y, z = 0;
    /// direction
    int dx, dy, dz;                            // discrete coordinates of this node
    /// id of predicessors
    int parentId = -1;

    /// pointer to heap location
    priorityQueue::handle_type heapkey;

    /// g cost
    double g = std::numeric_limits<double>::infinity();
    /// heuristic cost
    double h;
    /// if has been opened
    bool opened = false;
    /// if has been closed
    bool closed = false;


    /// 3D constructor
    State(int id, int x, int y, int z, int dx, int dy, int dz )
      : id(id), x(x), y(y), z(z), dx(dx), dy(dy), dz(dz)
    {}

    /// 3D constructor
    State(int id, int x, int y, int z)
      : id(id), x(x), y(y), z(z)
    {}

  };

  struct JPS3DNeib {
    // for each (dx,dy,dz) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    int ns[27][3][26];
    int f1[27][3][12];
    int f2[27][3][12];
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        26 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          8 forced neighbors to check
    //                          8 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          8 forced neighbors to check
    //                          12 neighbors to add if forced
    // diagonal (norm sqrt(3)): 7 neighbors always added
    //                          6 forced neighbors to check
    //                          12 neighbors to add if forced
    static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
    JPS3DNeib();
    private:
    void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
    void FNeib( int dx, int dy, int dz, int norm1, int dev,
        int& fx, int& fy, int& fz,
        int& nx, int& ny, int& nz);
  };

  class GraphSearch
  {
    public:
     
      GraphSearch(const hagen_planner::EDTEnvironment::Ptr map_util_, double eps=1, bool verbose=false);

      bool plan( Eigen::Vector3i start_pt, Eigen::Vector3i end_pt, bool useJps, int maxExpand=-1);
      std::vector<StatePtr> getPath() const;
      std::vector<StatePtr> getOpenSet() const;
      std::vector<StatePtr> getCloseSet() const;
      std::vector<StatePtr> getAllSet() const;
      bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;
      Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
  
      hagen_planner::EDTEnvironment::Ptr edt_env_;
      Eigen::Vector3d center_;
	    Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
      double step_size_, inv_step_size_;

      
      bool plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id);
      void getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
      void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
      std::vector<StatePtr> recoverPath(StatePtr node, int id);

      int coordToId(int x, int y, int z) const;
      int coordToId(Eigen::Vector3i pose) const;
      bool isFree(int x, int y, int z) const;
      bool isOccupied(int x, int y, int z) const;
      double getHeur(int x, int y, int z) const;
      bool hasForced(int x, int y, int z, int dx, int dy, int dz);
      bool jump(int x, int y, int z, int dx, int dy, int dz, int& new_x, int& new_y, int& new_z);

      std::vector<Eigen::Vector3i> rayTrace(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2);
      bool isOutside(Eigen::Vector3i idx);
      bool checkOccupancy(const Eigen::Vector3d &pos);
      bool isBlocked(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int8_t val=0);
      bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt
      , Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);


      int xDim_, yDim_, zDim_;
      double eps_;
      bool verbose_;

      const char val_free_ = 0;
      int xGoal_, yGoal_, zGoal_;
      Eigen::Vector3i search_dim;

      priorityQueue pq_;
      std::vector<StatePtr> hm_;
      std::vector<bool> seen_;
      double max_size = 100;
      double res_ = 0.1;
      std::vector<StatePtr> path_;

      std::vector<std::vector<int>> ns_;
      std::shared_ptr<JPS3DNeib> jn3d_;
 };
}
#endif
