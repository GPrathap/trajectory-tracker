#include <path_finding_opt/graph_search.h>
#include <cmath>

using namespace hagen_planner;


GraphSearch::GraphSearch(const hagen_planner::EDTEnvironment::Ptr map_util_
                        , double eps, bool verbose) :
  edt_env_(map_util_), eps_(eps), verbose_(verbose)
{
  // set 3D neighbors
  for(int x = -1; x <= 1; x ++) {
    for(int y = -1; y <= 1; y ++) {
      for(int z = -1; z <= 1; z ++) {
        if(x == 0 && y == 0 && z == 0) continue;
        ns_.push_back(std::vector<int>{x, y, z});
      }
    }
  }
  jn3d_ = std::make_shared<JPS3DNeib>();
  xDim_ = 100;
  yDim_ = 100;
  zDim_ = 100;
  Eigen::Vector3i pool_size(xDim_, yDim_, zDim_);
  POOL_SIZE_ = pool_size;
  CENTER_IDX_ = pool_size / 2;
  step_size_ = 0.1;
  inv_step_size_ = 1 / step_size_;
  max_size = xDim_ * yDim_ * zDim_;
  hm_.resize((int)max_size);
  seen_.resize((int)max_size, false);
  res_ = step_size_;
  search_dim << xDim_, yDim_, zDim_;
}

int GraphSearch::coordToId(int x, int y, int z) const {
  return x + y*xDim_ + z*xDim_*yDim_;
}

int GraphSearch::coordToId(Eigen::Vector3i pose) const {
  return coordToId(pose[0], pose[1], pose[2]);
}

bool GraphSearch::isFree(int x, int y, int z) const {
  const Eigen::Vector3i pn(x,y,z);
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
     edt_env_->getInflateOccupancy(Index2Coord(pn))==false;
}

bool GraphSearch::isOccupied(int x, int y, int z) const {
  const Eigen::Vector3i pn(x,y,z);
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
    edt_env_->getInflateOccupancy(Index2Coord(pn)) == true;
}

double GraphSearch::getHeur(int x, int y, int z) const {
  return eps_ * std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_) + (z - zGoal_) * (z - zGoal_));
}

Eigen::Vector3d GraphSearch::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

bool GraphSearch::checkOccupancy(const Eigen::Vector3d &pos) { 
  return (bool)edt_env_->getInflateOccupancy(pos); 
}

bool GraphSearch::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        // ROS_WARN("Start point is insdide an obstacle. a");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // ROS_WARN("End point is insdide an obstacle. a");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }
    return true;
}



bool GraphSearch::isBlocked(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int8_t val) {
  std::vector<Eigen::Vector3i> pns = rayTrace(p1, p2);
  for (const auto &pn : pns) {
    if(checkOccupancy(Index2Coord(pn))){
      return true;
    }
  }
  return false;
}

std::vector<Eigen::Vector3i> GraphSearch::rayTrace(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2) {
  Eigen::Vector3d diff = pt2 - pt1;
  double k = 0.8;
  int max_diff = (diff / res_).lpNorm<Eigen::Infinity>() / k;
  double s = 1.0 / max_diff;
  Eigen::Vector3d step = diff * s;

  std::vector<Eigen::Vector3i> pns;
  Eigen::Vector3i prev_pn = Eigen::Vector3i::Constant(-1);
  for (int n = 1; n < max_diff; n++) {
    Eigen::Vector3d pt = pt1 + step * n;
    Eigen::Vector3i new_pn;
    bool inside = Coord2Index(pt, new_pn);
    if (inside == false)
      break;
    if (new_pn != prev_pn)
      pns.push_back(new_pn);
    prev_pn = new_pn;
  }
  return pns;
}

bool GraphSearch::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;
	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("JSP Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}
	return true;
};


bool GraphSearch::isOutside(Eigen::Vector3i idx){
  if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		return true;
	}
	return false;
}

bool GraphSearch::plan( Eigen::Vector3i start_pt, Eigen::Vector3i end_pt, bool useJps, int maxExpand)
{
  pq_.clear();
  path_.clear();
  hm_.resize(xDim_ * yDim_ * zDim_);
  std::cout<< "=====1: " << std::endl;

  int goal_id = coordToId(end_pt[0], end_pt[1], end_pt[2]);
  xGoal_ = end_pt[0]; yGoal_ = end_pt[1]; zGoal_ = end_pt[2];
  int start_id = coordToId(start_pt[0], start_pt[1], start_pt[2]);
  StatePtr currNode_ptr = std::make_shared<State>(State(start_id, start_pt[0], start_pt[1], start_pt[2]
            , 0, 0, 0));
  currNode_ptr->g = 
  currNode_ptr->h = getHeur(start_pt[0], start_pt[1], start_pt[2]);
  return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

bool GraphSearch::plan(StatePtr& currNode_ptr, int maxExpand, int start_id, int goal_id) {
  std::vector<bool> vector2(xDim_ * yDim_ * zDim_, 0);
  seen_ = vector2;
  currNode_ptr->heapkey = pq_.push(currNode_ptr);
  currNode_ptr->opened = true;
  hm_[currNode_ptr->id] = currNode_ptr;
  seen_[currNode_ptr->id] = true;
  int expand_iteration = 0;
  while(true)
  {
    expand_iteration++;
    currNode_ptr = pq_.top(); pq_.pop();
    currNode_ptr->closed = true; // Add to closed list
    if(currNode_ptr->id == goal_id) {
      if(verbose_)
        printf("Goal Reached!!!!!!\n\n");
      break;
    }

    std::vector<int> succ_ids;
    std::vector<double> succ_costs;
    getJpsSucc(currNode_ptr, succ_ids, succ_costs);
    if(verbose_)
    for( int s = 0; s < (int) succ_ids.size(); s++ )
    {
      if(succ_ids[s] >= max_size){
        std::cout<< "Out of map..." << std::endl;
        continue;
      }
      StatePtr& child_ptr = hm_[succ_ids[s]];
      double tentative_gval = currNode_ptr->g + succ_costs[s];
      
      if( tentative_gval < child_ptr->g )
      {
        child_ptr->parentId = currNode_ptr->id;  // Assign new parent
        child_ptr->g = tentative_gval;    // Update gval
        if( child_ptr->opened && !child_ptr->closed) {
          pq_.increase( child_ptr->heapkey );      
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          child_ptr->dz = (child_ptr->z - currNode_ptr->z);
          if(child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if(child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
           if(child_ptr->dz != 0)
            child_ptr->dz /= std::abs(child_ptr->dz);
        }
        else if( child_ptr->opened && child_ptr->closed)
        {
          path_ = recoverPath(currNode_ptr, start_id);
          printf("ASTAR ERROR!\n");
        }
        else // new node, add to heap
        {
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } //
    } // Process successors

    if(maxExpand > 0 && expand_iteration >= maxExpand) {
      if(verbose_)
        printf("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
      return false;
    }
    if( pq_.empty()) {
      if(verbose_)
        printf("Priority queue is empty!!!!!!\n\n");
      return false;
    }
  }

  if(verbose_) {
    printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
    printf("Expand [%d] nodes!\n", expand_iteration);
  }

  path_ = recoverPath(currNode_ptr, start_id);

  return true;
}

void GraphSearch::getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs) {
    const int norm1 = std::abs(curr->dx)+std::abs(curr->dy)+std::abs(curr->dz);
    int num_neib = jn3d_->nsz[norm1][0];
    int num_fneib = jn3d_->nsz[norm1][1];
    int id = (curr->dx+1)+3*(curr->dy+1)+9*(curr->dz+1);
    for( int dev = 0; dev < num_neib+num_fneib; ++dev) {
      int new_x, new_y, new_z;
      int dx, dy, dz;
      if(dev < num_neib) {
        dx = jn3d_->ns[id][0][dev];
        dy = jn3d_->ns[id][1][dev];
        dz = jn3d_->ns[id][2][dev];
        if(!jump(curr->x, curr->y, curr->z,
              dx, dy, dz, new_x, new_y, new_z)) continue;
      }
      else {
        int nx = curr->x + jn3d_->f1[id][0][dev-num_neib];
        int ny = curr->y + jn3d_->f1[id][1][dev-num_neib];
        int nz = curr->z + jn3d_->f1[id][2][dev-num_neib];
        if(isOccupied(nx,ny,nz)) {
          dx = jn3d_->f2[id][0][dev-num_neib];
          dy = jn3d_->f2[id][1][dev-num_neib];
          dz = jn3d_->f2[id][2][dev-num_neib];
          if(!jump(curr->x, curr->y, curr->z,
                dx, dy, dz, new_x, new_y, new_z)) continue;
        }
        else
          continue;
      }

      int new_id = coordToId(new_x, new_y, new_z);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, dx, dy, dz);
        hm_[new_id]->h = getHeur(new_x, new_y, new_z);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
            (new_y - curr->y) * (new_y - curr->y) +
            (new_z - curr->z) * (new_z - curr->z)));
    }
}

bool GraphSearch::jump(int x, int y, int z, int dx, int dy, int dz, int& new_x, int& new_y, int& new_z) {
  new_x = x + dx;
  new_y = y + dy;
  new_z = z + dz;
  if (!isFree(new_x, new_y, new_z))
    return false;

  if (new_x ==  xGoal_ && new_y == yGoal_ && new_z == zGoal_)
    return true;

  if (hasForced(new_x, new_y, new_z, dx, dy, dz))
    return true;

  const int id = (dx+1)+3*(dy+1)+9*(dz+1);
  const int norm1 = std::abs(dx) + std::abs(dy) +std::abs(dz);
  int num_neib = jn3d_->nsz[norm1][0];
  for( int k = 0; k < num_neib-1; ++k )
  {
    int new_new_x, new_new_y, new_new_z;
    if(jump(new_x,new_y,new_z,
          jn3d_->ns[id][0][k], jn3d_->ns[id][1][k], jn3d_->ns[id][2][k],
        new_new_x, new_new_y, new_new_z)) return true;
  }
  return jump(new_x, new_y, new_z, dx, dy, dz, new_x, new_y, new_z);
}

inline bool GraphSearch::hasForced(int x, int y, int z, int dx, int dy, int dz) {
  int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
  int id = (dx+1)+3*(dy+1)+9*(dz+1);
  switch(norm1)
  {
    case 1:
      for( int fn = 0; fn < 8; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    case 2:
      for( int fn = 0; fn < 8; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    case 3:
      for( int fn = 0; fn < 6; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    default:
      return false;
  }
}


std::vector<StatePtr> GraphSearch::getPath() const {
  return path_;
}

std::vector<StatePtr> GraphSearch::getOpenSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->opened && !it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::getCloseSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->closed)
      ss.push_back(it);
  }
  return ss;
}


std::vector<StatePtr> GraphSearch::getAllSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::recoverPath(StatePtr node, int start_id) {
  std::vector<StatePtr> path;
  path.push_back(node);
  while (node && node->id != start_id) {
    node = hm_[node->parentId];
    path.push_back(node);
  }

  return path;
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib() {
  int id = 0;
  for(int dz = -1; dz <= 1; ++dz) {
    for(int dy = -1; dy <= 1; ++dy) {
      for(int dx = -1; dx <= 1; ++dx) {
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        for(int dev = 0; dev < nsz[norm1][0]; ++dev)
          Neib(dx,dy,dz,norm1,dev,
              ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
        for(int dev = 0; dev < nsz[norm1][1]; ++dev)
        {
          FNeib(dx,dy,dz,norm1,dev,
              f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
              f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
        }
        id ++;
      }
    }
  }
}


void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
    int& tx, int& ty, int& tz)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; tz=0; return;
        case 1: tx=-1; ty=0; tz=0; return;
        case 2: tx=0; ty=1; tz=0; return;
        case 3: tx=1; ty=1; tz=0; return;
        case 4: tx=-1; ty=1; tz=0; return;
        case 5: tx=0; ty=-1; tz=0; return;
        case 6: tx=1; ty=-1; tz=0; return;
        case 7: tx=-1; ty=-1; tz=0; return;
        case 8: tx=0; ty=0; tz=1; return;
        case 9: tx=1; ty=0; tz=1; return;
        case 10: tx=-1; ty=0; tz=1; return;
        case 11: tx=0; ty=1; tz=1; return;
        case 12: tx=1; ty=1; tz=1; return;
        case 13: tx=-1; ty=1; tz=1; return;
        case 14: tx=0; ty=-1; tz=1; return;
        case 15: tx=1; ty=-1; tz=1; return;
        case 16: tx=-1; ty=-1; tz=1; return;
        case 17: tx=0; ty=0; tz=-1; return;
        case 18: tx=1; ty=0; tz=-1; return;
        case 19: tx=-1; ty=0; tz=-1; return;
        case 20: tx=0; ty=1; tz=-1; return;
        case 21: tx=1; ty=1; tz=-1; return;
        case 22: tx=-1; ty=1; tz=-1; return;
        case 23: tx=0; ty=-1; tz=-1; return;
        case 24: tx=1; ty=-1; tz=-1; return;
        case 25: tx=-1; ty=-1; tz=-1; return;
      }
    case 1:
      tx = dx; ty = dy; tz = dz; return;
    case 2:
      switch(dev)
      {
        case 0:
          if(dz == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = 0; ty = 0; tz = dz; return;
          }
        case 1:
          if(dx == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = dx; ty = 0; tz = 0; return;
          }
        case 2:
          tx = dx; ty = dy; tz = dz; return;
      }
    case 3:
      switch(dev)
      {
        case 0: tx = dx; ty =  0; tz =  0; return;
        case 1: tx =  0; ty = dy; tz =  0; return;
        case 2: tx =  0; ty =  0; tz = dz; return;
        case 3: tx = dx; ty = dy; tz =  0; return;
        case 4: tx = dx; ty =  0; tz = dz; return;
        case 5: tx =  0; ty = dy; tz = dz; return;
        case 6: tx = dx; ty = dy; tz = dz; return;
      }
  }
}

void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; fz = 0; break;
        case 1: fx= 0; fy=-1; fz = 0; break;
        case 2: fx= 1; fy= 0; fz = 0; break;
        case 3: fx= 1; fy= 1; fz = 0; break;
        case 4: fx= 1; fy=-1; fz = 0; break;
        case 5: fx=-1; fy= 0; fz = 0; break;
        case 6: fx=-1; fy= 1; fz = 0; break;
        case 7: fx=-1; fy=-1; fz = 0; break;
      }
      nx = fx; ny = fy; nz = dz;
      // switch order if different direction
      if(dx != 0){
        fz = fx; fx = 0;
        nz = fz; nx = dx;
      }if(dy != 0){
        fz = fy; fy = 0;
        nz = fz; ny = dy;
      }
      return;
    case 2:
      if(dx == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = 0; ny = dy; nz = -dz;
            return;
          case 1:
            fx = 0; fy = -dy; fz = 0;
            nx = 0; ny = -dy; nz = dz;
            return;
          case 2:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = dz;
            return;
          case 3:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = dz;
            return;
          case 4:
            fx = 1; fy = 0; fz = -dz;
            nx = 1; ny = dy; nz = -dz;
            return;
          case 5:
            fx = 1; fy = -dy; fz = 0;
            nx = 1; ny = -dy; nz = dz;
            return;
          case 6:
            fx = -1; fy = 0; fz = -dz;
            nx = -1; ny = dy; nz = -dz;
            return;
          case 7:
            fx = -1; fy = -dy; fz = 0;
            nx = -1; ny = -dy; nz = dz;
            return;
          // Extras
          case 8:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = 0;
            return;
          case 9:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = 0; nz = dz;
            return;
          case 10:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = 0;
            return;
          case 11:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = 0; nz = dz;
            return;
        }
      }else if(dy == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = dx; ny = 0; nz = -dz;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = 0; nz = dz;
            return;
          case 2:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = dz;
            return;
          case 3:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1;nz = dz;
            return;
          case 4:
            fx = 0; fy = 1; fz = -dz;
            nx = dx; ny = 1; nz = -dz;
            return;
          case 5:
            fx = -dx; fy = 1; fz = 0;
            nx = -dx; ny = 1; nz = dz;
            return;
          case 6:
            fx = 0; fy = -1; fz = -dz;
            nx = dx; ny = -1; nz = -dz;
            return;
          case 7:
            fx = -dx; fy = -1; fz = 0;
            nx = -dx; ny = -1; nz = dz;
            return;
          // Extras
          case 8:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = 0;
            return;
          case 9:
            fx = 0; fy = 1; fz = 0;
            nx = 0; ny = 1; nz = dz;
            return;
          case 10:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1; nz = 0;
            return;
          case 11:
            fx = 0; fy = -1; fz = 0;
            nx = 0; ny = -1; nz = dz;
            return;
        }
      }else{// dz==0
        switch(dev)
        {
          case 0:
            fx = 0; fy = -dy; fz = 0;
            nx = dx; ny = -dy; nz = 0;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = dy; nz = 0;
            return;
          case 2:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = dy; nz = 1;
            return;
          case 3:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = dy; nz = -1;
            return;
          case 4:
            fx = 0; fy = -dy; fz = 1;
            nx = dx; ny = -dy; nz = 1;
            return;
          case 5:
            fx = -dx; fy = 0; fz = 1;
            nx = -dx; ny = dy; nz = 1;
            return;
          case 6:
            fx = 0; fy = -dy; fz = -1;
            nx = dx; ny = -dy; nz = -1;
            return;
          case 7:
            fx = -dx; fy = 0; fz = -1;
            nx = -dx; ny = dy; nz = -1;
            return;
          // Extras
          case 8:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = 0; nz = 1;
            return;
          case 9:
            fx = 0; fy = 0; fz = 1;
            nx = 0; ny = dy; nz = 1;
            return;
          case 10:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = 0; nz = -1;
            return;
          case 11:
            fx = 0; fy = 0; fz = -1;
            nx = 0; ny = dy; nz = -1;
            return;
        }
      }
    case 3:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = dz;
          return;
        case 1:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = dz;
          return;
        case 2:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = dy; nz = -dz;
          return;
        // Need to check up to here for forced!
        case 3:
          fx = 0; fy = -dy; fz = -dz;
          nx = dx; ny = -dy; nz = -dz;
          return;
        case 4:
          fx = -dx; fy = 0; fz = -dz;
          nx = -dx; ny = dy; nz = -dz;
          return;
        case 5:
          fx = -dx; fy = -dy; fz = 0;
          nx = -dx; ny = -dy; nz = dz;
          return;
        // Extras
        case 6:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = 0; nz = dz;
          return;
        case 7:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = 0;
          return;
        case 8:
          fx = 0; fy = -dy; fz = 0;
          nx = 0; ny = -dy; nz = dz;
          return;
        case 9:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = 0;
          return;
        case 10:
          fx = 0; fy = 0; fz = -dz;
          nx = 0; ny = dy; nz = -dz;
          return;
        case 11:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = 0; nz = -dz;
          return;
      }
  }
}





