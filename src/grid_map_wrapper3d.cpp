#include "hybrid_a_star/grid_map_wrapper3d.h"

using namespace grid_map;

namespace hybrid_a_star {

Grid_Map_Wrapper3D::Grid_Map_Wrapper3D(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model, CostmapRPtr grid_map,
    const std::vector<std::string> &layer_names,
    const std::vector<float> &times, const std::string &static_obs_layer)
    : Grid_Map_Wrapper(graph_param, v, vm, model, grid_map, layer_names, times,
                       static_obs_layer) {
  // make sure that the resolution in heading is non-zero
  assert(graph_param.resolution[2] > 0);

  updateParams();
}

Grid_Map_Wrapper3D::Grid_Map_Wrapper3D(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model,
    const CostmapSPtr &grid_map, const std::vector<std::string> &layer_names,
    const std::vector<float> &times, const std::string &static_obs_layer)
    : Grid_Map_Wrapper(graph_param, v, vm, model, grid_map, layer_names, times,
                       static_obs_layer) {
  // make sure that the resolution in heading is non-zero
  assert(graph_param.resolution[2] > 0);

  updateParams();
}

Grid_Map_Wrapper3D::Grid_Map_Wrapper3D(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model,
    const std::vector<std::string> &layer_names,
    const std::vector<float> &times, const std::string &static_obs_layer)
    : Grid_Map_Wrapper(graph_param, v, vm, model, layer_names, times,
                       static_obs_layer) {
  // make sure that the resolution in heading is non-zero
  assert(graph_param.resolution[2] > 0);

  updateParams();
}

Grid_Map_Wrapper3D::Grid_Map_Wrapper3D(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model)
    : Grid_Map_Wrapper(graph_param, v, vm, model) {
  // make sure that the resolution in heading is non-zero
  assert(graph_param.resolution[2] > 0);

  updateParams();
}

void Grid_Map_Wrapper3D::updateParams() {
  // based on the chord length and grid resolution set bloated obstacle
  bloated_obstacles = (vm->chord_l > graph_param->resolution[0]) ? false : true;
  num_divisions =
      static_cast<int>(ceil(1 * vm->chord_l / graph_param->resolution[0]));
}

void Grid_Map_Wrapper3D::setBloatedObs(bool bloated_obstacles) {
  this->bloated_obstacles = bloated_obstacles;
}

void Grid_Map_Wrapper3D::updateTotalCells() {
  Grid_Map_Wrapper::updateTotalCells();

  nheads = static_cast<int>(ceil(2 * M_PI / graph_param->resolution[2]));
  // update the total number of cells
  total_cells = total_cells * nheads;
}

int Grid_Map_Wrapper3D::getNheads() const { return nheads; }

int Grid_Map_Wrapper3D::getIndexHigher(const rob_pos &pos,
                                       bool &success) const {
  // get the preliminary index
  int prelim_index = Grid_Map_Wrapper::getIndex(pos, success);
  // the layer number based on the current heading
  float bounded_heading = pos.theta;
  // stopping_crit::bind_npi_pi(bounded_heading);
  bounded_heading += M_PI;
  int layer_num =
      static_cast<int>(floor(bounded_heading / graph_param->resolution[2]));
  if (layer_num < 0) {
    layer_num = 0;
  } else if (layer_num >= nheads) {
    // std::cout<<"Why is this happening? not binding angle correctly
    // then"<<std::endl;
    success = false;
  }
  // std::cout<<"bounded heading of "<<bounded_heading<<" Layer num:
  // "<<layer_num<<" raw:
  // "<<bounded_heading/graph_param->resolution[2]<<std::endl;
  return layer_num * nrows * ncols + prelim_index;
}

inline void Grid_Map_Wrapper3D::splitIndex(const int &index, int &layer_num,
                                           int &remainder) const {
  // get layer number first
  layer_num = static_cast<int>(index / (nrows * ncols));
  // the remainder will be the two-d index
  remainder = index % (nrows * ncols);
}

rob_pos Grid_Map_Wrapper3D::getPosition(const int &index, const float &time,
                                        bool &success) const {
  // split the index
  int layer_num, remainder;
  splitIndex(index, layer_num, remainder);
  // fill in x and y coordinate using the base's functionality
  rob_pos new_pos = Grid_Map_Wrapper::getPosition(remainder, time, success);
  new_pos.theta = -M_PI + (layer_num + 0.5) * graph_param->resolution[2];
  if (layer_num >= nheads)
    success = false;

  return new_pos;
}

rob_pos Grid_Map_Wrapper3D::getPosition(const int &index,
                                        const unsigned int &time_index,
                                        const float &time_frac,
                                        bool &success) const {
  // split the index
  int layer_num, remainder;
  splitIndex(index, layer_num, remainder);
  // fill in x and y coordinate using the base's functionality
  rob_pos new_pos =
      Grid_Map_Wrapper::getPosition(remainder, time_index, time_frac, success);
  new_pos.theta = -M_PI + (layer_num + 0.5) * graph_param->resolution[2];
  if (layer_num >= nheads)
    success = false;

  return new_pos;
}

bool Grid_Map_Wrapper3D::inBound(const int &index) const {
  // split the index
  int layer_num, remainder;
  splitIndex(index, layer_num, remainder);

  return Grid_Map_Wrapper::inBound(remainder);
}

float Grid_Map_Wrapper3D::getValue(const int &index,
                                   const unsigned int &time_index,
                                   const float &time_frac) const {
  // split the index
  int layer_num, remainder;
  splitIndex(index, layer_num, remainder);

  return Grid_Map_Wrapper::getValue(remainder, time_index, time_frac);
}

float Grid_Map_Wrapper3D::getValue(const int &index, const float &time) const {
  // split the index
  int layer_num, remainder;
  splitIndex(index, layer_num, remainder);

  return Grid_Map_Wrapper::getValue(remainder, time);
}

bool Grid_Map_Wrapper3D::passable(const int &index,
                                  const unsigned int &time_index,
                                  const float &time_frac) const {
  // split the index
  int layer_num, remainder;
  splitIndex(index, layer_num, remainder);

  return Grid_Map_Wrapper::passable(remainder, time_index, time_frac);
}

bool Grid_Map_Wrapper3D::passable(const int &index, const float &time) const {
  // split the index
  int layer_num, remainder;
  splitIndex(index, layer_num, remainder);

  return Grid_Map_Wrapper::passable(remainder, time);
}

void Grid_Map_Wrapper3D::loadCostmap(const std::string &filename,
                                     const std::string &scenario) {
  // call the parent function to load cost map
  Grid_Map_Wrapper::loadCostmap(filename, scenario);

  // modify the number of headings
  nheads = static_cast<int>(ceil(2 * M_PI / graph_param->resolution[2]));
  // update the total number of cells
  total_cells = nrows * ncols * nheads;
  // based on the chord length and grid resolution set bloated obstacle
  bloated_obstacles = (vm->chord_l > graph_param->resolution[0]) ? false : true;
  num_divisions =
      static_cast<int>(ceil(vm->chord_l / graph_param->resolution[0]));
}

rob_pos Grid_Map_Wrapper3D::getNeighbor(const rob_pos &current,
                                        const int &steering_index,
                                        bool &success) const {
  rob_pos neighbor =
      Grid_Map_Wrapper::getNeighbor(current, steering_index, success);

  // update the time index and fraction of the new pose
  Grid_Map_Wrapper::updateTimeIndex(neighbor);

  // update the velocity of next pose based on dynamic constraint and max vel
  float max_inc = (neighbor.time - current.time)*vm->constraint.max_accel;
  vm->apply_gen_const(current.velocity, neighbor.velocity, max_inc);

  // Check for obstacles between front & back centers
  Eigen::Matrix2d neighbor_rotation;
  Position neighbor_back(neighbor.x, neighbor.y);
  neighbor_rotation << cos(neighbor.theta), -sin(neighbor.theta),
      sin(neighbor.theta), cos(neighbor.theta);
  Position neighbor_front = neighbor_rotation * front_center_ + neighbor_back;
  Position temp_pos;
  std::lock_guard<std::mutex> guard(*costmap_mtx_);
  // if neither positions are inside gridmap, return with failure
  if (!cost_map->isInside(neighbor_back) ||
      !cost_map->isInside(neighbor_front)) {
    success = false;
    return neighbor;
  }
  try {
    for (LineIterator it(*cost_map, neighbor_back, neighbor_front);
         !it.isPastEnd(); ++it) {
      if (!cost_map->getPosition(*it, temp_pos)) {
        continue;
      }
      if (!passable(temp_pos, neighbor.time_index, neighbor.time_frac)) {
        success = false;
        return neighbor;
      }
    }
  } catch (const std::invalid_argument &ia) {
    // exception occured due to LineIterator creation
    success = false;
    return neighbor;
  }

  // extra check to see if there are any obstacles in between the current
  // position and the neighboring position
  if (!bloated_obstacles) {
    // Just use Line iterator, fastest way
    Position current_back(current.x, current.y);
    try {
      for (LineIterator it(*cost_map, current_back, neighbor_back);
           !it.isPastEnd(); ++it) {
        cost_map->getPosition(*it, temp_pos);
        if (!passable(temp_pos, current.time_index, current.time_frac)) {
          success = false;
          return neighbor;
        }
      }
    } catch (const std::invalid_argument &ia) {
      // exception occured due to LineIterator creation
      success = false;
      return neighbor;
    }
  }

  return neighbor;
}
} // namespace hybrid_a_star
