#include "hybrid_a_star/grid_map_wrapper.h"

using namespace grid_map;

namespace hybrid_a_star {

// additional constructor
Grid_Map_Wrapper::Grid_Map_Wrapper(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model, CostmapRPtr grid_map,
    const std::vector<std::string> &layer_names,
    const std::vector<float> &times, const std::string &static_obs_layer)
    : Base_Graph(graph_param, v, vm, model) {
  cost_map.reset(grid_map);
  assert(times.size() == layer_names.size());
  assert(times.size() > 0);
  this->layer_names = layer_names;
  this->times = times;
  this->static_obs_layer = static_obs_layer;
  this->obstacle_threshold = INFINITY;
  this->interpolate_time = false;
  if (layer_names.size() == 1) {
    this->layer_names.push_back(layer_names[0]);
    this->times.push_back(times[0] + 1.0);
  }

  // front center of the car
  this->front_center_ << this->v->baselink_to_front, 0;

  // verify if parameters are consistent
  checkConsistency();
  // update the total number of cells
  updateTotalCells();
}

Grid_Map_Wrapper::Grid_Map_Wrapper(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model,
    const CostmapSPtr &grid_map, const std::vector<std::string> &layer_names,
    const std::vector<float> &times, const std::string &static_obs_layer)
    : Base_Graph(graph_param, v, vm, model) {
  assert(times.size() == layer_names.size());
  assert(times.size() > 0);
  this->cost_map = grid_map;
  this->layer_names = layer_names;
  this->times = times;
  this->static_obs_layer = static_obs_layer;
  this->obstacle_threshold = INFINITY;
  this->interpolate_time = false;
  if (layer_names.size() == 1) {
    this->layer_names.push_back(layer_names[0]);
    this->times.push_back(times[0] + 1.0);
  }

  // front center of the car
  this->front_center_ << this->v->baselink_to_front, 0;

  // verify if parameters are consistent
  checkConsistency();
  // update the total number of cells
  updateTotalCells();
}

Grid_Map_Wrapper::Grid_Map_Wrapper(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model,
    const std::vector<std::string> &layer_names,
    const std::vector<float> &times, const std::string &static_obs_layer)
    : Base_Graph(graph_param, v, vm, model) {
  assert(times.size() == layer_names.size());
  assert(times.size() > 0);
  this->cost_map = nullptr;
  this->layer_names = layer_names;
  this->times = times;
  this->static_obs_layer = static_obs_layer;
  this->obstacle_threshold = INFINITY;
  this->interpolate_time = false;
  if (layer_names.size() == 1) {
    this->layer_names.push_back(layer_names[0]);
    this->times.push_back(times[0] + 1.0);
  }

  // front center of the car
  this->front_center_ << this->v->baselink_to_front, 0;
}

Grid_Map_Wrapper::Grid_Map_Wrapper(
    const parameters::graph &graph_param, const parameters::vehicle &v,
    const parameters::vehicle_motion &vm,
    const kinematic_models::gen_kinematic_func model)
    : Base_Graph(graph_param, v, vm, model) {
  this->cost_map = NULL;
  this->obstacle_threshold = INFINITY;
  this->interpolate_time = false;

  // front center of the car
  this->front_center_ << this->v->baselink_to_front, 0;
}

void Grid_Map_Wrapper::setAllLayerNames(
    const std::vector<std::string> &layer_names,
    const std::vector<float> &times, const std::string &static_obs_layer) {
  assert(times.size() == layer_names.size());
  assert(times.size() > 0);
  this->layer_names = layer_names;
  this->times = times;
  this->static_obs_layer = static_obs_layer;
  if (layer_names.size() == 1) {
    this->layer_names.push_back(layer_names[0]);
    this->times.push_back(times[0] + 1.0);
  }
}

void Grid_Map_Wrapper::setStaticLayerName(const std::string &static_obs_layer) {
  this->static_obs_layer = static_obs_layer;
}

void Grid_Map_Wrapper::setAggLayerNames(
    const std::vector<std::string> &layer_names,
    const std::vector<float> &times) {
  assert(times.size() == layer_names.size());
  assert(times.size() > 0);
  this->layer_names = layer_names;
  this->times = times;
  if (layer_names.size() == 1) {
    this->layer_names.push_back(layer_names[0]);
    this->times.push_back(times[0] + 1.0);
  }
}

void Grid_Map_Wrapper::updateLinearVelSlope() {
  speed_linear_slope = (min_vel - max_vel) / obstacle_threshold;
}

void Grid_Map_Wrapper::checkConsistency() const {
  float tolerance = 1e-3;
  // check width and height
  assert(fabs(graph_param->width - cost_map->getLength().y()) <= tolerance);
  assert(fabs(graph_param->height - cost_map->getLength().x()) <= tolerance);
  // check the center of the grid
  assert(
      fabs(graph_param->origin[0] - cost_map->getPosition().x()) <= tolerance &&
      fabs(graph_param->origin[1] - cost_map->getPosition().y()) <= tolerance);
  // check the resolution
  assert(fabs(graph_param->resolution[0] - cost_map->getResolution()) <=
         tolerance);
  assert(graph_param->resolution[0] == graph_param->resolution[1]);
}

void Grid_Map_Wrapper::updateTotalCells() {
  // store the number of rows, columns and the total cells
  nrows = cost_map->getSize()(0);
  ncols = cost_map->getSize()(1);
  total_cells = nrows * ncols;
}

void Grid_Map_Wrapper::setObstacleThreshold(float threshold) {
  this->obstacle_threshold = threshold;
  updateLinearVelSlope();
}

void Grid_Map_Wrapper::setVelRange(const float &min_vel, const float &max_vel) {
  Base_Graph::setVelRange(min_vel, max_vel);
  updateLinearVelSlope();
}

void Grid_Map_Wrapper::SetMaxVel(const float &max_vel) {
  Base_Graph::SetMaxVel(max_vel);
  updateLinearVelSlope();
}

void Grid_Map_Wrapper::SetMinVel(const float &min_vel) {
  Base_Graph::SetMinVel(min_vel);
  updateLinearVelSlope();
}

void Grid_Map_Wrapper::setInterpolTime(const bool &interpolate_time) {
  this->interpolate_time = interpolate_time;
}

bool Grid_Map_Wrapper::getInterpolTime() { return interpolate_time; }

void Grid_Map_Wrapper::getGridIndex(Index &grid_index, const int &index) const {
  grid_index[0] = (int)index / ncols;
  grid_index[1] = index % ncols;
}

float Grid_Map_Wrapper::getSpeedLim(const rob_pos &pos) const {
  // perform linear interpolation between maximum & minimum velocity
  return speed_linear_slope * (getValue(pos) - getStaticValue(pos)) + max_vel;
}

void Grid_Map_Wrapper::getSpeedLim(rob_pos &pos) const {
  // perform linear interpolation between maximum & minimum velocity
  pos.velocity =
      speed_linear_slope * (getValue(pos) - getStaticValue(pos)) + max_vel;
}

int Grid_Map_Wrapper::getIndex(const Index &grid_index) const {
  return grid_index[0] * ncols + grid_index[1];
}

int Grid_Map_Wrapper::getIndex(const rob_pos &pos, bool &success) const {
  Position temp_pos(pos.x, pos.y);
  Index temp_ind;
  success = cost_map->getIndex(temp_pos, temp_ind);

  return getIndex(temp_ind);
}

rob_pos Grid_Map_Wrapper::getPosition(Index &grid_index, const float &time,
                                      bool &success) const {
  Position temp_pos;
  success = cost_map->getPosition(grid_index, temp_pos);

  rob_pos new_pos;
  new_pos.x = temp_pos.x();
  new_pos.y = temp_pos.y();
  new_pos.time = time;
  fillTimeIndex(new_pos.time, new_pos.time_index, new_pos.time_frac);
  getSpeedLim(new_pos);

  return new_pos;
}

rob_pos Grid_Map_Wrapper::getPosition(Index &grid_index,
                                      const unsigned int &time_index,
                                      const float &time_frac,
                                      bool &success) const {
  Position temp_pos;
  success = cost_map->getPosition(grid_index, temp_pos);

  rob_pos new_pos;
  new_pos.x = temp_pos.x();
  new_pos.y = temp_pos.y();
  new_pos.time_index = time_index;
  new_pos.time_frac = time_frac;
  new_pos.time = times[time_index] +
                 time_frac * (times[time_index + 1] - times[time_index]);
  getSpeedLim(new_pos);

  return new_pos;
}

rob_pos Grid_Map_Wrapper::getPosition(const int &index, const float &time,
                                      bool &success) const {
  Index grid_index;
  getGridIndex(grid_index, index);
  return getPosition(grid_index, time, success);
}

rob_pos Grid_Map_Wrapper::getPosition(const int &index,
                                      const unsigned int &time_index,
                                      const float &time_frac,
                                      bool &success) const {
  Index grid_index;
  getGridIndex(grid_index, index);
  return getPosition(grid_index, time_index, time_frac, success);
}

bool Grid_Map_Wrapper::inBound(const int &index) const {
  Position temp_pos;
  Index grid_index;
  getGridIndex(grid_index, index);
  bool success = cost_map->getPosition(grid_index, temp_pos);

  return success;
}

bool Grid_Map_Wrapper::inBound(const rob_pos &c_pos) const {
  Position temp_pos(c_pos.x, c_pos.y);
  return cost_map->isInside(temp_pos);
}

bool Grid_Map_Wrapper::inBound(const Coordinate2d &coord) const {
  return cost_map->isInside(coord);
}

float Grid_Map_Wrapper::getValue(const int &index,
                                 const unsigned int &time_index,
                                 const float &time_frac) const {
  Index grid_index;
  getGridIndex(grid_index, index);
  if (inBound(index)) {
    if (interpolate_time) {
      const float &val0 = cost_map->at(layer_names[time_index], grid_index);
      const float &val1 = cost_map->at(layer_names[time_index + 1], grid_index);
      const float &static_val = cost_map->at(static_obs_layer, grid_index);
      return std::max(static_val, val0 + time_frac * (val1 - val0));
    } else {
      if (time_frac > 1.0) {
        return cost_map->at(layer_names[time_index + 1], grid_index);
      } else {
        return cost_map->at(layer_names[time_index], grid_index);
      }
    }
  } else {
    return INFINITY;
  }
}

float Grid_Map_Wrapper::getValue(const int &index, const float &time) const {
  // get time index and time fraction
  unsigned int time_index;
  float time_frac;
  fillTimeIndex(time, time_index, time_frac);

  return getValue(index, time_index, time_frac);
}

float Grid_Map_Wrapper::getValue(const rob_pos &c_pos) const {
  Position temp_pos(c_pos.x, c_pos.y);

  return getValue(temp_pos, c_pos.time_index, c_pos.time_frac);
}

float Grid_Map_Wrapper::getValue(const Coordinate2d &coord,
                                 const unsigned int &time_index,
                                 const float &time_frac) const {
  if (inBound(coord)) {
    if (interpolate_time) {
      const float &val0 = cost_map->atPosition(layer_names[time_index], coord);
      const float &val1 =
          cost_map->atPosition(layer_names[time_index + 1], coord);
      const float &static_val = cost_map->atPosition(static_obs_layer, coord);
      return std::max(static_val, val0 + time_frac * (val1 - val0));
    } else {
      if (time_frac > 1.0) {
        return cost_map->atPosition(layer_names[time_index + 1], coord);
      } else {
        return cost_map->atPosition(layer_names[time_index], coord);
      }
    }
  } else {
    return INFINITY;
  }
}

float Grid_Map_Wrapper::getValue(const Coordinate2d &coord,
                                 const float &time) const {
  // get time index and time fraction
  unsigned int time_index;
  float time_frac;
  fillTimeIndex(time, time_index, time_frac);

  return getValue(coord, time_index, time_frac);
}

float Grid_Map_Wrapper::getStaticValue(const int &index,
                                       const unsigned int &time_index,
                                       const float &time_frac) const {
  Index grid_index;
  getGridIndex(grid_index, index);
  if (inBound(index)) {
    return cost_map->at(static_obs_layer, grid_index);
  } else {
    return INFINITY;
  }
}

float Grid_Map_Wrapper::getStaticValue(const int &index,
                                       const float &time) const {
  // get time index and time fraction
  unsigned int time_index;
  float time_frac;
  fillTimeIndex(time, time_index, time_frac);

  return getStaticValue(index, time_index, time_frac);
}

float Grid_Map_Wrapper::getStaticValue(const rob_pos &c_pos) const {
  Position temp_pos(c_pos.x, c_pos.y);

  return getStaticValue(temp_pos, c_pos.time_index, c_pos.time_frac);
}

float Grid_Map_Wrapper::getStaticValue(const Coordinate2d &coord,
                                       const unsigned int &time_index,
                                       const float &time_frac) const {
  if (inBound(coord)) {
    return cost_map->atPosition(static_obs_layer, coord);
  } else {
    return INFINITY;
  }
}

float Grid_Map_Wrapper::getStaticValue(const Coordinate2d &coord,
                                       const float &time) const {
  // get time index and time fraction
  unsigned int time_index;
  float time_frac;
  fillTimeIndex(time, time_index, time_frac);

  return getStaticValue(coord, time_index, time_frac);
}

bool Grid_Map_Wrapper::passable(const float &grid_value) const {
  return !(std::isnan(grid_value) || grid_value >= obstacle_threshold ||
           std::isinf(grid_value));
}

bool Grid_Map_Wrapper::passable(const int &index,
                                const unsigned int &time_index,
                                const float &time_frac) const {
  float grid_value = getValue(index, time_index, time_frac);
  return passable(grid_value);
}

bool Grid_Map_Wrapper::passable(const int &index, const float &time) const {
  float grid_value = getValue(index, time);
  return passable(grid_value);
}

bool Grid_Map_Wrapper::passable(const rob_pos &c_pos) const {
  float grid_value = getValue(c_pos);
  return passable(grid_value);
}

bool Grid_Map_Wrapper::passable(const Coordinate2d &coord,
                                const unsigned int &time_index,
                                const float &time_frac) const {
  float grid_value = getValue(coord, time_index, time_frac);
  return passable(grid_value);
}

bool Grid_Map_Wrapper::passable(const Coordinate2d &coord,
                                const float &time) const {
  float grid_value = getValue(coord, time);
  return passable(grid_value);
}

bool Grid_Map_Wrapper::temporallyValid(const unsigned int &time_index) const {
  return time_index < times.size() - 1;
}

bool Grid_Map_Wrapper::temporallyValid(const float &time_frac) const {
  // assumes that time starts after the beginning of the time vector
  return time_frac >= 0.;
}

bool Grid_Map_Wrapper::temporallyValid(const rob_pos &c_pos) const {
  return c_pos.time_index < times.size() && Base_Graph::temporallyValid(c_pos);
}

void Grid_Map_Wrapper::fillTimeIndex(const float &time,
                                     unsigned int &time_index,
                                     float &time_frac) const {
  // default parameters if not already so
  time_index = 0;
  time_frac = 0.;

  // iterate through the time vector
  for (unsigned int i = 1; i < times.size(); i++) {
    if (time > times[i] && i < times.size() - 1) {
      time_index = i;
    } else {
      if (i > 0) {
        time_frac = (time - times[i - 1]) / (times[i] - times[i - 1]);
      }
      break;
    }
  }
}

void Grid_Map_Wrapper::fillTimeIndex(rob_pos &c_pos) const {
  fillTimeIndex(c_pos.time, c_pos.time_index, c_pos.time_frac);
}

void Grid_Map_Wrapper::updateTimeIndex(const float &time,
                                       unsigned int &time_index,
                                       float &time_frac) const {
  for (unsigned int i = time_index + 1; i < times.size() - 1; i++) {
    if (time > times[i]) {
      time_index = i;
    } else {
      break;
    }
  }
  time_frac =
      (time - times[time_index]) / (times[time_index + 1] - times[time_index]);
}

void Grid_Map_Wrapper::updateTimeIndex(rob_pos &c_pos) const {
  updateTimeIndex(c_pos.time, c_pos.time_index, c_pos.time_frac);
}

bool Grid_Map_Wrapper::checkCollission(const rob_pos &pos1, const rob_pos &pos2,
                                       float &total_cost,
                                       bool ignore_start) const {
  Position p1{pos1.x, pos1.y}, p2{pos2.x, pos2.y}, temp;
  std::lock_guard<std::mutex> guard(*costmap_mtx_);
  try {
    float current_time, time_frac,
        slope = (pos2.time - pos1.time) / ((p2 - p1).norm());
    unsigned int time_index = pos1.time_index;
    LineIterator it(*cost_map, p1, p2);
    if (ignore_start && !it.isPastEnd()) {
      ++it;
    }
    for (; !it.isPastEnd(); ++it) {
      cost_map->getPosition(*it, temp);
      // update time index and time fraction
      current_time = slope * ((temp - p1).norm()) + pos1.time;
      updateTimeIndex(current_time, time_index, time_frac);

      // update total cost and check passable
      total_cost += getValue(temp, time_index, time_frac);
      if (!passable(temp, time_index, time_frac)) {
        return true;
      }
    }
  } catch (const std::invalid_argument &ia) {
    // both the points are in the same cell and outside the grid
    return true;
  }
  return false;
}

rob_pos Grid_Map_Wrapper::getNeighbor(const rob_pos &current,
                                      const int &steering_index,
                                      bool &success) const {
  rob_pos neighbor;
  if (steering_index < vm->num_actions) {
    const int sign = 1;
    neighbor = model(current, *vm, *v, steering_index, sign);
  } else {
    const int sign = -1;
    int index = steering_index - vm->num_actions;
    neighbor = model(current, *vm, *v, index, sign);
  }

  // update velocity of the neighbor based on speed limit
  Grid_Map_Wrapper::getSpeedLim(neighbor);

  success = traversable(neighbor);
  return neighbor;
}

void Grid_Map_Wrapper::loadCostmap(const std::string &filename,
                                   const std::string &scenario) {
  // if the file was loaded before, deallocate previous memory
  if (loaded_flag) {
    delete graph_param;
    graph_param = NULL;
    cost_map = NULL;
  }

  // check if reference to mutex is present
  if (!costmap_mtx_) {
    costmap_mtx_ = std::make_shared<std::mutex>();
  }
  // check if reference to costmap is present
  std::vector<std::string> all_layers = layer_names;
  all_layers.push_back(static_obs_layer);
  if (!cost_map) {
    cost_map = std::make_shared<Costmap>();
    for (const std::string &layer_name : all_layers) {
      cost_map->add(layer_name, 0.);
    }
  }

  // create a new graph parameter from loading the yaml file
  parameters::graph *new_graph_param = new parameters::graph;
  *new_graph_param = parameters::getGraphParamYaml(
      filename, scenario, parameters::default_scenario, nrows, ncols);

  // file in some details from the graph parameter
  Length length;
  Position origin;
  // origin field
  origin.x() = new_graph_param->origin[0];
  origin.y() = new_graph_param->origin[1];
  // length field
  length[0] = new_graph_param->height;
  length[1] = new_graph_param->width;

  // update total number of cells
  total_cells = nrows * ncols;

  // change properties of cost map based on the parameter
  for (const std::string &layer_name : all_layers) {
    if (!cost_map->exists(layer_name)) {
      // remove all other layers first
      for (auto c_layer_name : cost_map->getLayers()) {
        cost_map->erase(c_layer_name);
      }
      for (const std::string &n_layer_name : all_layers) {
        cost_map->add(n_layer_name, 0.);
      }
      break;
    }
  }
  cost_map->setGeometry(length, new_graph_param->resolution[0], origin);

  // initialise all elements of grid to zeros
  for (const std::string &layer_name : all_layers) {
    Matrix &grid = cost_map->get(layer_name);
    grid.setConstant(0.0);
  }
  Matrix &grid = cost_map->get(static_obs_layer);
  // put obstacles where need be
  YAML::Node node = YAML::LoadFile(filename);
  std::string use_key =
      node[scenario]["obs_loc"] ? scenario : parameters::default_scenario;
  for (std::size_t i = 0; i < node[use_key]["obs_loc"].size(); i++) {
    int start_row_from = node[use_key]["obs_loc"][i][0].as<int>();
    int start_col_from = node[use_key]["obs_loc"][i][2].as<int>();
    int num_rows =
        node[use_key]["obs_loc"][i][1].as<int>() - start_row_from + 1;
    int num_cols =
        node[use_key]["obs_loc"][i][3].as<int>() - start_col_from + 1;
    grid.block(start_row_from, start_col_from, num_rows, num_cols) =
        Eigen::MatrixXf::Constant(num_rows, num_cols, obstacle_threshold);
  }

  // storing into the class member
  graph_param = new_graph_param;

  // check consistency
  checkConsistency();

  loaded_flag = true;
}

bool Grid_Map_Wrapper::nearest_obstacle(const Coordinate2d &current,
                                        const unsigned int &time_index,
                                        const float &time_frac,
                                        const float &radius,
                                        Coordinate2d &obs_pos,
                                        float &distance) const {
  // do a spiral iterator which starts from origin outward
  bool found_obs = false;
  std::lock_guard<std::mutex> guard(*costmap_mtx_);
  for (SpiralIterator it(*cost_map, current, radius); !it.isPastEnd(); ++it) {
    // get the position of this obstacle
    cost_map->getPosition(*it, obs_pos);
    // check if is obstacle
    if (!passable(obs_pos, time_index, time_frac)) {
      /*
      // adjust it, pessimistically, by the resolution parameter
      Coordinate2d segment = current - obs_pos;
      double scale =
          std::sqrt(2) * graph_param->resolution[0] / segment.dot(segment);
      if (scale < 1.0) {
        obs_pos = obs_pos + scale * segment;
        // calculate the distance from current position
        distance = std::hypot(obs_pos[0] - current[0], obs_pos[1] - current[1]);
      } else {
        obs_pos = current;
        distance = 0;
      }
      */

      // calculate the distance from current position
      distance = std::hypot(obs_pos[0] - current[0], obs_pos[1] - current[1]);

      // stop searching, already found
      found_obs = true;
      return found_obs;
    }
  }

  return found_obs;
}

// find nearest obstacle within radius
bool Grid_Map_Wrapper::nearest_obstacle(const Coordinate2d &current,
                                        const float &time, const float &radius,
                                        Coordinate2d &obs_pos,
                                        float &distance) const {
  // retrieve the time index and time frac from time
  unsigned int time_index;
  float time_frac;
  fillTimeIndex(time, time_index, time_frac);

  return nearest_obstacle(current, time_index, time_frac, radius, obs_pos,
                          distance);
}

Grid_Map_Wrapper::~Grid_Map_Wrapper() {
  if (loaded_flag) {
    delete graph_param;
  }
}

void Grid_Map_Wrapper::setValue(const int &index, const float &time,
                                float value) {
  // in the strict sense, the condition must be checked
  if (inBound(index)) {
    // get time index and time fraction
    unsigned int time_index;
    float time_frac;
    fillTimeIndex(time, time_index, time_frac);

    // get grid index
    Index grid_index;
    getGridIndex(grid_index, index);

    // set costmap layers
    cost_map->at(layer_names[time_index], grid_index) = value;
  }
}

void Grid_Map_Wrapper::setValue(const rob_pos &c_pos, float value) {
  // in the strict sense, the condition must be checked
  if (inBound(c_pos)) {
    Position temp_pos(c_pos.x, c_pos.y);
    cost_map->atPosition(layer_names[c_pos.time_index], temp_pos) = value;
  }
}

void Grid_Map_Wrapper::print() {
  for (const std::string &layer_name : layer_names) {
    std::cout << "\nLayer name: " << layer_name << std::endl
              << cost_map->get(layer_name) << std::endl
              << std::endl;
  }
}

// function to get starting and goal nodes from the yaml file
void getStartGoalYaml(const Grid_Map_Wrapper &gmw, rob_pos &start_pos,
                      rob_pos &goal_pos, const std::string &filename,
                      const std::string &scenario, bool &success) {
  // load the .yaml file based on the filename
  YAML::Node node = YAML::LoadFile(filename);

  // check if the default scenario atleast exists
  std::string default_scenario = "default";
  assert(node[default_scenario]);

  // check if scenario name exists
  assert(node[scenario]);

  std::string use_key =
      node[scenario]["start_ind"] ? scenario : default_scenario;

  Index start_index;
  bool success_start;
  start_index[0] = node[use_key]["start_ind"][0].as<int>();
  start_index[1] = node[use_key]["start_ind"][1].as<int>();
  float time = 0.;
  start_pos = gmw.getPosition(start_index, time, success);
  start_pos.theta =
      (M_PI / 2) + (M_PI / 180) * node[use_key]["start_ind"][2].as<float>();
  stopping_crit::bind_npi_pi(start_pos.theta);

  Index goal_index;
  bool success_goal;
  goal_index[0] = node[use_key]["goal_ind"][0].as<int>();
  goal_index[1] = node[use_key]["goal_ind"][1].as<int>();
  goal_pos = gmw.getPosition(goal_index, time, success);
  goal_pos.theta =
      (M_PI / 2) + (M_PI / 180) * node[use_key]["goal_ind"][2].as<float>();
  stopping_crit::bind_npi_pi(goal_pos.theta);

  success = success_start && success_goal;
}
} // namespace hybrid_a_star
