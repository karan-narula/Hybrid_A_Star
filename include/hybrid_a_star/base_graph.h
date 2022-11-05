#ifndef BASE_GRAPH
#define BASE_GRAPH

/*
 * \file base_graph.h
 * \brief This is an abstract class on which various discrete and continous
 * graphs will be built. It simply outlines the required methods necessary for
 * A-star algorithm
 */

#include "utils.h"
#include <Eigen/Core>
#include <functional>
#include <mutex>
#include <string>

namespace hybrid_a_star {
namespace path_planning {
template <typename cost_map_type> class A_Star;
}

template <typename cost_map_type> class Base_Graph {
public:
  // useful for 2d coorddinates
  typedef Eigen::Vector2d Coordinate2d;

protected:
  // pointers to parameters
  const parameters::graph *graph_param;
  const parameters::vehicle *v;
  const parameters::vehicle_motion *vm;
  const kinematic_models::gen_kinematic_func model;

  // number of rows and columns of 2D grid
  int nrows;
  int ncols;
  int total_cells;

  // one member will be the costmap which takes in arbitarily type depending on
  // inheritance
  std::shared_ptr<cost_map_type> cost_map;

  // mutex for locking shared costmap (if necessary)
  std::shared_ptr<std::mutex> costmap_mtx_;

  // loaded and offline file before?
  bool loaded_flag;

  // minimum and maximum traversing velocities
  float min_vel, max_vel;

public:
  // constructor
  Base_Graph(const parameters::graph &graph_param, const parameters::vehicle &v,
             const parameters::vehicle_motion &vm,
             const kinematic_models::gen_kinematic_func model)
      : model(model) {
    this->graph_param = &graph_param;
    this->v = &v;
    this->vm = &vm;
    // this->model = model;
    loaded_flag = false;
    min_vel = vm.min_vel;
    max_vel = min_vel;
  }

  // some setters (in case parameters were changed)
  void setGraphParam(const parameters::graph &graph_param) {
    this->graph_param = &graph_param;
  }
  void setVehicleParam(const parameters::vehicle &v) { this->v = &v; }
  void setVehicleMotionParam(const parameters::vehicle_motion &vm) {
    this->vm = &vm;
  }
  void setVelRange(const float &min_vel, const float &max_vel) {
    this->min_vel = min_vel;
    this->max_vel = max_vel;
  }
  void SetMaxVel(const float &max_vel) { this->max_vel = max_vel; }
  void SetMinVel(const float &min_vel) { this->min_vel = min_vel; }

  // function to return speed limits
  float getMaxVel() const { return max_vel; }
  float getMinVel() const { return min_vel; }

  // function to return total number of cells
  int getTotalCells() const { return total_cells; }

  // function to return number of rows
  int getNrows() const { return nrows; }

  // function to return number of columns
  int getNcols() const { return ncols; }

  // function to return minimum turning radius based on vehicle motion
  // parameters
  float getMinTurnR() const { return vm->min_turn_r; }

  // function to return turning radius based on provided steering angle
  float getTurnR(const float &steering_angle) const {
    return v->wheelbase / tan(fabs(steering_angle));
  }

  // function to return reference to cost map
  cost_map_type &getCostmap() const { return *cost_map; }

  // function to return reference to graph param
  const parameters::graph &getGraphParam() const { return *graph_param; }

  // function to return speed limit of the current position
  virtual float getSpeedLim(const rob_pos &pos) const { return max_vel; }
  virtual void getSpeedLim(rob_pos &pos) const { pos.velocity = max_vel; }

  // function to get index of the position
  virtual int getIndex(const rob_pos &pos, bool &success) const = 0;

  // function to get index for higher dimensional OG
  virtual int getIndexHigher(const rob_pos &pos, bool &success) const {
    // by default is just getIndex, needs to be overriden otherwise
    return getIndex(pos, success);
  }

  // function to get position based on index
  virtual rob_pos getPosition(const int &index, const float &time,
                              bool &success) const = 0;
  virtual rob_pos getPosition(const int &index, const unsigned int &time_index,
                              const float &time_frac, bool &success) const = 0;

  // check if the queried cell is inbound
  virtual bool inBound(const int &index) const = 0;
  virtual bool inBound(const rob_pos &c_pos) const = 0;
  virtual bool inBound(const Coordinate2d &coord) const = 0;

  // check the value in the cell
  virtual float getValue(const int &index, const unsigned int &time_index,
                         const float &time_frac) const = 0;
  virtual float getValue(const int &index, const float &time) const = 0;
  virtual float getValue(const rob_pos &c_pos) const = 0;
  virtual float getValue(const Coordinate2d &coord,
                         const unsigned int &time_index,
                         const float &time_frac) const = 0;
  virtual float getValue(const Coordinate2d &coord,
                         const float &time) const = 0;

  // check if the queried cell is free to move
  virtual bool passable(const int &index, const unsigned int &time_index,
                        const float &time_frac) const = 0;
  virtual bool passable(const int &index, const float &time) const = 0;
  virtual bool passable(const rob_pos &c_pos) const = 0;
  virtual bool passable(const Coordinate2d &coord,
                        const unsigned int &time_index,
                        const float &time_frac) const = 0;
  virtual bool passable(const Coordinate2d &coord, const float &time) const = 0;

  // check if queried cell is traversable
  bool traversable(const int &index, const unsigned int &time_index,
                   const float &time_frac) const {
    return passable(index, time_index, time_frac) && inBound(index);
  }
  bool traversable(const int &index, const float &time) const {
    return passable(index, time) && inBound(index);
  }
  bool traversable(const rob_pos &c_pos) const {
    return passable(c_pos) && inBound(c_pos);
  }

  // check if temporal information is valid within the graph
  virtual bool temporallyValid(const unsigned int &time_index) const = 0;
  virtual bool temporallyValid(const float &time_frac) const { return true; }
  virtual bool temporallyValid(const rob_pos &c_pos) const {
    return temporallyValid(c_pos.time_frac);
  }

  // find time index and fraction from floating time
  virtual void fillTimeIndex(rob_pos &c_pos) const = 0;
  virtual void fillTimeIndex(const float &time, unsigned int &time_index,
                             float &time_frac) const = 0;

  // for locally updating the time index
  virtual void updateTimeIndex(rob_pos &c_pos) const { fillTimeIndex(c_pos); }
  virtual void updateTimeIndex(const float &time, unsigned int &time_index,
                               float &time_frac) const {
    fillTimeIndex(time, time_index, time_frac);
  }

  // check if straight line between two points is collision free
  virtual bool checkCollission(const rob_pos &pos1, const rob_pos &pos2,
                               float &total_cost,
                               bool ignore_start = false) const = 0;

  // function to return the next neighbor
  virtual rob_pos getNeighbor(const rob_pos &current, const int &steering_index,
                              bool &success) const = 0;

  // function to verify if the underlying costmap has consistent parameters
  virtual void checkConsistency() const = 0;

  // function to update the total number of cells
  virtual void updateTotalCells() = 0;

  // function to set the costmap and graph parameters
  void setCostmap(cost_map_type *cost_map, bool check_consistency = false) {
    // store the cost map
    this->cost_map.reset(cost_map);
    // check consistency
    if (check_consistency)
      checkConsistency();
    // update the total number of cells based on the new cost map
    updateTotalCells();
  }
  void setCostmap(cost_map_type *cost_map, const parameters::graph &graph_param,
                  bool check_consistency = false) {
    // store the cost map
    this->cost_map.reset(cost_map);
    this->graph_param = &graph_param;
    // check consistency
    if (check_consistency)
      checkConsistency();
    // update the total number of cells based on the new cost map
    updateTotalCells();
  }
  // same functions as above but accepting shared pointers
  void setCostmap(const std::shared_ptr<cost_map_type> &cost_map,
                  bool check_consistency = false) {
    // store the cost map
    this->cost_map = cost_map;
    // check consistency
    if (check_consistency)
      checkConsistency();
    // update the total number of cells based on the new cost map
    updateTotalCells();
  }
  void setCostmap(const std::shared_ptr<cost_map_type> &cost_map,
                  const parameters::graph &graph_param,
                  bool check_consistency = false) {
    // store the cost map
    this->cost_map = cost_map;
    this->graph_param = &graph_param;
    // check consistency
    if (check_consistency)
      checkConsistency();
    // update the total number of cells based on the new cost map
    updateTotalCells();
  }

  // allow setting shared object to mutex for locking shared costmap
  void setMutex(std::mutex *costmap_mtx) { costmap_mtx_ = costmap_mtx; }
  void setMutex(const std::shared_ptr<std::mutex> &costmap_mtx) {
    costmap_mtx_ = costmap_mtx;
  }

  // for debuggin purposes, it is useful to have a function load costmap from a
  // file
  virtual void loadCostmap(const std::string &filename,
                           const std::string &scenario) = 0;

  // find nearest obstacle within radius
  virtual bool nearest_obstacle(const Coordinate2d &current,
                                const unsigned int &time_index,
                                const float &time_frac, const float &radius,
                                Coordinate2d &obs_pos,
                                float &distance) const = 0;
  virtual bool nearest_obstacle(const Coordinate2d &current, const float &time,
                                const float &radius, Coordinate2d &obs_pos,
                                float &distance) const = 0;

  // virtual destructor to correctly deallocate memory when upcasting
  virtual ~Base_Graph(){};

  // make the a_star class a friend to enable access to private and protected
  // members
  friend class path_planning::A_Star<cost_map_type>;
};

} // namespace hybrid_a_star

#endif
