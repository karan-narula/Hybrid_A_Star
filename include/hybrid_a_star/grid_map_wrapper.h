#ifndef GRID_MAP_WRAPPER
#define GRID_MAP_WRAPPER

/*
 * \file grid_map_wrapper.h
 * \brief This provides a class which wraps around grid_map ros package while
 * providing functionality outlined in base_graph
 */

#include "base_graph.h"
#include "grid_map_ros/grid_map_ros.hpp"

namespace hybrid_a_star {

// define common costmap types
typedef grid_map::GridMap Costmap;
typedef std::unique_ptr<grid_map::GridMap> CostmapUPtr;
typedef grid_map::GridMap *CostmapRPtr;
typedef std::shared_ptr<grid_map::GridMap> CostmapSPtr;

class Grid_Map_Wrapper : public Base_Graph<Costmap> {
protected:
  // layer names of aggregate obstacles at different time instances
  std::vector<std::string> layer_names;
  // layer name of just the static obstacle
  std::string static_obs_layer;
  // vector of sorted (increasing order) time instaces corresponding to the
  // layer names
  std::vector<float> times;
  // grid value above which is considered as an obstacle
  float obstacle_threshold;
  // slope interpolating speed based on cost in cost-map
  float speed_linear_slope;
  // interpolate between times?
  bool interpolate_time;

  // front position of the car for collision detection
  grid_map::Position front_center_;

public:
  // additional constructor to take in already created grid mmap
  Grid_Map_Wrapper(const parameters::graph &graph_param,
                   const parameters::vehicle &v,
                   const parameters::vehicle_motion &vm,
                   const kinematic_models::gen_kinematic_func model,
                   CostmapRPtr grid_map,
                   const std::vector<std::string> &layer_names,
                   const std::vector<float> &times,
                   const std::string &static_obs_layer);
  // same as above but with shared pointer
  Grid_Map_Wrapper(const parameters::graph &graph_param,
                   const parameters::vehicle &v,
                   const parameters::vehicle_motion &vm,
                   const kinematic_models::gen_kinematic_func model,
                   const CostmapSPtr &grid_map,
                   const std::vector<std::string> &layer_names,
                   const std::vector<float> &times,
                   const std::string &static_obs_layer);

  // constructor without grid map
  Grid_Map_Wrapper(const parameters::graph &graph_param,
                   const parameters::vehicle &v,
                   const parameters::vehicle_motion &vm,
                   const kinematic_models::gen_kinematic_func model,
                   const std::vector<std::string> &layer_names,
                   const std::vector<float> &times,
                   const std::string &static_obs_layer);

  // constructor without grid map, without layername
  Grid_Map_Wrapper(const parameters::graph &graph_param,
                   const parameters::vehicle &v,
                   const parameters::vehicle_motion &vm,
                   const kinematic_models::gen_kinematic_func model);

  // allow setting the layername separately
  void setAllLayerNames(const std::vector<std::string> &layer_names,
                        const std::vector<float> &times,
                        const std::string &static_obs_layer);
  void setStaticLayerName(const std::string &static_obs_layer);
  void setAggLayerNames(const std::vector<std::string> &layer_names,
                        const std::vector<float> &times);

  // function to check consistency between underlying costmap and the graph
  // parameters
  virtual void checkConsistency() const;

  // function to update the total number of cells
  virtual void updateTotalCells();

  // function to set obstacle threshold
  void setObstacleThreshold(float threshold);

  // function to set speed limits
  void setVelRange(const float &min_vel, const float &max_vel);
  void SetMaxVel(const float &max_vel);
  void SetMinVel(const float &min_vel);

  // function to set whether to interpolate between the layers based on time
  void setInterpolTime(const bool &interpolate_time);

  // function to get whether to interpolate between the layers based on time
  bool getInterpolTime();

  // useful function to convert single index to grid_map's index
  void getGridIndex(grid_map::Index &grid_index, const int &index) const;

  // function to return speed limit of the current position
  virtual float getSpeedLim(const rob_pos &pos) const;
  virtual void getSpeedLim(rob_pos &pos) const;

  // function to get index of the position
  int getIndex(const grid_map::Index &grid_index) const;
  virtual int getIndex(const rob_pos &pos, bool &success) const;

  // function to get position based on index
  rob_pos getPosition(grid_map::Index &grid_index, const float &time,
                      bool &success) const;
  rob_pos getPosition(grid_map::Index &grid_index,
                      const unsigned int &time_index, const float &time_frac,
                      bool &success) const;
  virtual rob_pos getPosition(const int &index, const float &time,
                              bool &success) const;
  virtual rob_pos getPosition(const int &index, const unsigned int &time_index,
                              const float &time_frac, bool &success) const;

  // check if the queried cell is inbound
  virtual bool inBound(const int &index) const;
  virtual bool inBound(const rob_pos &c_pos) const;
  virtual bool inBound(const Coordinate2d &coord) const;

  // check the value in the cell
  virtual float getValue(const int &index, const unsigned int &time_index,
                         const float &time_frac) const;
  virtual float getValue(const int &index, const float &time) const;
  virtual float getValue(const rob_pos &c_pos) const;
  virtual float getValue(const Coordinate2d &coord,
                         const unsigned int &time_index,
                         const float &time_frac) const;
  virtual float getValue(const Coordinate2d &coord, const float &time) const;

  // retrieve value from static obstacle layer
  float getStaticValue(const int &index, const unsigned int &time_index,
                       const float &time_frac) const;
  float getStaticValue(const int &index, const float &time) const;
  float getStaticValue(const rob_pos &c_pos) const;
  float getStaticValue(const Coordinate2d &coord,
                       const unsigned int &time_index,
                       const float &time_frac) const;
  float getStaticValue(const Coordinate2d &coord, const float &time) const;

  // check if the queried cell is free to move
  bool passable(const float &grid_value) const;
  virtual bool passable(const Coordinate2d &coord,
                        const unsigned int &time_index,
                        const float &time_frac) const;
  virtual bool passable(const Coordinate2d &coord, const float &time) const;
  virtual bool passable(const int &index, const unsigned int &time_index,
                        const float &time_frac) const;
  virtual bool passable(const int &index, const float &time) const;
  virtual bool passable(const rob_pos &c_pos) const;

  // check if temporal information is valid within the graph
  virtual bool temporallyValid(const unsigned int &time_index) const;
  virtual bool temporallyValid(const float &time_frac) const;
  virtual bool temporallyValid(const rob_pos &c_pos) const;

  // find time index and fraction from floating time
  virtual void fillTimeIndex(rob_pos &c_pos) const;
  virtual void fillTimeIndex(const float &time, unsigned int &time_index,
                             float &time_frac) const;

  // for locally updating the time index
  virtual void updateTimeIndex(rob_pos &c_pos) const;
  virtual void updateTimeIndex(const float &time, unsigned int &time_index,
                               float &time_frac) const;

  // check if straight line between two points is collision free
  virtual bool checkCollission(const rob_pos &pos1, const rob_pos &pos2,
                               float &total_cost,
                               bool ignore_start = false) const;

  // get neighbor from the current position
  virtual rob_pos getNeighbor(const rob_pos &current, const int &steering_index,
                              bool &success) const;

  // for debuggin purposes, it is useful to have a function load costmap from a
  // file
  void loadCostmap(const std::string &filename, const std::string &scenario);

  // find nearest obstacle within radius
  virtual bool nearest_obstacle(const Coordinate2d &current,
                                const unsigned int &time_index,
                                const float &time_frac, const float &radius,
                                Coordinate2d &obs_pos, float &distance) const;
  virtual bool nearest_obstacle(const Coordinate2d &current, const float &time,
                                const float &radius, Coordinate2d &obs_pos,
                                float &distance) const;

  // virtual destructor to deallocate graph-related stuff which were created by
  // loadCostmap
  virtual ~Grid_Map_Wrapper();

  // the following are for debuggin only! In real case, there shouldn't be a
  // need to set the value of the cost map
  void setValue(const int &index, const float &time, float value);
  void setValue(const rob_pos &c_pos, float value);
  void print();

private:
  // update the slope for interpolating speed
  void updateLinearVelSlope();
};

// useful function to get starting and goal positions from yaml file (works only
// with Grid_Map_Wrapper and the inherited objects
void getStartGoalYaml(const Grid_Map_Wrapper &gmw, rob_pos &start_pos,
                      rob_pos &goal_pos, const std::string &filename,
                      const std::string &scenario, bool &success);

} // namespace hybrid_a_star

#endif
