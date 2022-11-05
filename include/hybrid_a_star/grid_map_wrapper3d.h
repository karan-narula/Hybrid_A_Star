#ifndef GRID_MAP_WRAPPER3D
#define GRID_MAP_WRAPPER3D

/*
 * \file grid_map_wrapper3d.h
 * \brief This extends the grid_map_wrapper class to allow for the extra third
 * dimension.
 */

#include "grid_map_wrapper.h"

namespace hybrid_a_star {
class Grid_Map_Wrapper3D : public Grid_Map_Wrapper {
protected:
  int nheads;
  bool bloated_obstacles;
  int num_divisions;

public:
  // constructor
  Grid_Map_Wrapper3D(const parameters::graph &graph_param,
                     const parameters::vehicle &v,
                     const parameters::vehicle_motion &vm,
                     const kinematic_models::gen_kinematic_func model,
                     CostmapRPtr grid_map,
                     const std::vector<std::string> &layer_names,
                     const std::vector<float> &times,
                     const std::string &static_obs_layer);

  // alternate one with shared pointer
  Grid_Map_Wrapper3D(const parameters::graph &graph_param,
                     const parameters::vehicle &v,
                     const parameters::vehicle_motion &vm,
                     const kinematic_models::gen_kinematic_func model,
                     const CostmapSPtr &grid_map,
                     const std::vector<std::string> &layer_names,
                     const std::vector<float> &times,
                     const std::string &static_obs_layer);

  // alternate constructor without gridmap
  Grid_Map_Wrapper3D(const parameters::graph &graph_param,
                     const parameters::vehicle &v,
                     const parameters::vehicle_motion &vm,
                     const kinematic_models::gen_kinematic_func model,
                     const std::vector<std::string> &layer_names,
                     const std::vector<float> &times,
                     const std::string &static_obs_layer);

  // alternate constructor without gridmap and layername
  Grid_Map_Wrapper3D(const parameters::graph &graph_param,
                     const parameters::vehicle &v,
                     const parameters::vehicle_motion &vm,
                     const kinematic_models::gen_kinematic_func model);
  // update all object's parameters (to be invoked when the references' have
  // been modified)
  void updateParams();
  // set bloated obstacle to enable extra processing avoid going through
  // obstacle
  void setBloatedObs(bool bloated_obstacles);

  // function to update the total number of cells
  virtual void updateTotalCells();

  // helper function to split the index
  inline void splitIndex(const int &index, int &layer_num,
                         int &remainder) const;

  // function to return number of headings
  int getNheads() const;

  // allow using inherited methods which are otherwise hidden due to overload
  using Grid_Map_Wrapper::getPosition;
  using Grid_Map_Wrapper::getValue;
  using Grid_Map_Wrapper::inBound;
  using Grid_Map_Wrapper::passable;

  // rewrite function that inputs and outputs index
  virtual int getIndexHigher(const rob_pos &pos, bool &success) const;
  virtual rob_pos getPosition(const int &index, const float &time,
                              bool &success) const;
  virtual rob_pos getPosition(const int &index, const unsigned int &time_index,
                              const float &time_frac, bool &success) const;
  virtual bool inBound(const int &index) const;
  virtual float getValue(const int &index, const unsigned int &time_index,
                         const float &time_frac) const;
  virtual float getValue(const int &index, const float &time) const;
  virtual bool passable(const int &index, const unsigned int &time_index,
                        const float &time_frac) const;
  virtual bool passable(const int &index, const float &time) const;

  // additional functionality to the loadCostmap function
  virtual void loadCostmap(const std::string &filename,
                           const std::string &scenario);
  // additional functionality of top of the usual get neighbor from
  // Grid_Map_Wrapper
  virtual rob_pos getNeighbor(const rob_pos &current, const int &steering_index,
                              bool &success) const;
};

} // namespace hybrid_a_star

#endif
