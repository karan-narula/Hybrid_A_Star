#include "grid_map_ros/grid_map_ros.hpp"
#include <ros/package.h>

#include "hybrid_a_star/a_star.h"
#include "hybrid_a_star/grid_map_wrapper3d.h"
#include "hybrid_a_star/struct_filler.h"

#include <boost/functional/hash.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <unordered_map>

#include <gtest/gtest.h>

using namespace grid_map;

std::mutex g_mutex;

class MoveTest : public ::testing::Test {
protected:
  void SetUp() override {
    // add one default layer
    test_map = new GridMap();
    test_map->add(layer_name[0]);
    // create original origin of the map
    Position origin(0, 0);

    // set the geometry of the map
    test_map->setGeometry(map_size, resolution, origin);

    // fill in unique values in the overlapping region of the original map and
    // the moved map
    float radius = hypot(origin[0] - new_origin[0], origin[1] - new_origin[1]);
    CircleIterator it = CircleIterator(*test_map, new_origin, radius);
    int i;
    for (int i(0); !it.isPastEnd() && i < 20; ++it, ++i) {
      test_map->at(layer_name[0], *it) = i;
      Position current;
      test_map->getPosition(*it, current);
      obstacle_loc[i] = current;
    }
  }
  virtual ~MoveTest() { delete test_map; }

  GridMap *test_map;
  // fixed map size, say 100 x 100
  const Length map_size{100, 100};
  // resolution of the map
  const float resolution = 1.0;
  // layer name of the map
  const std::vector<std::string> layer_name{"layer"};
  const std::vector<float> time{0.};
  // position of the four corner of the grid
  const Position original_top_left{50 - 1e-3, 50 - 1e-3};
  const Position original_bottom_left{-50 + 1e-3, 50 - 1e-3};
  const Position original_top_right{50 - 1e-3, -50 + 1e-3};
  const Position original_bottom_right{-50 + 1e-3, -50 + 1e-3};
  // index of the four corners of the grid
  Index top_left_ind{0, 0};
  Index top_right_ind{0, 99};
  Index bottom_left_ind{99, 0};
  Index bottom_right_ind{99, 99};

  // position of new origin
  Position new_origin{10.0, 20.0};

  // arithmatic for the change in positions
  Position new_top_left = original_top_left + new_origin;
  Position new_bottom_left = original_bottom_left + new_origin;
  Position new_top_right = original_top_right + new_origin;
  Position new_bottom_right = original_bottom_right + new_origin;

  // number of obstacles and the associated positions
  int num_obstacles = 20;
  Position obstacle_loc[20];
};

// overload operator to check equivalence of indices
bool operator==(Index &lhs, Index &rhs) {
  if (lhs[0] == rhs[0] && lhs[1] == rhs[1])
    return true;
  else
    return false;
}

TEST_F(MoveTest, SetUpWorks) {
  // before moving check first that all corner positions are in bounds
  EXPECT_TRUE(this->test_map->isInside(this->original_top_left))
      << "Original Top left coordinate not inside grid";
  EXPECT_TRUE(this->test_map->isInside(this->original_bottom_left))
      << "Original Bottom left coordinate not inside grid ";
  EXPECT_TRUE(this->test_map->isInside(this->original_top_right))
      << "Original Top right coordinate not inside grid ";
  EXPECT_TRUE(this->test_map->isInside(this->original_bottom_right))
      << "Original Bottom right coordinate not inside grid ";

  // similarly check the indices that all the corner points are in bounds
  Position temp_pos;
  EXPECT_TRUE(this->test_map->getPosition(this->top_left_ind, temp_pos))
      << "Original Top left index is not inside grid";
  EXPECT_TRUE(this->test_map->getPosition(this->top_right_ind, temp_pos))
      << "Original Top right index is not inside grid";
  EXPECT_TRUE(this->test_map->getPosition(this->bottom_left_ind, temp_pos))
      << "Original Bottom left index is not inside grid";
  EXPECT_TRUE(this->test_map->getPosition(this->bottom_right_ind, temp_pos))
      << "Original Bottom right index is not inside grid";

  // check if we can get back the same indices from the corner positions
  Index temp_index;
  EXPECT_TRUE(this->test_map->getIndex(this->original_top_left, temp_index));
  EXPECT_TRUE(this->top_left_ind == temp_index);
  EXPECT_TRUE(this->test_map->getIndex(this->original_bottom_left, temp_index));
  EXPECT_TRUE(this->bottom_left_ind == temp_index);
  EXPECT_TRUE(this->test_map->getIndex(this->original_top_right, temp_index));
  EXPECT_TRUE(this->top_right_ind == temp_index);
  EXPECT_TRUE(
      this->test_map->getIndex(this->original_bottom_right, temp_index));
  EXPECT_TRUE(this->bottom_right_ind == temp_index);
}

TEST_F(MoveTest, OriginUpdated) {
  // move the map
  EXPECT_TRUE(this->test_map->move(this->new_origin)) << "move didn't work";

  // check if the origin is infact updated
  auto new_origin_ref = this->test_map->getPosition();
  EXPECT_EQ(this->new_origin, new_origin_ref)
      << "Origin didn't update correctly";
}

TEST_F(MoveTest, OriginalCornerOutBounds) {
  // move the map
  EXPECT_TRUE(this->test_map->move(this->new_origin)) << "move didn't work";

  // check if the modified corner positions are within bounds of the moved map
  EXPECT_TRUE(this->test_map->isInside(this->new_top_left))
      << "Top left coordinate not inside grid";
  EXPECT_TRUE(this->test_map->isInside(this->new_bottom_left))
      << "Bottom left coordinate not inside grid ";
  EXPECT_TRUE(this->test_map->isInside(this->new_top_right))
      << "Top right coordinate not inside grid ";
  EXPECT_TRUE(this->test_map->isInside(this->new_bottom_right))
      << "Bottom right coordinate not inside grid ";

  // similar as above but retrieve the index first
  Index new_top_left_ind, new_top_right_ind, new_bottom_left_ind,
      new_bottom_right_ind;
  EXPECT_TRUE(this->test_map->getIndex(this->new_top_left, new_top_left_ind))
      << "Top left index not inside grid";
  EXPECT_TRUE(
      this->test_map->getIndex(this->new_bottom_left, new_bottom_left_ind))
      << "Bottom left index not inside grid";
  EXPECT_TRUE(this->test_map->getIndex(this->new_top_right, new_top_right_ind))
      << "Top right index not inside grid";
  EXPECT_TRUE(
      this->test_map->getIndex(this->new_bottom_right, new_bottom_right_ind))
      << "Bottom right index not inside grid";

  // check if the original corner points are within the map
  EXPECT_TRUE(this->test_map->isInside(this->original_top_left))
      << "Old top left position is still inside after motion";
  EXPECT_FALSE(this->test_map->isInside(this->original_top_right))
      << "Old top right position should no longer be inside after motion";
  EXPECT_FALSE(this->test_map->isInside(this->original_bottom_left))
      << "Old bottom left position should no longer be inside after motion";
  EXPECT_FALSE(this->test_map->isInside(this->original_bottom_right))
      << "Old bottom right position should no longer be inside after motion";

  // similar to above but check via index
  Index temp_ind;
  EXPECT_TRUE(this->test_map->getIndex(this->original_top_left, temp_ind))
      << "Old top left index is still inside after motion";
  EXPECT_FALSE(this->test_map->getIndex(this->original_top_right, temp_ind))
      << "Old top right index should no longer be inside after motion";
  EXPECT_FALSE(this->test_map->getIndex(this->original_bottom_left, temp_ind))
      << "Old bottom left index should no longer be inside after motion";
  EXPECT_FALSE(this->test_map->getIndex(this->original_bottom_right, temp_ind))
      << "Old bottom right index should no longer be inside after motion";
}

// hash function for index
struct IndexHash {
  std::size_t operator()(const Index &k) const {
    size_t seed = 0;
    boost::hash_combine(seed, k[0]);
    boost::hash_combine(seed, k[1]);
    return seed;
  }
};
struct IndexEqual {
  bool operator()(const Index &lhs, const Index &rhs) const {
    return lhs[0] == rhs[0] && lhs[1] == rhs[1];
  }
};

TEST_F(MoveTest, UniqueIndexForAllCells) {
  // move the map
  EXPECT_TRUE(this->test_map->move(this->new_origin)) << "move didn't work";

  // get index of the top left corner (should be inside)
  Index temp_ind;
  EXPECT_TRUE(this->test_map->getIndex(this->new_top_left, temp_ind))
      << "New top left index is inside the grid";

  // get center of the top left cell from the index
  Position top_left_center;
  EXPECT_TRUE(this->test_map->getPosition(temp_ind, top_left_center))
      << "Can't fetch the center of top left cell from its index";

  // get number of rows and number of columns
  int nrows = this->test_map->getSize()(0);
  int ncols = this->test_map->getSize()(1);

  // iterate through rows and columns and check if index is unique
  std::unordered_map<Index, Position, IndexHash, IndexEqual> all_indices;
  Position current = top_left_center;
  for (int i = 0; i < nrows; i++) {
    for (int j = 0; j < ncols; j++) {
      // get index of current position
      EXPECT_TRUE(this->test_map->getIndex(current, temp_ind))
          << "Can't get index for (" << i << "," << j
          << ") with position: " << current[0] << "," << current[1];

      // thoroughly check that all of these are within the bounds
      EXPECT_TRUE(this->test_map->isInside(current))
          << "Position (" << current[0] << "," << current[1]
          << ") should have been inside";

      // check if this index was already there before
      EXPECT_TRUE(all_indices.count(temp_ind) == 0)
          << "Index (" << temp_ind[0] << "," << temp_ind[1]
          << ") is not unique";

      // put it in
      all_indices.insert({temp_ind, current});

      // move
      current[1] -= this->resolution;
    }
    current[1] = top_left_center[1];
    current[0] -= this->resolution;
  }
}

TEST_F(MoveTest, VerifyContent) {
  // move the map
  EXPECT_TRUE(this->test_map->move(this->new_origin)) << "move didn't work";

  // check that the content in the overlapped region is the same
  for (int i = 0; i < this->num_obstacles; i++) {
    EXPECT_EQ(i, this->test_map->atPosition(this->layer_name[0],
                                            this->obstacle_loc[i]));
  }
}

TEST_F(MoveTest, VerifyNewCornersRawIndex) {
  // move the map
  EXPECT_TRUE(this->test_map->move(this->new_origin)) << "move didn't work";
  // rearrange data so starting index is at (0,0)
  this->test_map->convertToDefaultStartIndex();

  // check if internal variables of grid map is updated correctly by getting
  // positions from index
  Position temp_pos;
  EXPECT_TRUE(this->test_map->getPosition(this->top_left_ind, temp_pos));
  EXPECT_NEAR(0,
              hypot(this->new_top_left[0] - temp_pos[0],
                    this->new_top_left[1] - temp_pos[1]),
              sqrt(2) * this->resolution);

  EXPECT_TRUE(this->test_map->getPosition(this->bottom_left_ind, temp_pos));
  EXPECT_NEAR(0,
              hypot(this->new_bottom_left[0] - temp_pos[0],
                    this->new_bottom_left[1] - temp_pos[1]),
              sqrt(2) * this->resolution);

  EXPECT_TRUE(this->test_map->getPosition(this->top_right_ind, temp_pos));
  EXPECT_NEAR(0,
              hypot(this->new_top_right[0] - temp_pos[0],
                    this->new_top_right[1] - temp_pos[1]),
              sqrt(2) * this->resolution);

  EXPECT_TRUE(this->test_map->getPosition(this->bottom_right_ind, temp_pos));
  EXPECT_NEAR(0,
              hypot(this->new_bottom_right[0] - temp_pos[0],
                    this->new_bottom_right[1] - temp_pos[1]),
              sqrt(2) * this->resolution);

  // check the other way around, i.e. the index
  Index temp_ind;
  EXPECT_TRUE(this->test_map->getIndex(this->new_top_left, temp_ind));
  EXPECT_TRUE(temp_ind == this->top_left_ind);

  EXPECT_TRUE(this->test_map->getIndex(this->new_bottom_left, temp_ind));
  EXPECT_TRUE(temp_ind == this->bottom_left_ind);

  EXPECT_TRUE(this->test_map->getIndex(this->new_top_right, temp_ind));
  EXPECT_TRUE(temp_ind == this->top_right_ind);

  EXPECT_TRUE(this->test_map->getIndex(this->new_bottom_right, temp_ind));
  EXPECT_TRUE(temp_ind == this->bottom_right_ind);
}

// test hybrid a-star after the grid map has moved
class MovePlannerTest : public ::testing::Test {
protected:
  // number of replanning to tests
  const int num_replans = 2;

  // some parameters for the planner
  std::vector<float> steering_actions{-10 * M_PI / 180, 0 * M_PI / 180,
                                      10 * M_PI /
                                          180}; // store the steering actions
  hybrid_a_star::parameters::planner_intrinsics intrinsics{
      1.0, true, 1000000}; // for chord length, etc
  std::string filename; // filename of the .yaml file containing configuration
                        // of scenario
  std::string scenario{"dead end with mini"}; // store the scenario name
  hybrid_a_star::parameters::penalties penalty_params{
      0, 0, 0, 0}; // penalties for the global travel cost
  hybrid_a_star::parameters::stop_explore stop_params{
      5 * M_PI / 180,
      0.1}; // thresholds dictating when the goal has been reached
  hybrid_a_star::parameters::vehicle
      vehicle_params;         // parameter describing shape of the vehicle
  bool allow_reverse = false; // allow the car to go backwards
  float dvwnf = 0;            // default value when not found

  // graph parameter
  hybrid_a_star::parameters::graph graph_param;

  // starting and goal positions
  hybrid_a_star::rob_pos start_pos, goal_pos;

  // starting and goal index
  Index start_ind{50, 50}, goal_ind{50, 88};

  // motion models
  hybrid_a_star::parameters::vehicle_motion vm_vani{
      allow_reverse, steering_actions, graph_param,
      intrinsics.default_chord_l_factor, vehicle_params};
  hybrid_a_star::parameters::vehicle_motion vm_hybrid{
      allow_reverse, steering_actions, graph_param,
      intrinsics.default_chord_l_factor, vehicle_params};

  // create graph to perform search for vanilla a-star and hybrid a-star
  std::vector<std::string> layer_name{"OG"};
  std::vector<float> time{0.};
  hybrid_a_star::Grid_Map_Wrapper gmw_twoD{
      graph_param,  vehicle_params,
      vm_vani,      hybrid_a_star::parameters::two_d_pre_model,
      layer_name,   time,
      layer_name[0]};
  hybrid_a_star::kinematic_models::gen_kinematic_func nonholo_model =
      &hybrid_a_star::kinematic_models::non_holo_car_steer;
  hybrid_a_star::Grid_Map_Wrapper3D gmw_hybrid{
      graph_param, vehicle_params, vm_hybrid,    nonholo_model,
      layer_name,  time,           layer_name[0]};

  // create vanilla a-star planner
  hybrid_a_star::stopping_crit::gen_stop_func<GridMap> twoD_stop =
      &hybrid_a_star::stopping_crit::xyInd<GridMap>;
  std::vector<hybrid_a_star::heuristics::gen_heuristic_func<GridMap>>
      a_star_heuristic{hybrid_a_star::parameters::vani_heuristic<GridMap>};
  hybrid_a_star::global_cost::gen_globalcost_func twoD_cost_func =
      &hybrid_a_star::global_cost::raw_distance;
  hybrid_a_star::path_planning::A_Star<GridMap> vani_planner{
      gmw_twoD,       a_star_heuristic,
      twoD_stop,      twoD_cost_func,
      penalty_params, stop_params,
      false,          intrinsics.max_num_iterations};

  // create dijsktra planner
  std::vector<hybrid_a_star::heuristics::gen_heuristic_func<GridMap>>
      dij_heuristic{hybrid_a_star::parameters::dijsktra_heuristic<GridMap>};
  hybrid_a_star::stopping_crit::gen_stop_func<GridMap> no_stop =
      &hybrid_a_star::stopping_crit::no_stop<GridMap>;
  hybrid_a_star::path_planning::A_Star<GridMap> dij_pre_planner{
      gmw_twoD,       dij_heuristic, no_stop, twoD_cost_func,
      penalty_params, stop_params,   false,   intrinsics.max_num_iterations};
  hybrid_a_star::path_planning::A_Star<GridMap> dij_planner{
      gmw_twoD,       dij_heuristic, twoD_stop, twoD_cost_func,
      penalty_params, stop_params,   false,     intrinsics.max_num_iterations};

  // create hybrid a-star planner
  hybrid_a_star::stopping_crit::gen_stop_func<GridMap> hybrid_stop =
      &hybrid_a_star::stopping_crit::xyInd_headRaw<GridMap>;
  hybrid_a_star::global_cost::gen_globalcost_func hybrid_cost_func =
      &hybrid_a_star::global_cost::penal_steer_reverse_mult;
  std::vector<hybrid_a_star::heuristics::gen_heuristic_func<GridMap>>
      hybrid_heuristics{hybrid_a_star::parameters::vani_heuristic<GridMap>,
                        &hybrid_a_star::heuristics::reedsheeps_length<GridMap>};
  hybrid_a_star::path_planning::A_Star<GridMap> hybrid_planner{
      gmw_hybrid,
      hybrid_heuristics,
      hybrid_stop,
      hybrid_cost_func,
      penalty_params,
      stop_params,
      intrinsics.precompute_flag,
      intrinsics.max_num_iterations};
  std::vector<hybrid_a_star::path_planning::A_Star<GridMap> *> precomp_planners{
      &dij_pre_planner};
  std::vector<
      hybrid_a_star::precompute::gen_default_func_when_not_found<GridMap>>
      funcs_when_not_found{hybrid_a_star::precompute::default_return<GridMap>};

  MovePlannerTest(){};

  void SetUp() override {
    // get the filename
    filename =
        ros::package::getPath("hybrid_a_star") + "/scripts/scenarios.yaml";

    // read graph parameter from the yaml file directly
    int nrows, ncols;
    hybrid_a_star::parameters::graph graph_param =
        hybrid_a_star::parameters::getGraphParamYaml(
            filename, scenario, hybrid_a_star::parameters::default_scenario,
            nrows, ncols);

    // create motion model for vanilla a-star and hybrid a-star
    vm_vani =
        hybrid_a_star::parameters::getPrecomputeVM(graph_param, vehicle_params);
    vm_hybrid.set_values(graph_param, intrinsics.default_chord_l_factor);

    // load costmap
    gmw_twoD.loadCostmap(filename, scenario);
    gmw_hybrid.loadCostmap(filename, scenario);

    // update total cells of graph
    gmw_hybrid.updateTotalCells();
    gmw_twoD.updateTotalCells();

    // update the graph
    gmw_hybrid.updateParams();

    // populate the start and end goal
    bool success;
    unsigned int time_index = 0;
    float time_frac = 0.;
    start_pos = gmw_twoD.getPosition(start_ind, time_index, time_frac, success);
    start_pos.theta = 90 * M_PI / 180;
    goal_pos = gmw_twoD.getPosition(goal_ind, time_index, time_frac, success);
    goal_pos.theta = 180 * M_PI / 180;

    // check if user wanted precomputation
    if (intrinsics.precompute_flag) {
      hybrid_planner.setPrecompPlanners(&precomp_planners,
                                        &funcs_when_not_found, dvwnf);
    }
  }
};

// function to plan and print out statistics
bool planAndPrint(hybrid_a_star::path_planning::A_Star<GridMap> &planner,
                  hybrid_a_star::Grid_Map_Wrapper &gmw,
                  const hybrid_a_star::rob_pos &start_pos,
                  const hybrid_a_star::rob_pos &goal_pos,
                  const std::string method_name, const std::string &layer_name,
                  const bool &show_statistics = true,
                  const bool &show_graph = true) {
  planner.setStartGoal(start_pos, goal_pos);
  planner.search();
  planner.produce_path(true);
  printf("Path from %s has %sbeen found with %u iterations and %d expansions. "
         "Time taken to find path is %d milliseconds. Error code is %u. Cost "
         "is: %f \n",
         method_name.c_str(), planner.isPathFound() ? "" : "not ",
         planner.getNumIter(), planner.getNumExpand(), planner.getSearchTime(),
         planner.getErrorCode(), planner.getCost());

  // visualise path via populating the grid with different number and printing
  // it out at the end
  auto &current_map = gmw.getCostmap();
  current_map.add("temp", current_map.get(layer_name));
  if (show_graph && planner.isPathFound()) {
    std::vector<hybrid_a_star::rob_pos> path = planner.getPath();
    std::cout << std::endl;
    auto it = path.rbegin();
    int i;
    for (i = 1; it != path.rend(); ++it, i++) {
      // printf("Position is (%f, %f, %f)\n", it->x, it->y, it->theta);
      float value = -1 * i;
      gmw.setValue(*it, value);
    }
    // just before printing, show move stuff around so we can see clearly
    current_map.convertToDefaultStartIndex();
    gmw.print();

    std::cout << std::endl << std::endl;
  } else if (show_graph) {
    // just mark the starting and ending locations
    float value = 20;
    gmw.setValue(start_pos, value);
    gmw.setValue(goal_pos, value);
    // just before printing, show move stuff around so we can see clearly
    current_map.convertToDefaultStartIndex();
    gmw.print();
  }
  current_map[layer_name] = current_map["temp"];
  current_map.erase("temp");

  return planner.isPathFound();
}

void move_before_replan(
    const std::vector<hybrid_a_star::Grid_Map_Wrapper *> &gmws,
    hybrid_a_star::rob_pos &start_pos, hybrid_a_star::rob_pos &goal_pos,
    const Index &start_ind, const Index &goal_ind,
    const std::string &layer_name) {

  // define new starting and goal position
  Index new_start_ind{32, start_ind[1]};
  Index new_goal_ind{70, 30};
  auto &current_map = gmws[0]->getCostmap();

  Position temp;
  current_map.getPosition(new_start_ind, temp);
  start_pos.x = temp[0];
  start_pos.y = temp[1];
  start_pos.theta = -90 * M_PI / 180;

  current_map.getPosition(new_goal_ind, temp);
  goal_pos.x = temp[0];
  goal_pos.y = temp[1];
  goal_pos.theta = 90 * M_PI / 180;

  // iterate over all costmaps and make modification
  for (auto it : gmws) {
    // get the grid map back
    auto &current_map = it->getCostmap();
    // replace all NANs with 1000
    for (GridMapIterator it(current_map); !it.isPastEnd(); ++it) {
      float &val = current_map.at(layer_name, *it);
      if (!std::isfinite(val))
        val = 1000.0;
    }

    // get the index of the origin
    const Position &origin = current_map.getPosition();
    Index old_origin_ind;
    current_map.getIndex(origin, old_origin_ind);

    // move origin index by a little
    int x_increment = round(0.5 * (0.5 - 0.3) * current_map.getSize()(0));
    int y_increment = round(0.25 * current_map.getSize()(1));
    Index new_origin_ind{old_origin_ind[0] + x_increment,
                         old_origin_ind[1] + y_increment};
    // move the map now
    Position new_origin;
    current_map.getPosition(new_origin_ind, new_origin);
    bool success = current_map.move(new_origin);

    // replace all NANs with zero and 1000 with inf
    for (GridMapIterator it(current_map); !it.isPastEnd(); ++it) {
      float &val = current_map.at(layer_name, *it);
      if (!std::isfinite(val)) {
        val = 0.0;
      } else if (val == 1000) {
        val = INFINITY;
      }
    }
  }
}

TEST_F(MovePlannerTest, ValidPathsBeforeMove) {
  // check paths before moving
  bool path_found = planAndPrint(
      this->vani_planner, this->gmw_twoD, this->start_pos, this->goal_pos,
      "vanilla A-star", this->layer_name[0], true, false);
  EXPECT_TRUE(path_found) << "Path for vanilla A star not found";
  path_found = planAndPrint(this->dij_planner, this->gmw_twoD, this->start_pos,
                            this->goal_pos, "Dijsktra", this->layer_name[0],
                            true, false);
  EXPECT_TRUE(path_found) << "Path for dijsktra A star not found";
  path_found = planAndPrint(this->hybrid_planner, this->gmw_hybrid,
                            this->start_pos, this->goal_pos, "hybrid A-star",
                            this->layer_name[0], true, true);
  EXPECT_TRUE(path_found) << "Path for hybrid A star not found";
}

TEST_F(MovePlannerTest, ValidPathsBeforeMoveReplans) {
  for (int i = 1; i <= this->num_replans; i++) {
    // check paths before moving
    bool path_found = planAndPrint(
        this->vani_planner, this->gmw_twoD, this->start_pos, this->goal_pos,
        "vanilla A-star", this->layer_name[0], true, false);
    EXPECT_TRUE(path_found) << "Path for vanilla A star not found during the "
                            << i << "th replanning stage";

    path_found = planAndPrint(this->dij_planner, this->gmw_twoD,
                              this->start_pos, this->goal_pos, "Dijsktra",
                              this->layer_name[0], true, false);
    EXPECT_TRUE(path_found) << "Path for dijsktra A star not found during the "
                            << i << "th replanning stage";

    path_found = planAndPrint(this->hybrid_planner, this->gmw_hybrid,
                              this->start_pos, this->goal_pos, "hybrid A-star",
                              this->layer_name[0], true, true);
    EXPECT_TRUE(path_found) << "Path for hybrid A star not found during the "
                            << i << "th replanning stage";
  }
}

TEST_F(MovePlannerTest, ValidPathsAfterMove) {

  // move and see again if the path can be found
  std::vector<hybrid_a_star::Grid_Map_Wrapper *> gmws{&this->gmw_twoD,
                                                      &this->gmw_hybrid};
  move_before_replan(gmws, this->start_pos, this->goal_pos, this->start_ind,
                     this->goal_ind, this->layer_name[0]);

  bool path_found = planAndPrint(
      this->vani_planner, this->gmw_twoD, this->start_pos, this->goal_pos,
      "vanilla A-star", this->layer_name[0], true, false);
  EXPECT_TRUE(path_found) << "Path for vanilla A star after moving not found";

  path_found = planAndPrint(this->dij_planner, this->gmw_twoD, this->start_pos,
                            this->goal_pos, "Dijsktra", this->layer_name[0],
                            true, false);
  EXPECT_TRUE(path_found) << "Path for dijsktra A star after moving not found";

  path_found = planAndPrint(this->hybrid_planner, this->gmw_hybrid,
                            this->start_pos, this->goal_pos, "hybrid A-star",
                            this->layer_name[0], true, true);
  EXPECT_TRUE(path_found) << "Path for hybrid A star after moving not found";
}

TEST_F(MovePlannerTest, ValidPathsAfterMoveReplans) {

  for (int i = 1; i <= this->num_replans; i++) {
    // reload costmap
    this->gmw_twoD.loadCostmap(this->filename, this->scenario);
    this->gmw_hybrid.loadCostmap(this->filename, this->scenario);

    // move and see again if the path can be found
    std::vector<hybrid_a_star::Grid_Map_Wrapper *> gmws{&this->gmw_twoD,
                                                        &this->gmw_hybrid};
    move_before_replan(gmws, this->start_pos, this->goal_pos, this->start_ind,
                       this->goal_ind, this->layer_name[0]);

    bool path_found = planAndPrint(
        this->vani_planner, this->gmw_twoD, this->start_pos, this->goal_pos,
        "vanilla A-star", this->layer_name[0], true, false);
    EXPECT_TRUE(path_found)
        << "Path for vanilla A star after moving not found during the " << i
        << "th replanning stage";

    path_found = planAndPrint(this->dij_planner, this->gmw_twoD,
                              this->start_pos, this->goal_pos, "Dijsktra",
                              this->layer_name[0], true, false);
    EXPECT_TRUE(path_found)
        << "Path for dijsktra A star after moving not found during the " << i
        << "th replanning stage";

    path_found = planAndPrint(this->hybrid_planner, this->gmw_hybrid,
                              this->start_pos, this->goal_pos, "hybrid A-star",
                              this->layer_name[0], true, true);
    EXPECT_TRUE(path_found)
        << "Path for hybrid A star after moving not found during the " << i
        << "th replanning stage";
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
