#ifndef A_STAR
#define A_STAR

/*
 * \file a_star.h
 * \brief Based on the graph structure and the parameters, compute the path
 */

#include "base_graph.h"
#include "error_codes.h"
#include <Eigen/Core>
#include <boost/heap/binomial_heap.hpp>
#include <chrono>
#include <map>
#include <set>
#include <vector>

namespace hybrid_a_star {

namespace path_planning {

// define structure to store useful information during search
struct Node {
  // predecessor node
  const Node *pred = NULL;

  // robot's position
  rob_pos pos;

  // the index of this position
  int pos_index;

  // the expansion number
  int num_expansion;

  // has this node been opened and closed
  bool opened = false;
  bool closed = false;

  // store the cost so far
  float cost;

  // store the priority for the priority queue to make comparison on
  float priority;
};

// define function to compare two nodes
struct CompareNodes {
  inline bool operator()(const Node *lhs, const Node *rhs) const;
};

// OPEN LIST AS BOOST IMPLEMENTATION
typedef boost::heap::binomial_heap<Node *, boost::heap::compare<CompareNodes>>
    priorityQueue;

template <typename cost_map_type> class A_Star {
private:
  // variables to help with middle-of planning precompute flag change
  bool planning;
  bool interim_precompute_flag;

  // pointer to base graph
  const Base_Graph<cost_map_type> *graph;

  // pointer to vector of heuristic functions
  const std::vector<heuristics::gen_heuristic_func<cost_map_type>>
      *heuristic_funcs;

  // pointer to stopping criteria
  const stopping_crit::gen_stop_func<cost_map_type> stopping_func;

  // pointer to function for obtaining the global cost
  const global_cost::gen_globalcost_func global_cost_func;

  // pointer to penalty parameters
  const parameters::penalties *penalties_params;

  // parameters for the stopping criteria
  const parameters::stop_explore *stop_params;

  // perform precomputation?
  bool precompute_flag;

  // preform oneshot dubins/reedsheeps?
  bool oneshot_flag;

  // store indices of oneshot nodes
  std::set<int> oneshot_visited_nodes;

  // pointers to nodes
  std::unique_ptr<Node[]> nodes;

  // start and goal  position
  rob_pos start_pos;
  rob_pos goal_pos;

  // vector storing the plan
  std::vector<rob_pos> path;

  // vector storing oneshot plan
  std::vector<rob_pos> oneshot_path;

  // flag to state if path was found
  bool path_found;

  // flag to indicate that path was found via oneshot
  bool oneshot_path_found;

  // number of oneshot points in the path
  int num_oneshot_points;

  // store error code based on the problem encountered
  uint32_t error_code;

  // index of the last position expanded (for backward propagation to produce
  // path)
  int index_las_pos;

  // current cost of the path
  float current_cost;

  // current priority of the path
  float current_priority;

  // cost of the oneshot path
  float oneshot_cost;

  // some statistics of the planner
  int num_iterations; // number of iterations before path is found or limit
                      // reached
  int num_expansions; // number of times new nodes are put in the open list;

  // maximum number of iterations (modifiable via the constants in parameters
  // namespace)
  unsigned int max_num_iterations;

  // store the time taken in milliseconds during the last search
  std::chrono::milliseconds search_time_elapsed;

  // pointer to a vector of planner objects which will be used for
  // precomputation
  std::vector<A_Star<cost_map_type> *> *precomp_planners;

  // pointer to a vector of default functions to call when queried grid has not
  // been evaluated
  const std::vector<precompute::gen_default_func_when_not_found<cost_map_type>>
      *funcs_when_not_found;

  // store default value when not found to be used with the function above
  float dvwnf;

  // fixed decaying one shot interval
  int max_oneshot_interval;

  // keep track of the number of times the node has been expanded
  std::map<int, int> expanded_counter;

  // long term planning (don't recalculate max theoretical heuristic to allow
  // better oneshot)
  bool recal_theoretical_max, store_counter;
  double theoretical_max_dist;
  std::map<int, double> oneshot_counter_map;

public:
  // constructor
  A_Star(const Base_Graph<cost_map_type> &graph,
         const std::vector<heuristics::gen_heuristic_func<cost_map_type>>
             &heuristic_funcs,
         const stopping_crit::gen_stop_func<cost_map_type> stopping_func,
         const global_cost::gen_globalcost_func global_cost_func,
         const parameters::penalties &penalties_params,
         const parameters::stop_explore &stop_params,
         const bool &precompute_flag, const unsigned int &max_num_iterations);

  // alternate constructor with oneshot flag
  A_Star(const Base_Graph<cost_map_type> &graph,
         const std::vector<heuristics::gen_heuristic_func<cost_map_type>>
             &heuristic_funcs,
         const stopping_crit::gen_stop_func<cost_map_type> stopping_func,
         const global_cost::gen_globalcost_func global_cost_func,
         const parameters::penalties &penalties_params,
         const parameters::stop_explore &stop_params,
         const bool &precompute_flag, const bool &oneshot_flag,
         const unsigned int &max_num_iterations);

  // allow resetting precompute flag
  void setPrecomputeFlag(const bool &precompute_flag);
  // allow resetting oneshot flag
  void setOneshotFlag(const bool &oneshot_flag);
  // allowing setting maximum oneshot interval
  void setOneshotInterval(const int &max_oneshot_interval);
  // allow resetting maximum number of iterations for search
  void setMaxNumIter(const unsigned int &max_num_iterations);
  // allow resetting flag for recalculating max distance
  void setRecalTMax(bool recal_theoretical_max);
  // allow resetting flag for storing the counter
  void setStoreCounter(bool store_counter);
  // allow changing reference to stop parameters
  void setStopParams(const parameters::stop_explore &stop_params);

  // set reference to vector of other instances of planners for precomputation
  void setPrecompPlanners(
      std::vector<A_Star<cost_map_type> *> *precomp_planners,
      const std::vector<
          precompute::gen_default_func_when_not_found<cost_map_type>>
          *funcs_when_not_found,
      const float &dvwnf);

  // function to precompute from the set of planners
  void precomputeNow();

  // set start and goal pos
  void setStartGoal(const rob_pos &start_pos, const rob_pos &goal_pos);

  // search until goal is reached
  void search();

  // attempt to oneshot it
  bool oneshot_attempt(const rob_pos &current_pos, const rob_pos &goal_pos,
                       std::vector<rob_pos> &temp_oneshot_path);

  // produce the path
  void produce_path(bool start_to_goal = false);

  // function to get maximum heuristics from the list of heuristic functions
  void getHeuristic(const rob_pos &goal_pos, const rob_pos &current,
                    float &heuristic);

  // functions to check if node was expanded
  bool checkExpanded(const Eigen::Vector2d &c_pt);
  bool checkExpanded(const rob_pos &c_pos);

  // function to assimilate heuristics
  void assimilateHeuristics(float &heuristic, const rob_pos &neighbor,
                            bool &success);

  // return the path
  std::vector<rob_pos> &getPath();

  // return oneshot path
  const std::vector<rob_pos> &getOneshotPath();

  // return the total cost of traversing to goal
  float &getCost();

  // return the "estimated" cost of traversing to goal
  float &getPriority();

  // return the the number of points in oneshot path
  int getNumOneshotPts();

  // return mapping of expanded nodes (FOR DEBUGGING)
  const std::map<int, int> &getExpandedCounter();

  // check if path was found
  bool &isPathFound();

  // check if path was found via oneshot
  bool isOneshotPathFound();

  // return the error code
  uint32_t getErrorCode();

  // destructor to deallocate the node pointers
  ~A_Star() = default;

  // function to return by reference the starting and goal poses
  const rob_pos &getStartPos() const;
  const rob_pos &getGoalPos() const;

  // return the statistics of the planner
  const int &getNumIter();
  const int &getNumExpand();
  const int getSearchTime();
};

// useful function to determine if nodes were re-expanded during search
inline std::map<int, int>
nodesReexpanded(const std::map<int, int> &expanded_counter, bool &reexpanded) {
  std::map<int, int> reexpanded_counter;
  for (auto it : expanded_counter) {
    if (it.second > 1) {
      reexpanded_counter.insert({it.first, it.second});
    }
  }

  if (reexpanded_counter.size() > 0) {
    reexpanded = true;
  } else {
    reexpanded = false;
  }
  return reexpanded_counter;
}

} // namespace path_planning
} // namespace hybrid_a_star

#include "a_star.tpp"

#endif
