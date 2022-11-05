#include <unordered_map>

#include "hybrid_a_star/a_star.h"

using namespace hybrid_a_star;

// define the function to compare two nodes
bool path_planning::CompareNodes::operator()(const Node *lhs,
                                             const Node *rhs) const {
  if (lhs->priority == rhs->priority) {
    if (lhs->cost == rhs->cost) {
      return lhs->num_expansion > rhs->num_expansion;
    }
    return lhs->cost < rhs->cost;
  }

  return lhs->priority > rhs->priority;
}

template <typename cost_map_type>
path_planning::A_Star<cost_map_type>::A_Star(
    const Base_Graph<cost_map_type> &graph,
    const std::vector<heuristics::gen_heuristic_func<cost_map_type>>
        &heuristic_funcs,
    const stopping_crit::gen_stop_func<cost_map_type> stopping_func,
    const global_cost::gen_globalcost_func global_cost_func,
    const parameters::penalties &penalties_params,
    const parameters::stop_explore &stop_params, const bool &precompute_flag,
    const unsigned int &max_num_iterations)
    : stopping_func(stopping_func), global_cost_func(global_cost_func) {
  this->graph = &graph;
  this->heuristic_funcs = &heuristic_funcs;
  // this->stopping_func = stopping_func;
  // this->global_cost_func = global_cost_func;
  this->penalties_params = &penalties_params;
  this->stop_params = &stop_params;
  this->precompute_flag = precompute_flag;
  this->max_num_iterations = max_num_iterations;
  this->dvwnf = 0;
  this->planning = false;
  this->interim_precompute_flag = precompute_flag;

  path_found = false;
  oneshot_path_found = false;

  num_iterations = 0;
  num_expansions = 0;
  num_oneshot_points = 0;

  recal_theoretical_max = true;
  store_counter = false;
  theoretical_max_dist = 0.0;

  oneshot_flag = false;
  max_oneshot_interval = 200;
  oneshot_cost = 0;
  // nodes = new Node[graph.getTotalCells()];

  // check logically if the discretisation in heading is lower than heading
  // threshold. if not, warn the user assert(graph.graph_param->resolution[2] <=
  // stop_params.heading_threshold_error);
}

template <typename cost_map_type>
path_planning::A_Star<cost_map_type>::A_Star(
    const Base_Graph<cost_map_type> &graph,
    const std::vector<heuristics::gen_heuristic_func<cost_map_type>>
        &heuristic_funcs,
    const stopping_crit::gen_stop_func<cost_map_type> stopping_func,
    const global_cost::gen_globalcost_func global_cost_func,
    const parameters::penalties &penalties_params,
    const parameters::stop_explore &stop_params, const bool &precompute_flag,
    const bool &oneshot_flag, const unsigned int &max_num_iterations)
    : stopping_func(stopping_func), global_cost_func(global_cost_func) {
  this->graph = &graph;
  this->heuristic_funcs = &heuristic_funcs;
  // this->stopping_func = stopping_func;
  // this->global_cost_func = global_cost_func;
  this->penalties_params = &penalties_params;
  this->stop_params = &stop_params;
  this->precompute_flag = precompute_flag;
  this->oneshot_flag = oneshot_flag;
  this->max_num_iterations = max_num_iterations;
  this->dvwnf = 0;
  this->planning = false;
  this->interim_precompute_flag = precompute_flag;

  path_found = false;
  oneshot_path_found = false;
  max_oneshot_interval = 200;
  oneshot_cost = 0;

  num_iterations = 0;
  num_expansions = 0;
  num_oneshot_points = 0;

  recal_theoretical_max = true;
  store_counter = false;
  theoretical_max_dist = 0.0;

  // nodes = new Node[graph.getTotalCells()];

  // check logically if the discretisation in heading is lower than heading
  // threshold. if not, warn the user assert(graph.graph_param->resolution[2]
  // <= stop_params.heading_threshold_error);
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setPrecomputeFlag(
    const bool &precompute_flag) {
  if (planning) {
    this->interim_precompute_flag = precompute_flag;
  } else {
    this->precompute_flag = precompute_flag;
    this->interim_precompute_flag = precompute_flag;
  }
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setOneshotFlag(
    const bool &oneshot_flag) {
  this->oneshot_flag = oneshot_flag;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setOneshotInterval(
    const int &max_oneshot_interval) {
  this->max_oneshot_interval = max_oneshot_interval;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setMaxNumIter(
    const unsigned int &max_num_iterations) {
  this->max_num_iterations = max_num_iterations;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setRecalTMax(
    bool recal_theoretical_max) {
  this->recal_theoretical_max = recal_theoretical_max;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setStoreCounter(bool store_counter) {
  this->store_counter = store_counter;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setStopParams(
    const parameters::stop_explore &stop_params) {
  this->stop_params = &stop_params;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setPrecompPlanners(
    std::vector<A_Star<cost_map_type> *> *precomp_planners,
    const std::vector<
        precompute::gen_default_func_when_not_found<cost_map_type>>
        *funcs_when_not_found,
    const float &dvwnf) {
  if (precompute_flag || interim_precompute_flag) {
    if (planning) {
      precompute_flag = false;
      interim_precompute_flag = true;
    }
    this->precomp_planners = precomp_planners;
    this->funcs_when_not_found = funcs_when_not_found;
    // make sure the vector is of the same size
    assert(precomp_planners->size() == funcs_when_not_found->size());
    this->dvwnf = dvwnf;
  }
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::setStartGoal(
    const rob_pos &start_pos, const rob_pos &goal_pos) {
  this->start_pos = start_pos;
  this->goal_pos = goal_pos;
}

template <typename cost_map_type>
const rob_pos &path_planning::A_Star<cost_map_type>::getStartPos() const {
  return start_pos;
}

template <typename cost_map_type>
const rob_pos &path_planning::A_Star<cost_map_type>::getGoalPos() const {
  return goal_pos;
}

template <typename cost_map_type>
std::vector<rob_pos> &path_planning::A_Star<cost_map_type>::getPath() {
  return path;
}

template <typename cost_map_type>
const std::vector<rob_pos> &
path_planning::A_Star<cost_map_type>::getOneshotPath() {
  if (!oneshot_path_found) {
    oneshot_path.clear();
  }

  return oneshot_path;
}

template <typename cost_map_type>
float &path_planning::A_Star<cost_map_type>::getCost() {
  return current_cost;
}

template <typename cost_map_type>
float &path_planning::A_Star<cost_map_type>::getPriority() {
  return current_priority;
}

template <typename cost_map_type>
int path_planning::A_Star<cost_map_type>::getNumOneshotPts() {
  return num_oneshot_points;
}

template <typename cost_map_type>
const std::map<int, int> &
path_planning::A_Star<cost_map_type>::getExpandedCounter() {
  return expanded_counter;
}

template <typename cost_map_type>
bool &path_planning::A_Star<cost_map_type>::isPathFound() {
  return path_found;
}

template <typename cost_map_type>
bool path_planning::A_Star<cost_map_type>::isOneshotPathFound() {
  return oneshot_path_found;
}

template <typename cost_map_type>
uint32_t path_planning::A_Star<cost_map_type>::getErrorCode() {
  return error_code;
}

template <typename cost_map_type>
const int &path_planning::A_Star<cost_map_type>::getNumIter() {
  return num_iterations;
}

template <typename cost_map_type>
const int &path_planning::A_Star<cost_map_type>::getNumExpand() {
  return num_expansions;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::precomputeNow() {
  if (precompute_flag) {
    for (auto it = (*precomp_planners).begin(); it != (*precomp_planners).end();
         ++it) {
      // reverse set the start and goal nodes first
      (*it)->setStartGoal(goal_pos, start_pos);
      // search
      (*it)->search();
      // add the time used for precomputation
      search_time_elapsed += (*it)->search_time_elapsed;

      // printf("Total number of expanded vs free space %f %%\n",
      // 100.0*((*it)->getNumExpand())/(*it)->graph->getTotalCells());
    }
  }
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::search() {
  planning = true;
  // start the time with zero
  search_time_elapsed = std::chrono::milliseconds::zero();

  // start the timer
  auto start = std::chrono::steady_clock::now();

  // reset path found before we begin anew
  path_found = false;
  oneshot_path_found = false;

  // reset the oneshot cost
  oneshot_cost = 0;

  // reset the nodes
  nodes.reset(new Node[graph->getTotalCells()]);

  // reset the expanded counters
  expanded_counter.clear();

  // precompute stuff if need be
  precomputeNow();

  // double the number of steering if reverse is enabled
  int num_steerings = (graph->vm->allow_reverse) ? (graph->vm->num_actions) * 2
                                                 : (graph->vm->num_actions);

  // indices of predecessor and successor nodes
  int iPred, iSucc, iSucc_goal;

  // reset stats of number of iterations
  num_iterations = 0;
  num_expansions = 0;

  // keep track of when to do oneshot
  float oneshot_counter = max_oneshot_interval, oneshot_interval;

  // new cost of the successor
  float new_cost, heuristic, priority; // priority includes the heuristic cost

  // reset index of last position
  index_las_pos = -1;

  // create a start node
  bool success;
  iPred = graph->getIndexHigher(start_pos, success);
  // if start node was not within range get the error code
  if (!success || !graph->passable(start_pos)) {
    error_code = GetPathResult::INVALID_START;
    if (interim_precompute_flag != precompute_flag) {
      precompute_flag = interim_precompute_flag;
    }
    planning = false;
    return;
  }
  // similarly check if goal pos is valid
  iSucc_goal = graph->getIndexHigher(goal_pos, success);
  if (!success || !graph->passable(goal_pos)) {
    error_code = GetPathResult::INVALID_GOAL;
    return;
  }
  getHeuristic(goal_pos, start_pos, heuristic);
  assimilateHeuristics(heuristic, start_pos, success);
  Node start_node =
      Node{NULL, start_pos, iPred, 0, true, false, 0.0, heuristic};
  // create unordered map to store handle to items pushed to priority queue
  // std::unordered_map<int, priorityQueue::handle_type> handles_map;
  // create priority queue and push the node in
  priorityQueue O;
  nodes[iPred] = start_node;
  O.push(&nodes[iPred]);
  // handles_map.insert({iPred, O.push(&nodes[iPred])});

  // store initial heuristic as theoretical admissible max
  if (recal_theoretical_max) {
    theoretical_max_dist = heuristic;
    oneshot_counter_map[iPred] = 0;
  }

  // place holder for the current node and its sucessor
  Node *pred;
  Node *succ;

  // comparator for nodes
  CompareNodes nodes_comparator;

  // search till the open list is empty
  while (!O.empty()) {
    // pop the node with lowest cost from the cell
    pred = O.top();
    // iPred = graph->getIndexHigher(pred->pos, success);
    iPred = pred->pos_index;

    if (expanded_counter.count(iPred)) {
      expanded_counter[iPred] += 1;
    } else {
      expanded_counter.insert({iPred, 0});
    }

    // increment the iteration number;
    num_iterations++;

    // if this node has been closed, remove it and move on to some other node
    if (nodes[iPred].closed) {
      O.pop();
      continue;
    }
    // otherwise, expand the node if it is opened
    else { // if (nodes[iPred].opened) {
      // add it to the closed list
      nodes[iPred].closed = true;
      // remove the node from open list
      O.pop();

      // check if goal is reached via stopping criteria
      if (stopping_func(pred->pos, goal_pos, *graph, *stop_params)) {
        if (pred->pred) {
          if (index_las_pos == pred->pred->pos_index &&
              pred->cost == pred->priority) {
            current_priority = pred->priority;
            current_cost = pred->cost;
            oneshot_path_found = true;
          } else {
            oneshot_path_found = false;
            oneshot_cost = 0;
            index_las_pos = pred->pos_index;
          }
        } else {
          oneshot_path_found = false;
          oneshot_cost = 0;
          index_las_pos = pred->pos_index;
        }

        path_found = true;
        error_code = GetPathResult::SUCCESS;
        break;

      } else if (num_iterations > max_num_iterations) {
        error_code = GetPathResult::PAT_EXCEEDED;
        break;

        // otherwise continue with search
      } else {
        // change the oneshot interval based on how far it is from goal
        oneshot_interval = (pred->priority - pred->cost) *
                           max_oneshot_interval / theoretical_max_dist;
        // Below should be where we test if dubin's or reedsheep's path alone
        if (store_counter && oneshot_counter_map.count(iPred)) {
          oneshot_counter = oneshot_counter_map[iPred];
        } else if (store_counter) {
          oneshot_counter_map[iPred] = oneshot_counter;
        }

        // allows reaching the goal directly (at low frequency)
        if (oneshot_flag && oneshot_counter >= oneshot_interval) {

          // reset the counter
          oneshot_counter = 0;

          std::vector<rob_pos> temp_oneshot_path;
          if (oneshot_attempt(pred->pos, goal_pos, temp_oneshot_path)) {
            new_cost = pred->cost + oneshot_cost;

            if (!nodes[iSucc_goal].opened ||
                new_cost < nodes[iSucc_goal].cost) {
              // copy the path to oneshot
              oneshot_path = temp_oneshot_path;

              // store indices
              index_las_pos = pred->pos_index;

              succ = new Node{pred, goal_pos, iSucc_goal, num_expansions,
                              true, false,    new_cost,   new_cost};
              // if already present on openlist, update the successor
              nodes[iSucc_goal] = *succ;
              O.push(&nodes[iSucc_goal]);
              /*
              if (nodes[iSucc_goal].opened) {
                nodes[iSucc_goal] = *succ;
                O.decrease(handles_map[iSucc_goal], &nodes[iSucc_goal]);
              } else {
                // put successor on openlist
                nodes[iSucc_goal] = *succ;
                handles_map.insert({iSucc_goal, O.push(&nodes[iSucc_goal])});
              }
              */

              delete succ;
            }
          }
        }

        /*
        printf("Came from (%f, %f, %f) at time (%f), velocity (%f), steering "
               "angle (%f), timing index %d and timing frac %f\n",
               pred->pos.x, pred->pos.y, pred->pos.theta, pred->pos.time,
               pred->pos.velocity, pred->pos.steering_angle,
               pred->pos.time_index, pred->pos.time_frac);
        */
        //  include nodes via forward simulation of kinematic models
        for (int i = 0; i < num_steerings; i++) {
          rob_pos neighbor = graph->getNeighbor(pred->pos, i, success);
          // check if the position is traversable and on grid
          if (success) {
            // get the index of the position
            iSucc = graph->getIndexHigher(neighbor, success);
            // TODO cost-to-go should based on all cells encountered along the
            // way
            // check if successor (neighbor) is not on closed list
            if (!nodes[iSucc].closed) {
              // calculate new cost: sum of  cell value and cost so far
              new_cost = pred->cost +
                         global_cost_func(pred->pos, neighbor, *(graph->vm),
                                          *penalties_params) +
                         graph->getValue(neighbor);

              // if successor is not on open list or that we found a shorter
              // path
              if (!nodes[iSucc].opened || new_cost < nodes[iSucc].cost) {
                // printf("Expanding position: (%f, %f, %f) with cost %f\n",
                // neighbor.x, neighbor.y, neighbor.theta, new_cost);
                num_expansions++;
                oneshot_counter++;

                // calculate the priority which inlcudes the heuristic cost
                getHeuristic(goal_pos, neighbor, heuristic);
                // heuristic = heuristic_func(goal_pos, neighbor, *graph);
                assimilateHeuristics(heuristic, neighbor, success);
                priority = new_cost + heuristic; // create sucessor node
                succ = new Node{pred, neighbor, iSucc,    num_expansions,
                                true, false,    new_cost, priority};

                // if already present on openlist, update the successor
                nodes[iSucc] = *succ;
                O.push(&nodes[iSucc]);
                /*
                if (nodes[iSucc].opened &&
                    nodes_comparator(&nodes[iSucc], succ)) {
                  nodes[iSucc] = *succ;
                  O.decrease(handles_map[iSucc], &nodes[iSucc]);
                } else if (!nodes[iSucc].opened) {
                  // put successor on openlist
                  nodes[iSucc] = *succ;
                  handles_map.insert({iSucc, O.push(&nodes[iSucc])});
                }
                */

                delete succ;
              }
            }
          }
        }
      }
    }
  }

  // store some statistics
  // num_iterations = iterations;
  // num_expansions = expansions;

  // calculating elapsed time in milliseconds
  auto stop = std::chrono::steady_clock::now();
  search_time_elapsed +=
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  // if path is not found and number of iterations still ok, then path is
  // blocked
  if (!path_found && num_iterations < max_num_iterations) {
    error_code = GetPathResult::NO_PATH_FOUND;
  }

  if (interim_precompute_flag != precompute_flag) {
    precompute_flag = interim_precompute_flag;
  }
  planning = false;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::getHeuristic(const rob_pos &goal_pos,
                                                        const rob_pos &current,
                                                        float &heuristic) {
  // iterate through the vector of functions
  heuristic = 0;
  float temp_cost;
  for (auto it = (*heuristic_funcs).begin(); it != (*heuristic_funcs).end();
       ++it) {
    temp_cost = (*it)(current, goal_pos, *graph);
    if (temp_cost > heuristic)
      heuristic = temp_cost;
  }
}

template <typename cost_map_type>
bool path_planning::A_Star<cost_map_type>::checkExpanded(
    const Eigen::Vector2d &c_pt) {
  // create rob pos first
  rob_pos c_pos;
  c_pos.x = c_pt[0];
  c_pos.y = c_pt[1];

  return checkExpanded(c_pos);
}

template <typename cost_map_type>
bool path_planning::A_Star<cost_map_type>::checkExpanded(const rob_pos &c_pos) {
  // get node index
  bool c_success, s_success;
  int cpt_index = graph->getIndexHigher(c_pos, c_success);
  int start_index = graph->getIndexHigher(start_pos, s_success);
  // check if current node is the same as start node
  if (c_success && s_success && cpt_index == start_index) {
    return true;
  }
  // check if node has been expanded
  if (c_success) {
    return nodes[cpt_index].pred != NULL;
  }

  return c_success;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::assimilateHeuristics(
    float &heuristic, const rob_pos &neighbor, bool &success) {
  // loop through the planners and the function when index was not previously
  // evaluated
  float temp_value;
  int neighbor_index, goal_index;
  rob_pos neighbor_pos, goal_pos_n;
  if (precompute_flag) {
    for (std::size_t i = 0; i < precomp_planners->size(); ++i) {
      // get neighbor index from the planner's graph
      neighbor_index =
          (*precomp_planners)[i]->graph->getIndexHigher(neighbor, success);
      neighbor_pos = (*precomp_planners)[i]->graph->getPosition(
          neighbor_index, neighbor.time_index, neighbor.time_frac, success);

      // check if the node has been evaluated
      if (success) {
        if ((*precomp_planners)[i]->nodes[neighbor_index].pred != NULL) {
          // get goal index from the planner's graph
          goal_index =
              (*precomp_planners)[i]->graph->getIndexHigher(goal_pos, success);
          goal_pos_n = (*precomp_planners)[i]->graph->getPosition(
              goal_index, goal_pos.time_index, goal_pos.time_frac, success);

          // triangle inequality to compensate for the fact that position is
          // continuous (not centre of the cell)
          temp_value =
              std::fabs((*precomp_planners)[i]->nodes[neighbor_index].cost -
                        heuristics::euclidean_distance(neighbor, neighbor_pos) -
                        heuristics::euclidean_distance(goal_pos, goal_pos_n));
        } else {
          temp_value =
              (*funcs_when_not_found)[i](goal_pos, neighbor, *graph, dvwnf);
        }

        // keep finding maximum of the cost
        // std::cout<<temp_value<<std::endl;
        if (temp_value > heuristic)
          heuristic = temp_value;
      }
    }
  }
}

template <typename cost_map_type>
bool path_planning::A_Star<cost_map_type>::oneshot_attempt(
    const rob_pos &current_pos, const rob_pos &goal_pos,
    std::vector<rob_pos> &temp_oneshot_path) {

  double t, dist;
  rob_pos pos1, pos2;
  bool first_time = false;

  float total_cost = 0.;

  // double increment = graph->vm->chord_l_min;

  double increment =
      std::min(0.1f, std::min(graph->graph_param->resolution[0],
                              graph->graph_param->resolution[1]));

  // time-related parameters
  float current_time = current_pos.time, time_frac = current_pos.time_frac;
  unsigned int time_index = current_pos.time_index;

  // create oneshot path type depending on allow_reverse
  if (graph->vm->allow_reverse) {
    // create reedsheeps path
    ompl::base::ReedsSheppStateSpace reedsSheppPath(graph->getMinTurnR());

    // set the start and goal states
    heuristics::State *rsStart =
        reedsSheppPath.allocState()->as<heuristics::State>();
    heuristics::State *rsEnd =
        reedsSheppPath.allocState()->as<heuristics::State>();
    rsStart->setXY(current_pos.x, current_pos.y);
    rsStart->setYaw(current_pos.theta);
    rsEnd->setXY(goal_pos.x, goal_pos.y);
    rsEnd->setYaw(goal_pos.theta);

    // create temporary states for iterating
    heuristics::State *next =
        reedsSheppPath.allocState()->as<heuristics::State>();

    // get reedsheeps path
    auto path = reedsSheppPath.reedsShepp(rsStart, rsEnd);
    dist = graph->getMinTurnR() * path.length();

    // iterate through each segment
    reedsSheppPath.copyState(next, rsStart);
    pos1 = current_pos;
    for (double seg = 0.0; seg <= dist; seg += increment) {
      // generate interpolated next point
      t = seg / dist;
      reedsSheppPath.interpolate(rsStart, rsEnd, t, first_time, path, next);

      // check if within bounds
      pos2.x = next->getX();
      pos2.y = next->getY();
      pos2.theta = next->getYaw();
      if (!graph->inBound(pos2)) {
        reedsSheppPath.freeState(next);
        reedsSheppPath.freeState(rsStart);
        reedsSheppPath.freeState(rsEnd);
        return false;
      }
      pos2.time = pos1.time + increment / pos1.velocity;
      graph->updateTimeIndex(pos2);
      graph->getSpeedLim(pos2);
      graph->vm->apply_vel_const(pos1.velocity, pos2.velocity);
      // TODO add steering angle

      // check if path is obstacle free
      if (graph->checkCollission(pos1, pos2, total_cost)) {
        reedsSheppPath.freeState(next);
        reedsSheppPath.freeState(rsStart);
        reedsSheppPath.freeState(rsEnd);
        return false;
      }

      // append to oneshot path
      temp_oneshot_path.push_back(pos2);

      pos1 = pos2;
    }

    // push in the goal
    temp_oneshot_path.push_back(goal_pos);

    // deallocate the states
    reedsSheppPath.freeState(next);
    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);
  } else {
    // create dubins path
    ompl::base::DubinsStateSpace dubinsPath(graph->getMinTurnR());

    // set the start and goal states
    heuristics::State *dbStart =
        dubinsPath.allocState()->as<heuristics::State>();
    heuristics::State *dbEnd = dubinsPath.allocState()->as<heuristics::State>();
    dbStart->setXY(current_pos.x, current_pos.y);
    dbStart->setYaw(current_pos.theta);
    dbEnd->setXY(goal_pos.x, goal_pos.y);
    dbEnd->setYaw(goal_pos.theta);

    // create temporary states for iterating
    heuristics::State *next = dubinsPath.allocState()->as<heuristics::State>();
    heuristics::State *ref = dubinsPath.allocState()->as<heuristics::State>();
    heuristics::State *next_abs =
        dubinsPath.allocState()->as<heuristics::State>();
    next->setXY(0., 0.);
    next->setYaw(current_pos.theta);
    dubinsPath.copyState(ref, next);

    // get dubins path
    auto path = dubinsPath.dubins(dbStart, dbEnd);

    // calculate cumulative length
    double cum_length[3];
    cum_length[0] = graph->getMinTurnR() * path.length_[0];
    cum_length[1] = cum_length[0] + graph->getMinTurnR() * path.length_[1];
    cum_length[2] = cum_length[1] + graph->getMinTurnR() * path.length_[2];
    dist = cum_length[2];

    // iterate through each segment
    int path_index = 0, steering_index;
    float seg = 0.0, cur_inc, rho;
    bool inc_path_index = false;
    pos1 = current_pos;
    // TODO don't add the previous pose in oneshot path
    temp_oneshot_path.push_back(pos1);
    while (path_index < 3) {
      // adjust distance if overshoots the end of current path segment
      if (seg + increment > cum_length[path_index]) {
        cur_inc = (cum_length[path_index] - seg);
        inc_path_index = true;
      } else {
        cur_inc = increment;
      }
      seg += cur_inc;

      // calculate current time and update time-related parameters
      pos2 = pos1;
      float time_incr = cur_inc / pos1.velocity;
      pos2.time = pos1.time + time_incr;
      graph->updateTimeIndex(pos2);
      float max_vel_inc = time_incr * graph->vm->constraint.max_accel;
      float max_steer_inc = time_incr * graph->vm->constraint.max_steering_vel;

      // TODO Choose target radius curvature without depending on vehicle motion
      // alignment
      switch (path.type_[path_index]) {
      case ompl::base::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT:
        pos2.steering_angle = graph->vm->steering_actions[0];
        break;
      case ompl::base::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT:
        pos2.steering_angle = graph->vm->steering_actions[2];
        break;
      case ompl::base::DubinsStateSpace::DubinsPathSegmentType::DUBINS_STRAIGHT:
        pos2.steering_angle = graph->vm->steering_actions[1];
        break;
      }
      // graph->vm->apply_gen_const(pos1.steering_angle, pos2.steering_angle,
      //                           max_steer_inc);
      // rho = graph->v->wheelbase / tan(fabs(pos2.steering_angle));
      rho = graph->getMinTurnR();
      const float &phi = next->getYaw();
      float v = seg;
      if (path_index > 0) {
        v -= cum_length[path_index - 1];
      }
      v /= rho;
      if (pos2.steering_angle > 0.) {
        next->setX(next->getX() + sin(phi) - sin(phi - v));
        next->setY(next->getY() + cos(phi - v) - cos(phi));
        next->setYaw(phi - v);
      } else if (pos2.steering_angle < 0.) {
        next->setX(next->getX() + sin(phi + v) - sin(phi));
        next->setY(next->getY() + cos(phi) - cos(phi + v));
        next->setYaw(phi + v);
      } else {
        next->setX(next->getX() + v * cos(phi));
        next->setY(next->getY() + v * sin(phi));
        // rho = graph->getMinTurnR();
      }
      next_abs->setX(rho * next->getX() + dbStart->getX());
      next_abs->setY(rho * next->getY() + dbStart->getY());
      dubinsPath.getSubspace(1)->enforceBounds(
          next_abs->as<ompl::base::SO2StateSpace::StateType>(1));
      pos2.x = next_abs->getX();
      pos2.y = next_abs->getY();
      pos2.theta = next->getYaw();

      // update the velocity of next pose based on dynamic constraint and max
      // vel
      graph->getSpeedLim(pos2);
      graph->vm->apply_gen_const(pos1.velocity, pos2.velocity, max_vel_inc);

      // check if path is obstacle free
      if (graph->checkCollission(pos1, pos2, total_cost)) {
        dubinsPath.freeState(dbStart);
        dubinsPath.freeState(dbEnd);
        dubinsPath.freeState(next);
        dubinsPath.freeState(next_abs);
        dubinsPath.freeState(ref);
        return false;
      }

      // append to oneshot path
      temp_oneshot_path.push_back(pos2);

      if (inc_path_index) {
        path_index += 1;
        inc_path_index = false;
        dubinsPath.copyState(ref, next);
      } else {
        dubinsPath.copyState(next, ref);
      }
      pos1 = pos2;
    }

    // deallocate the states
    dubinsPath.freeState(dbStart);
    dubinsPath.freeState(dbEnd);
    dubinsPath.freeState(next);
    dubinsPath.freeState(next_abs);
    dubinsPath.freeState(ref);
  }

  // TODO use the total cost directly and avoid maintaining the structure
  // oneshot_visited_nodes get the cost of travelling the oneshot path (exact
  // cost)
  oneshot_visited_nodes.clear();
  oneshot_cost = dist;
  int key;
  for (auto &p : temp_oneshot_path) {
    key = graph->getIndexHigher(p, first_time);
    if (!oneshot_visited_nodes.count(key)) {
      oneshot_cost += graph->getValue(key, p.time_index, p.time_frac);
      oneshot_visited_nodes.insert(key);
    } else {
      total_cost -= graph->getValue(key, p.time_index, p.time_frac);
    }
  }
  // oneshot_cost = 0.;

  return true;
}

template <typename cost_map_type>
void path_planning::A_Star<cost_map_type>::produce_path(bool start_to_goal) {
  // clear the path vector first
  path.clear();

  // clear the number of oneshot points
  num_oneshot_points = 0;

  // start fillin in the path
  if (path_found && nodes) {
    const Node *current_node = &nodes[index_las_pos];
    // copy part of the path from oneshot
    if (oneshot_path_found) {
      path.assign(oneshot_path.rbegin(), oneshot_path.rend());
      path.pop_back();

      if (store_counter) {
        int ref_counter = oneshot_counter_map[index_las_pos];
        for (auto ind : oneshot_visited_nodes) {
          oneshot_counter_map[ind] = ref_counter;
        }
      }
      num_oneshot_points = oneshot_path.size();
    } else {
      current_priority = current_node->priority;
      current_cost = current_node->cost;
    }

    while (current_node != NULL) {
      /*
      printf("(Cost, Priority) -> (%f,%f)\n", current_node->cost,
             current_node->priority);
      */
      path.push_back(current_node->pos);
      current_node = current_node->pred;
    }

    // order the path to be from start to goal instead (if required)
    if (start_to_goal)
      std::reverse(path.begin(), path.end());

  } else {
    current_cost = INFINITY;
  }
}

template <typename cost_map_type>
const int path_planning::A_Star<cost_map_type>::getSearchTime() {
  return search_time_elapsed.count();
}
