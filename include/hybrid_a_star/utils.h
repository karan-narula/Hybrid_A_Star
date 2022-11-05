#ifndef UTILS
#define UTILS

/*
 * \file utils.h
 * \brief contains function defintions for heuristic functions, cost functions,
 * kinematic models, stopping criterias and global costs \info some should be
 * provided in the wiki
 */

#include "parameters.h"
#include <assert.h>
#include <functional>
#include <iostream>

// from the ompl, include stuffs for dubins and reedsheep curves
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace hybrid_a_star {
// forward declare stuff in base_graph
template <typename cost_map_type> class Base_Graph;

// name space for stopping criteria
namespace stopping_crit {
// general type def for a stopping criteria function
template <typename cost_map_type>
using gen_stop_func = bool (*)(const rob_pos &, const rob_pos &,
                               const Base_Graph<cost_map_type> &graph,
                               const parameters::stop_explore &);

// helper function to bind angle between [-180,180] degrees
inline void bind_npi_pi(float &heading) {
  // normalize it to be betweeen [0, 2*pi]
  heading = fmod(heading + M_PI, 2 * M_PI);
  if (heading < 0)
    heading += 2 * M_PI;
  heading -= M_PI;
}

// never stops (no early exit, evaluate all cells)
template <typename cost_map_type>
inline bool no_stop(const rob_pos &current, const rob_pos &goal,
                    const Base_Graph<cost_map_type> &graph,
                    const parameters::stop_explore &stop_param) {
  return false;
}

// physically in the same 2D cell and within heading threshold error
template <typename cost_map_type>
inline bool xyInd_headRaw(const rob_pos &current, const rob_pos &goal,
                          const Base_Graph<cost_map_type> &graph,
                          const parameters::stop_explore &stop_param) {
  // find heading discrepancy
  float heading_discrep = current.theta - goal.theta;
  // bind it to be between -pi and pi
  stopping_crit::bind_npi_pi(heading_discrep);
  // take absolute value
  heading_discrep = fabs(heading_discrep);

  bool success;
  return graph.getIndex(current, success) == graph.getIndex(goal, success) &&
         heading_discrep <= stop_param.heading_threshold_error;
}

// within distance and heading threshold error
template <typename cost_map_type>
inline bool xyRaw_headRaw(const rob_pos &current, const rob_pos &goal,
                          const Base_Graph<cost_map_type> &graph,
                          const parameters::stop_explore &stop_param) {
  // find heading discrepancy
  float heading_discrep = current.theta - goal.theta;
  // bind it to be between -pi and pi
  stopping_crit::bind_npi_pi(heading_discrep);
  // take absolute value
  heading_discrep = fabs(heading_discrep);

  // find distance discrepancy
  float distance_discrep = hypot(current.x - goal.x, current.y - goal.y);

  return distance_discrep <= stop_param.distance_threshold_error &&
         heading_discrep <= stop_param.heading_threshold_error;
}

// physically in the same 2D cell
template <typename cost_map_type>
inline bool xyInd(const rob_pos &current, const rob_pos &goal,
                  const Base_Graph<cost_map_type> &graph,
                  const parameters::stop_explore &stop_param) {
  bool success;
  return graph.getIndex(current, success) == graph.getIndex(goal, success);
}

// physically in the same higher dimensional cell
template <typename cost_map_type>
inline bool xyInd_headInd(const rob_pos &current, const rob_pos &goal,
                          const Base_Graph<cost_map_type> &graph,
                          const parameters::stop_explore &stop_param) {
  bool success;
  return graph.getIndexHigher(current, success) ==
         graph.getIndexHigher(goal, success);
}

} // namespace stopping_crit

// name space for kinematic models
namespace kinematic_models {
// create typedef for functions in kinematic models' name space
typedef hybrid_a_star::rob_pos (*gen_kinematic_func)(
    const hybrid_a_star::rob_pos &,
    const hybrid_a_star::parameters::vehicle_motion &,
    const hybrid_a_star::parameters::vehicle &, const int &, const int &);

// follows the definition from above for non-holonomic vehicle with steering
inline rob_pos
non_holo_car_steer(const rob_pos &curr_pos,
                   const hybrid_a_star::parameters::vehicle_motion &vm,
                   const hybrid_a_star::parameters::vehicle &v,
                   const int &steering_index, const int &sign_chord_l) {
  // check if steering index is valid
  assert(steering_index >= 0 && steering_index < vm.num_actions);

  int dir = (vm.chord_l > 0) ? 0 : 1;
  float x = curr_pos.x + sign_chord_l * vm.chord_l * cos(curr_pos.theta);
  float y = curr_pos.y + sign_chord_l * vm.chord_l * sin(curr_pos.theta);
  float steering_angle = vm.steering_actions[steering_index];
  float theta = curr_pos.theta +
                sign_chord_l * (vm.chord_l / v.wheelbase) * tan(steering_angle);
  // binding the angles to be within -pi and pi
  stopping_crit::bind_npi_pi(theta);
  rob_pos next_pos{x,
                   y,
                   theta,
                   dir,
                   steering_index,
                   curr_pos.time,
                   curr_pos.velocity,
                   steering_angle};

  return next_pos;
}

// similar to above but uses vehicle's current velocity & fixed time increment
// for the next position
inline rob_pos non_holo_car_steer_dyn_fixed_dt(
    const rob_pos &curr_pos,
    const hybrid_a_star::parameters::vehicle_motion &vm,
    const hybrid_a_star::parameters::vehicle &v, const int &steering_index,
    const int &sign_chord_l) {
  // check if steering index is valid
  assert(steering_index >= 0 && steering_index < vm.num_actions);

  int dir = (vm.chord_l > 0) ? 0 : 1;
  float chord_l = curr_pos.velocity * vm.time_incr;
  float x = curr_pos.x + sign_chord_l * chord_l * cos(curr_pos.theta);
  float y = curr_pos.y + sign_chord_l * chord_l * sin(curr_pos.theta);
  float steering_angle = vm.steering_actions[steering_index];
  vm.apply_steer_const(curr_pos.steering_angle, steering_angle);
  float theta = curr_pos.theta +
                sign_chord_l * (chord_l / v.wheelbase) * tan(steering_angle);
  float time = curr_pos.time + vm.time_incr;
  // binding the angles to be within -pi and pi
  stopping_crit::bind_npi_pi(theta);
  rob_pos next_pos{x,
                   y,
                   theta,
                   dir,
                   steering_index,
                   time,
                   curr_pos.velocity,
                   steering_angle};

  return next_pos;
}

// similar to above but uses fixed distance by varying dt
inline rob_pos non_holo_car_steer_dyn_fixed_dist(
    const rob_pos &curr_pos,
    const hybrid_a_star::parameters::vehicle_motion &vm,
    const hybrid_a_star::parameters::vehicle &v, const int &steering_index,
    const int &sign_chord_l) {
  // check if steering index is valid
  assert(steering_index >= 0 && steering_index < vm.num_actions);

  int dir = (vm.chord_l > 0) ? 0 : 1;
  float x = curr_pos.x + sign_chord_l * vm.chord_l * cos(curr_pos.theta);
  float y = curr_pos.y + sign_chord_l * vm.chord_l * sin(curr_pos.theta);
  float steering_angle = vm.steering_actions[steering_index];
  float time_incr = vm.chord_l / curr_pos.velocity;
  float max_inc = time_incr * vm.constraint.max_steering_vel;
  vm.apply_gen_const(curr_pos.steering_angle, steering_angle, max_inc);
  float theta = curr_pos.theta +
                sign_chord_l * (vm.chord_l / v.wheelbase) * tan(steering_angle);
  float time = curr_pos.time + time_incr;
  // binding the angles to be within -pi and pi
  stopping_crit::bind_npi_pi(theta);
  rob_pos next_pos{x,
                   y,
                   theta,
                   dir,
                   steering_index,
                   time,
                   curr_pos.velocity,
                   steering_angle};

  return next_pos;
}

// for grid-based motion
inline rob_pos insane_car(const rob_pos &curr_pos,
                          const hybrid_a_star::parameters::vehicle_motion &vm,
                          const hybrid_a_star::parameters::vehicle &v,
                          const int &steering_index, const int &sign_chord_l) {
  float x, y;
  switch (steering_index) {
  case 0:
    x = curr_pos.x + sign_chord_l * vm.steering_actions[0];
    y = curr_pos.y;
    break;
  case 1:
    x = curr_pos.x;
    y = curr_pos.y + sign_chord_l * vm.steering_actions[1];
    break;
  case 2:
    x = curr_pos.x + sign_chord_l * vm.steering_actions[0];
    y = curr_pos.y + sign_chord_l * vm.steering_actions[1];
    break;
  case 3:
    x = curr_pos.x - sign_chord_l * vm.steering_actions[0];
    y = curr_pos.y + sign_chord_l * vm.steering_actions[1];
    break;
  }

  rob_pos next_pos{x, y, curr_pos.theta, curr_pos.dir, steering_index};

  return next_pos;
}

inline rob_pos
insane_car_update_dt(const rob_pos &curr_pos,
                     const hybrid_a_star::parameters::vehicle_motion &vm,
                     const hybrid_a_star::parameters::vehicle &v,
                     const int &steering_index, const int &sign_chord_l) {
  float x, y, time;
  switch (steering_index) {
  case 0:
    x = curr_pos.x + sign_chord_l * vm.steering_actions[0];
    y = curr_pos.y;
    time = curr_pos.time + vm.steering_actions[0] / curr_pos.velocity;
    break;
  case 1:
    x = curr_pos.x;
    y = curr_pos.y + sign_chord_l * vm.steering_actions[1];
    time = curr_pos.time + vm.steering_actions[1] / curr_pos.velocity;
    break;
  case 2:
    x = curr_pos.x + sign_chord_l * vm.steering_actions[0];
    y = curr_pos.y + sign_chord_l * vm.steering_actions[1];
    time =
        curr_pos.time + hypot(vm.steering_actions[0], vm.steering_actions[1]) /
                            curr_pos.velocity;
    break;
  case 3:
    x = curr_pos.x - sign_chord_l * vm.steering_actions[0];
    y = curr_pos.y + sign_chord_l * vm.steering_actions[1];
    time =
        curr_pos.time + hypot(vm.steering_actions[0], vm.steering_actions[1]) /
                            curr_pos.velocity;
    break;
  }

  rob_pos next_pos{x, y, curr_pos.theta, curr_pos.dir, steering_index, time};

  return next_pos;
}

} // namespace kinematic_models

// name space for all the implemented heuristics
namespace heuristics {
// general type def for heuristic functions
template <typename cost_map_type>
using gen_heuristic_func = float (*)(const rob_pos &, const rob_pos &,
                                     const Base_Graph<cost_map_type> &);

// Euclidean distance
template <typename cost_map_type>
inline float euclidean_distance(const rob_pos &c_pos, const rob_pos &n_pos,
                                const Base_Graph<cost_map_type> &graph) {
  return hypot(c_pos.x - n_pos.x, c_pos.y - n_pos.y);
}

inline float euclidean_distance(const rob_pos &c_pos, const rob_pos &n_pos) {
  return hypot(c_pos.x - n_pos.x, c_pos.y - n_pos.y);
}

// no heuristic (return 0)
template <typename cost_map_type>
inline float no_heuristic(const rob_pos &c_pos, const rob_pos &n_pos,
                          const Base_Graph<cost_map_type> &graph) {
  return 0;
}

typedef ompl::base::SE2StateSpace::StateType State;
// dubin's length, calculate on the fly
template <typename cost_map_type>
inline float dubins_length(const rob_pos &c_pos, const rob_pos &n_pos,
                           const Base_Graph<cost_map_type> &graph) {
  ompl::base::DubinsStateSpace dubinsPath(graph.getMinTurnR());
  auto *dbStart = dubinsPath.allocState()->as<State>();
  auto *dbEnd = dubinsPath.allocState()->as<State>();
  dbStart->setXY(c_pos.x, c_pos.y);
  dbStart->setYaw(c_pos.theta);
  dbEnd->setXY(n_pos.x, n_pos.y);
  dbEnd->setYaw(n_pos.theta);
  float dubinsCost = dubinsPath.distance(dbStart, dbEnd);

  dubinsPath.freeState(dbStart);
  dubinsPath.freeState(dbEnd);

  return dubinsCost;
}

// reedsheep's length, calculate on the fly
template <typename cost_map_type>
inline float reedsheeps_length(const rob_pos &c_pos, const rob_pos &n_pos,
                               const Base_Graph<cost_map_type> &graph) {
  ompl::base::ReedsSheppStateSpace reedsSheppPath(graph.getMinTurnR());
  auto *rsStart = reedsSheppPath.allocState()->as<State>();
  auto *rsEnd = reedsSheppPath.allocState()->as<State>();
  rsStart->setXY(c_pos.x, c_pos.y);
  rsStart->setYaw(c_pos.theta);
  rsEnd->setXY(n_pos.x, n_pos.y);
  rsEnd->setYaw(n_pos.theta);
  float reedsSheepCost = reedsSheppPath.distance(rsStart, rsEnd);

  reedsSheppPath.freeState(rsStart);
  reedsSheppPath.freeState(rsEnd);

  return reedsSheepCost;
}

} // namespace heuristics

// namespace for global cost of moving from one position to the other
namespace global_cost {
// general type def for the global cost functions
typedef float (*gen_globalcost_func)(const rob_pos &, const rob_pos &,
                                     const parameters::vehicle_motion &,
                                     const parameters::penalties &);

// function penalises distance alone (used for purely 2D)
inline float raw_distance(const rob_pos &c_pos, const rob_pos &n_pos,
                          const parameters::vehicle_motion &vm,
                          const parameters::penalties &penalty_params) {
  return heuristics::euclidean_distance(c_pos, n_pos);
}

// simply return chord length
inline float chord_length(const rob_pos &c_pos, const rob_pos &n_pos,
                          const parameters::vehicle_motion &vm,
                          const parameters::penalties &penalty_params) {
  return vm.chord_l;
}

// return distance based on current velocity and time increment
inline float vdt(const rob_pos &c_pos, const rob_pos &n_pos,
                 const parameters::vehicle_motion &vm,
                 const parameters::penalties &penalty_params) {
  return c_pos.velocity * (n_pos.time - c_pos.time);
}

// chord length and penalise change in heading
inline float
chord_length_penal_headRaw_add(const rob_pos &c_pos, const rob_pos &n_pos,
                               const parameters::vehicle_motion &vm,
                               const parameters::penalties &penalty_params) {
  return vm.chord_l +
         vm.chord_l_min *
             (penalty_params.change_dir * abs(c_pos.dir - n_pos.dir) +
              penalty_params.change_head * fabs(c_pos.theta - n_pos.theta));
}

// function penalises raw heading (additive)
inline float penal_headRaw_add(const rob_pos &c_pos, const rob_pos &n_pos,
                               const parameters::vehicle_motion &vm,
                               const parameters::penalties &penalty_params) {
  // calculate heading discrepancy and bind it to be within range
  float heading_discrep = (c_pos.theta - n_pos.theta);
  stopping_crit::bind_npi_pi(heading_discrep);

  return heuristics::euclidean_distance(c_pos, n_pos) +
         vm.chord_l_min *
             (penalty_params.change_dir * abs(c_pos.dir - n_pos.dir) +
              penalty_params.change_head * fabs(heading_discrep));
}

// function penalises instead the change in steering (factors are multiplicative
// not additive)
inline float penal_steer_mult(const rob_pos &c_pos, const rob_pos &n_pos,
                              const parameters::vehicle_motion &vm,
                              const parameters::penalties &penalty_params) {
  float factor;
  // penalise change in steering and direction
  bool change_steering_index = (c_pos.steering_index != n_pos.steering_index);
  bool change_dir = (c_pos.dir != n_pos.dir);
  if (change_steering_index || change_dir) {
    factor = 1.0;
    if (change_steering_index) {
      factor *= penalty_params.change_steering_index;
    }
    if (change_dir) {
      factor *= penalty_params.change_dir;
    }
  } else {
    factor = 0.;
  }

  return heuristics::euclidean_distance(c_pos, n_pos) + vm.chord_l_min * factor;
}

// similar to above but also penalises reverse direction (factors are
// multiplicative not additive)
inline float
penal_steer_reverse_mult(const rob_pos &c_pos, const rob_pos &n_pos,
                         const parameters::vehicle_motion &vm,
                         const parameters::penalties &penalty_params) {
  float factor;

  bool reverse = c_pos.dir;
  bool change_steering_index = (c_pos.steering_index != n_pos.steering_index);
  bool change_dir = (c_pos.dir != n_pos.dir);

  // penalise change in steering, direction and moving in reverse
  if (reverse || change_steering_index || change_dir) {
    factor = 1.0;
    if (reverse) {
      factor *= penalty_params.reversing;
    }
    if (change_steering_index) {
      factor *= penalty_params.change_steering_index;
    }
    if (change_dir) {
      factor *= penalty_params.change_dir;
    }
  } else {
    factor = 0.;
  }

  return heuristics::euclidean_distance(c_pos, n_pos) + vm.chord_l_min * factor;
}

// similar to the above but also considers change in velocity
inline float penal_steer_vel_mult(const rob_pos &c_pos, const rob_pos &n_pos,
                                  const parameters::vehicle_motion &vm,
                                  const parameters::penalties &penalty_params) {
  float factor;
  // penalise change in steering, direction and velocity
  bool change_steering_index = (c_pos.steering_index != n_pos.steering_index);
  bool change_dir = (c_pos.dir != n_pos.dir);
  bool change_vel = (c_pos.velocity != n_pos.velocity);

  if (change_steering_index || change_dir || change_vel) {
    factor = 1.0;
    if (change_steering_index) {
      factor *= penalty_params.change_steering_index;
    }
    if (change_dir) {
      factor *= penalty_params.change_dir;
    }
    if (change_vel) {
      factor *= penalty_params.change_velocity *
                fabs(c_pos.velocity - n_pos.velocity);
    }
  } else {
    factor = 0.;
  }

  return c_pos.velocity * (n_pos.time - c_pos.time) + vm.chord_l_min * factor;
}

inline float
penal_steer_vel_reverse_mult(const rob_pos &c_pos, const rob_pos &n_pos,
                             const parameters::vehicle_motion &vm,
                             const parameters::penalties &penalty_params) {
  float factor;

  bool reverse = c_pos.dir;
  bool change_steering_index = (c_pos.steering_index != n_pos.steering_index);
  bool change_dir = (c_pos.dir != n_pos.dir);
  bool change_vel = (c_pos.velocity != n_pos.velocity);

  // penalise change in steering, direction and moving in reverse
  if (reverse || change_steering_index || change_dir || change_vel) {
    factor = 1.0;
    if (reverse) {
      factor *= penalty_params.reversing;
    }
    if (change_steering_index) {
      factor *= penalty_params.change_steering_index;
    }
    if (change_dir) {
      factor *= penalty_params.change_dir;
    }
    if (change_vel) {
      factor *= penalty_params.change_velocity *
                fabs(c_pos.velocity - n_pos.velocity);
    }
  } else {
    factor = 0.;
  }

  return c_pos.velocity * (n_pos.time - c_pos.time) + vm.chord_l_min * factor;
}

// similar to above but factors are additive instead of multiplicative
inline float
penal_steer_vel_reverse_add(const rob_pos &c_pos, const rob_pos &n_pos,
                            const parameters::vehicle_motion &vm,
                            const parameters::penalties &penalty_params) {
  return c_pos.velocity * (n_pos.time - c_pos.time) +
         vm.chord_l_min *
             (penalty_params.change_dir * abs(c_pos.dir - n_pos.dir) +
              penalty_params.change_steering_index *
                  ((c_pos.steering_index - n_pos.steering_index) != 0 ? 1 : 0) +
              penalty_params.reversing * c_pos.dir +
              penalty_params.change_velocity *
                  fabs(c_pos.velocity - n_pos.velocity));
}
} // namespace global_cost

// parameters and functions for when the grid cell was not visited during
// precomputation
namespace precompute {
static float default_value_when_not_found = 100;

// signature of default function when not found takes on the same as heuristic
// function
template <typename cost_map_type>
using gen_default_func_when_not_found =
    float (*)(const rob_pos &, const rob_pos &,
              const Base_Graph<cost_map_type> &, const float &);

// simple function that just returns the default value
template <typename cost_map_type>
float default_return(const rob_pos &c_pos, const rob_pos &n_pos,
                     const Base_Graph<cost_map_type> &graph,
                     const float &dvwnf) {
  return dvwnf;
}

} // namespace precompute

// extend the parameters' name space with default heuristics and kinematic model
// for dijsktra and vanilla a-star
namespace parameters {
// the model used for vanilla 2D navigation during precomputation
static const kinematic_models::gen_kinematic_func two_d_pre_model =
    &kinematic_models::insane_car;

// heuristic for vanilla a-star
template <typename cost_map_type>
static const heuristics::gen_heuristic_func<cost_map_type> vani_heuristic =
    &heuristics::euclidean_distance<cost_map_type>;

// heuristic for dijsktra
template <typename cost_map_type>
static const heuristics::gen_heuristic_func<cost_map_type> dijsktra_heuristic =
    &heuristics::no_heuristic<cost_map_type>;

} // namespace parameters

} // namespace hybrid_a_star

#endif
