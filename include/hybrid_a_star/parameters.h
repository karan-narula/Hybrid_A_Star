#ifndef PARAMETERS
#define PARAMETERS

/*
 * \file parameters.h
 * \brief This is a collection of parameters organised into structures that are
 * used in this project. They will be configuratable via .yaml file and dynamic
 * reconfigure. \info The interpretation of the parameters will be explained via
 * wiki
 */

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace hybrid_a_star {
// useful struct to hold the positional information
struct rob_pos {
  // position and heading of the robot
  float x, y, theta;
  // direction of the robot (forward or reverse)
  int dir = 0; // 0 signifies forward direction while 1 signifies reverse
  int steering_index = 0; // robot current steering with 0th steering angle
  // the following are added to contain the dynamic information of the robot
  float time = 0., velocity = 0., steering_angle = 0.;
  unsigned int time_index = 0; // useful for knowing the temporal layer of graph
  float time_frac = 0.;        // fraction between subsequent indices

  // equality operator for this custom type
  bool operator==(const rob_pos &rhs) {
    if (x == rhs.x && y == rhs.y && theta == rhs.theta) {
      return true;
    } else {
      return false;
    }
  }
};

namespace parameters {
// parameters specific to the graph
struct graph {
  // center of the origin is at what coordinate (in meters)
  float origin[2] = {0, 0};
  // the cell size of the grid (in x, y and theta respectively)
  float resolution[3] = {1, 1, 5 * M_PI / 180};
  // the width of the entire grid
  float width = 100;
  // the height of the entire grid
  float height = 100;
};

// parameters specific to the shape/size of the vehicle
struct vehicle {
  float length;
  float width;
  float wheelbase;
  float baselink_to_front;
  // default constructor
  vehicle() : length(2.4), width(1.29), wheelbase(1.775) { updateBTF(); }

  // more specific constructor
  vehicle(const float &length, const float &width, const float &wheelbase)
      : length(length), width(width), wheelbase(wheelbase) {
    updateBTF();
  }

  // method to internally update baselink to front
  void updateBTF() {
    baselink_to_front = wheelbase + (length - wheelbase) / 2.;
  }
};

// parameters specific to the dynamic constraints of the vehicle
struct vehicle_dyn_constraints {
  float max_accel = INFINITY;
  float max_steering_vel = INFINITY;
};

// parameters specific to the motion of the vehicle
struct vehicle_motion {
  bool allow_reverse = false;
  std::vector<float> steering_actions{-10 * M_PI / 180, 0 * M_PI / 180,
                                      10 * M_PI / 180};
  int num_actions = 3;
  float chord_l_min;
  float chord_l;
  float min_turn_r;
  float max_steering;

  vehicle_dyn_constraints constraint;
  float time_incr;
  float min_vel;
  float max_vel_inc, max_steer_inc;

  vehicle_motion(const graph &g, const float &chord_l_factor,
                 const vehicle &v) {
    // set chord lengths
    set_values(g, chord_l_factor);
    // set the minimum turning radius
    set_min_r(v);
    // default minimum velocity & time increment
    min_vel = 0.;
    time_incr = 0.;
  }
  vehicle_motion(const graph &g, const float &chord_l_factor, const vehicle &v,
                 const vehicle_dyn_constraints &constraint,
                 const float &time_incr)
      : vehicle_motion(g, chord_l_factor, v) {
    this->constraint = constraint;
    this->time_incr = time_incr;
    // set the minimum velocity
    set_min_v();
  }
  vehicle_motion(bool allow_reverse, const std::vector<float> &sas,
                 const graph &g, const float &chord_l_factor, const vehicle &v)
      : allow_reverse(allow_reverse) {
    // set chord lengths
    set_values(g, chord_l_factor);
    // store the size of the vector
    num_actions = sas.size();

    // copy over the steering actions
    steering_actions = sas;

    // set the minimum turning radius
    set_min_r(v);
    // default minimum velocity & time increment
    min_vel = 0.;
    time_incr = 0.;
  }
  vehicle_motion(bool allow_reverse, const std::vector<float> &sas,
                 const graph &g, const float &chord_l_factor, const vehicle &v,
                 const vehicle_dyn_constraints &constraint,
                 const float &time_incr)
      : vehicle_motion(allow_reverse, sas, g, chord_l_factor, v) {
    this->constraint = constraint;
    this->time_incr = time_incr;
    // set the minimum velocity
    set_min_v();
  }

  // function to set minimum turning radius
  void set_min_r(const vehicle &v) {
    std::vector<float> abs_steerings(steering_actions);
    std::transform(steering_actions.begin(), steering_actions.end(),
                   abs_steerings.begin(), fabs);
    max_steering =
        *std::max_element(abs_steerings.begin(), abs_steerings.end());
    min_turn_r = v.wheelbase / tan(max_steering);
  }
  // function to set minimum velocity based on minimum chord legth and time
  // increment
  void set_min_v() {
    min_vel = chord_l_min / time_incr;
    if (std::isinf(min_vel)) {
      min_vel = 0.;
    }
    max_vel_inc = time_incr * constraint.max_accel;
    max_steer_inc = time_incr * constraint.max_steering_vel;
  }
  // function to reset the values similarly to a constructor
  void set_values(const graph &g, const float &chord_l_factor) {
    chord_l_min = hypot(g.resolution[0], g.resolution[1]);
    chord_l = chord_l_factor * chord_l_min;
  }
  // constraint functions
  inline void apply_gen_const(const float &c, float &n, const float &max_inc) const {
    float inc = n - c;
    if (fabs(inc) > max_inc) {
      n = c + copysign(max_inc, inc);
    }
  }
  inline void apply_steer_const(const float &c_sa, float &n_sa) const {
    apply_gen_const(c_sa, n_sa, max_steer_inc);
  }
  inline void apply_vel_const(const float &c_v, float &n_v) const {
    apply_gen_const(c_v, n_v, max_vel_inc);
  }
};

// thresholds for the stopping criteria
struct stop_explore {
  // within 3 degrees
  float heading_threshold_error = 3 * M_PI / 180;
  // within 10 cm
  float distance_threshold_error = 0.1;
};

// penalties during the motion
struct penalties {
  // units are in number of cells/diagonals

  // penalises when the direction was changed
  float change_dir = 2.0;
  // penalises when the heading is drastically changed
  float change_head = 1.1;
  // penalises when the steering index of the vehicle is changed
  float change_steering_index = 1.05;
  // penalises when the vehicle is reversing
  float reversing = 2.0;
  // penalises when the vehicle has changed its velocity based on dynamic
  // constraint. Ff velocity is adapted based on the cost-map itself, then this
  // factor would be redundant
  float change_velocity = 1.0;
  // penalises when the vehicle has changed its steering angle based on dynamic
  // constraint; this would happen when the steering index changes anyway so it
  // is encapsulated in the other factor
  float change_steering = 1.0;
};

struct planner_intrinsics {
  // factor to move in hybrid a-star
  float default_chord_l_factor = 1.0;

  // perform precomputation in hybrid a star?
  bool precompute_flag = true;

  // attempt oneshot dubins/reedsheeps?
  bool oneshot_flag = false;

  // maxmimum number of iterations for search
  unsigned int max_num_iterations = 1e5;
};

// default scenario of YAML file
static const std::string default_scenario = "default";

// default intrinsic for twoD planning
static const planner_intrinsics twoD_intrinsics{1.0, false, 100000};

// function to return graph parameters based on the yaml file
graph getGraphParamYaml(const std::string &filename,
                        const std::string &scenario,
                        const std::string &default_scenario, int &nrows,
                        int &ncols);

// function to return vehicle motion model for 2D cell-wise navigation
vehicle_motion getPrecomputeVM(const graph &graph_param, const vehicle &v);
void getPrecomputeVM(const graph &graph_param, const vehicle &v,
                     vehicle_motion &vm);

} // namespace parameters
} // namespace hybrid_a_star

#endif
