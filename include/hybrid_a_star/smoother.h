#ifndef SMOOTHER
#define SMOOTHER

/*
 * \file smoother.h
 * \brief allow for smoothing of path based on:
 * (i) Obstacle term
 * (ii) Curvature term
 * (iii) Smoothness term
 */

#include "base_graph.h"

#include <chrono>

class Smoother {
public:
  // useful for 2d coorddinates
  typedef Eigen::Vector2d Coordinate2d;

  // constructors
  Smoother() = default;

  Smoother(const unsigned int &num_iterations, const float &min_turn_r,
           const float &step_size, const float &max_obs_dist,
           const float &weight_obs, const float &weight_curv,
           const float &weight_smooth);

  Smoother(const unsigned int &num_iterations, const float &min_turn_r,
           const float &step_size, const float &max_obs_dist);

  // setters
  void setNumIter(const unsigned int &num_iterations);
  void setMinTurnR(const float &min_turn_r);
  void setStep(const float &step_size);
  void setObsThresh(const float &max_obs_dist);
  void setWeights(const float &weight_obs, const float &weight_curv,
                  const float &weight_smooth);

  // getters
  unsigned int getNumIter();
  float getMinTurnR();
  float getStep();
  float getObsThresh();
  float getWeightObs();
  float getWeightSmooth();
  float getWeightCurv();

  // smooth out the path
  template <typename cost_map_type>
  std::vector<hybrid_a_star::rob_pos> &
  smoothPath(const std::vector<hybrid_a_star::rob_pos> &path,
             const hybrid_a_star::Base_Graph<cost_map_type> &graph,
             int num_smoothed_pts = 0, bool start_to_goal = false);

  // return the microseconds for each optimisation component
  void getTimes(float &obs_time, float &curv_time, float &smooth_time);

private:
  // function to get gradient from the obstacle term
  template <typename cost_map_type>
  void obstacleTerm(const Coordinate2d &xi, const unsigned int &time_index,
                    const float &time_frac,
                    const hybrid_a_star::Base_Graph<cost_map_type> &graph,
                    Coordinate2d &gradient);

  // function to get gradient from the smoothness term
  void smoothnessTerm(const Coordinate2d &xim1, const Coordinate2d &xi,
                      const Coordinate2d &xip1, const int &i,
                      Coordinate2d &pco);

  // function to get gradient from the curvature term
  void curvatureTerm(const Coordinate2d &xim1, const Coordinate2d &xi,
                     const Coordinate2d &xip1, Coordinate2d &pc_im1,
                     Coordinate2d &pc_i, Coordinate2d &pc_ip1);

  // keep track of the timing cost for each component
  std::chrono::microseconds obs_time_, smooth_time_, curv_time_;

  // number of iterations to perform gradient descent
  unsigned int num_iterations_;

  // minimum turning radius of the car
  float min_turn_r_;
  float kappa_max_;

  // step size for gradient descent
  float step_size_;

  // maximum distance to obstacles for penalising
  float max_obs_dist_;

  // the weights for various components
  float weight_obs_;    // for penalising collision with obstacles
  float weight_curv_;   // imposes upperbound on instantaneous curvature of
                        // trajectory
  float weight_smooth_; // smoothness of the path

  // own copy of a smoothed path (avoid memory copy)
  std::vector<hybrid_a_star::rob_pos> smoothed_path;
};

#include "smoother.tpp"

#endif
