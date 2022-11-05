#include "hybrid_a_star/smoother.h"

// CUSP detection
inline bool isCusp(const std::vector<hybrid_a_star::rob_pos> &path,
                   const int &i) {
  if (i - 2 >= 0) {
    return path[i - 2].dir != path[i - 1].dir ||
           path[i - 1].dir != path[i].dir || path[i].dir != path[i + 1].dir;
  } else {
    return path[i - 1].dir != path[i].dir || path[i].dir != path[i + 1].dir;
  }
}

// function for clamping
inline float clamp(const float &num, const float &lower, const float &upper) {
  return std::max(lower, std::min(upper, num));
}

// function for returning orthogonal complement of two vectors
inline Smoother::Coordinate2d ort(const Smoother::Coordinate2d &a,
                                  const Smoother::Coordinate2d &b) {
  Smoother::Coordinate2d c;
  c = a - b * a.dot(b) / b.squaredNorm();

  return c;
}

Smoother::Smoother(const unsigned int &num_iterations, const float &min_turn_r,
                   const float &step_size, const float &max_obs_dist,
                   const float &weight_obs, const float &weight_curv,
                   const float &weight_smooth)
    : num_iterations_(num_iterations), min_turn_r_(min_turn_r),
      step_size_(step_size), max_obs_dist_(max_obs_dist),
      weight_obs_(weight_obs), weight_curv_(weight_curv),
      weight_smooth_(weight_smooth), kappa_max_(1 / min_turn_r_) {}

Smoother::Smoother(const unsigned int &num_iterations, const float &min_turn_r,
                   const float &step_size, const float &max_obs_dist)
    : num_iterations_(num_iterations), min_turn_r_(min_turn_r),
      step_size_(step_size), max_obs_dist_(max_obs_dist),
      kappa_max_(1 / min_turn_r_) {}

void Smoother::setNumIter(const unsigned int &num_iterations) {
  num_iterations_ = num_iterations;
}

void Smoother::setMinTurnR(const float &min_turn_r) {
  min_turn_r_ = min_turn_r;
  kappa_max_ = 1 / min_turn_r_;
}

void Smoother::setStep(const float &step_size) { step_size_ = step_size; }

void Smoother::setObsThresh(const float &max_obs_dist) {
  max_obs_dist_ = max_obs_dist;
}

void Smoother::setWeights(const float &weight_obs, const float &weight_curv,
                          const float &weight_smooth) {
  weight_obs_ = weight_obs;
  weight_curv_ = weight_curv;
  weight_smooth_ = weight_smooth;
}

unsigned int Smoother::getNumIter() { return num_iterations_; }

float Smoother::getMinTurnR() { return min_turn_r_; }

float Smoother::getStep() { return step_size_; }

float Smoother::getObsThresh() { return max_obs_dist_; }

float Smoother::getWeightObs() { return weight_obs_; }

float Smoother::getWeightSmooth() { return weight_smooth_; }

float Smoother::getWeightCurv() { return weight_curv_; }

template <typename cost_map_type>
std::vector<hybrid_a_star::rob_pos> &
Smoother::smoothPath(const std::vector<hybrid_a_star::rob_pos> &path,
                     const hybrid_a_star::Base_Graph<cost_map_type> &graph,
                     int num_smoothed_pts, bool start_to_goal) {

  smoothed_path = path;

  // calculate total weight
  float total_weight = weight_obs_ + weight_curv_ + weight_smooth_;
  float factor = step_size_ / total_weight;

  // directly return if total weight is smaller than threshold
  if (total_weight < 1e-3)
    return smoothed_path;

  // initialise the number of microseconds
  obs_time_ = smooth_time_ = curv_time_ = std::chrono::microseconds::zero();

  // directly return if empty path
  int num_pts = smoothed_path.size();
  if (num_pts == 0)
    return smoothed_path;

  // check point number to smooth the path until
  int pt_start = num_smoothed_pts - 1;
  int pt_end = num_pts - 1;
  if (pt_start < 1) {
    pt_start = 1;
  } else if (pt_start >= pt_end) {
    // path considered to already be OK
    return smoothed_path;
  }

  // predefined storage units for gradients and cusp
  // bool iscusp[smoothed_path.size()];
  Coordinate2d obs_grad[num_pts];
  Coordinate2d smooth_precomp[num_pts];
  Coordinate2d curv_precomp[3][num_pts];

  smooth_precomp[pt_start - 1] << 0, 0;
  smooth_precomp[pt_end] << 0, 0;
  curv_precomp[0][pt_start - 1] << 0, 0;
  curv_precomp[1][pt_start - 1] << 0, 0;
  curv_precomp[2][pt_start - 1] << 0, 0;
  curv_precomp[0][pt_end] << 0, 0;
  curv_precomp[1][pt_end] << 0, 0;
  curv_precomp[2][pt_end] << 0, 0;

  // predefined some vectors
  Coordinate2d correction, xim1, xi, xip1;

  // depending on the path order, change the direction of heading
  int heading_dir = 1;
  if (start_to_goal)
    heading_dir = -1;

  // iterate for the number of specified iterations
  for (int i = 0; i < num_iterations_; i++) {
    // iterate through current path and store gradients
    for (int j = pt_start; j < pt_end; j++) {
      xim1 << smoothed_path[j - 1].x, smoothed_path[j - 1].y;
      xi << smoothed_path[j].x, smoothed_path[j].y;
      xip1 << smoothed_path[j + 1].x, smoothed_path[j + 1].y;

      // keep the points fixed if they are a cusp point or adjacent to one
      if (isCusp(smoothed_path, j)) {
        obs_grad[j] << 0, 0;
        smooth_precomp[j] << 0, 0;
        curv_precomp[0][j] << 0, 0;
        curv_precomp[1][j] << 0, 0;
        curv_precomp[2][j] << 0, 0;
        continue;
      }
      // gradient contribution from the obstacles term
      if (weight_obs_ > 0) {
        auto timer_start_obs = std::chrono::steady_clock::now();
        obstacleTerm(xi, smoothed_path[j].time_index,
                     smoothed_path[j].time_frac, graph, obs_grad[j]);
        obs_time_ += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - timer_start_obs);
      }
      // precompute smoothness quantity (not gradient yet!)
      if (weight_smooth_ > 0) {
        auto timer_start_smooth = std::chrono::steady_clock::now();
        smoothnessTerm(xim1, xi, xip1, j, smooth_precomp[j]);
        smooth_time_ += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - timer_start_smooth);
      }
      // precompute curvature quantity (not gradient yet!)
      if (weight_curv_ > 0) {
        auto timer_start_curv = std::chrono::steady_clock::now();
        curvatureTerm(xim1, xi, xip1, curv_precomp[0][j], curv_precomp[1][j],
                      curv_precomp[2][j]);
        curv_time_ += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - timer_start_curv);
      }
    }

    // perform gradient descent base on precomputed stuff
    for (int j = pt_start; j < pt_end; j++) {
      // keep the points fixed if they are a cusp point or adjacent to one
      if (isCusp(smoothed_path, j))
        continue;

      xi << smoothed_path[j].x, smoothed_path[j].y;
      correction << 0.0, 0.0;

      // gradient contribution from the obstacles term
      if (weight_obs_ > 0) {
        auto timer_start_obs = std::chrono::steady_clock::now();
        correction -= obs_grad[j];
        obs_time_ += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - timer_start_obs);
      }

      // gradient contribution from the smoothness term
      if (weight_smooth_ > 0) {
        auto timer_start_smooth = std::chrono::steady_clock::now();
        correction -= 2 * weight_smooth_ *
                      (-2 * smooth_precomp[j] + smooth_precomp[j + 1] +
                       smooth_precomp[j - 1]);
        smooth_time_ += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - timer_start_smooth);
      }

      // gradient contribution from the curvature term
      if (weight_curv_ > 0) {
        auto timer_start_curv = std::chrono::steady_clock::now();
        correction -= 2 * weight_curv_ *
                      (curv_precomp[0][j - 1] + curv_precomp[1][j] +
                       curv_precomp[2][j + 1]);
        curv_time_ += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - timer_start_curv);
      }

      // perform gradient descent
      xi += factor * correction;
      // check if corrected point is still on grid
      if (!graph.inBound(xi))
        continue;
      smoothed_path[j].x = xi[0];
      smoothed_path[j].y = xi[1];
      if (j > 1) {
        Coordinate2d dxi =
            heading_dir *
            (Coordinate2d{smoothed_path[j - 1].x, smoothed_path[j - 1].y} - xi);
        smoothed_path[j - 1].theta = std::atan2(dxi(1), dxi(0));
      }
    }
    // set heading of the second last point before the goal
    Coordinate2d dxi =
        heading_dir *
        (xi - Coordinate2d{smoothed_path.back().x, smoothed_path.back().y});
    smoothed_path[pt_end - 1].theta = std::atan2(dxi(1), dxi(0));
  }

  return smoothed_path;
}

void Smoother::getTimes(float &obs_time, float &curv_time, float &smooth_time) {
  obs_time = (float)obs_time_.count() / 1e3;
  curv_time = (float)curv_time_.count() / 1e3;
  smooth_time = (float)smooth_time_.count() / 1e3;
}

// function to increment gradient from the obstacle term
template <typename cost_map_type>
void Smoother::obstacleTerm(
    const Coordinate2d &xi, const unsigned int &time_index,
    const float &time_frac,
    const hybrid_a_star::Base_Graph<cost_map_type> &graph,
    Coordinate2d &gradient) {

  Coordinate2d obs_pos;
  float obsDist;
  if (graph.nearest_obstacle(xi, time_index, time_frac, max_obs_dist_, obs_pos,
                             obsDist)) {
    Coordinate2d obsVect = xi - obs_pos;
    if (obsDist != 0) {
      gradient =
          weight_obs_ * 2 * (obsDist - max_obs_dist_) * obsVect / obsDist;
    } else {
      gradient = weight_obs_ * 2 * (obsDist - max_obs_dist_) * obsVect;
    }
  } else {
    gradient << 0, 0;
  }
}

void Smoother::smoothnessTerm(const Coordinate2d &xim1, const Coordinate2d &xi,
                              const Coordinate2d &xip1, const int &i,
                              Coordinate2d &pco) {
  pco = xip1 - 2 * xi + xim1;
}

// function to get gradient from the curvature term
void Smoother::curvatureTerm(const Coordinate2d &xim1, const Coordinate2d &xi,
                             const Coordinate2d &xip1, Coordinate2d &pc_im1,
                             Coordinate2d &pc_i, Coordinate2d &pc_ip1) {

  // deltas coordinates
  Coordinate2d delta_xi = xi - xim1;
  Coordinate2d delta_xip1 = xip1 - xi;

  Coordinate2d gradient{0, 0};
  // length of the above vectors
  float length_delta_xi = delta_xi.norm();
  float length_delta_xip1 = delta_xip1.norm();

  // ensures that the lengths are non-zero (present in denominator)
  if (length_delta_xi > 0 && length_delta_xip1 > 0) {
    // change in angle at vertex
    float delta_ang = std::acos(clamp((delta_xi.dot(delta_xip1)) /
                                          (length_delta_xi * length_delta_xip1),
                                      -1, 1));
    float kappa = delta_ang / length_delta_xi;

    // if the curvature is smaller than maximum, objective function is
    // satisfied
    if (kappa > kappa_max_) {
      // intermediate value
      float u = 1 / (std::sqrt(1 - std::pow(std::cos(delta_ang), 2)) *
                     length_delta_xi);

      // length of the vectors
      float length_xi = xi.norm();
      float length_xip1 = xip1.norm();

      // orthogonal complement vectors
      Coordinate2d p1, p2;
      p1 = ort(xi, -xip1) / (length_xi * length_xip1);
      p2 = ort(-xip1, xi) / (length_xi * length_xip1);

      // calculate components of gradients
      float s = delta_ang / (length_delta_xi * length_delta_xi);
      Coordinate2d ones{1, 1};
      pc_i = (u * (-p1 - p2) - (s * ones));
      pc_im1 = (u * p2 - (s * ones));
      pc_ip1 = (u * p1); // (kappa - kappa_max_) *
    } else {
      pc_i << 0, 0;
      pc_im1 << 0, 0;
      pc_ip1 << 0, 0;
    }
  } else {
    pc_i << 0, 0;
    pc_im1 << 0, 0;
    pc_ip1 << 0, 0;
  }
}
