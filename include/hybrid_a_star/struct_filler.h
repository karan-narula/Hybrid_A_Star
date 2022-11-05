#ifndef STRUCT_FILLER
#define STRUCT_FILLER

#include "parameters.h"

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

namespace hybrid_a_star {
namespace parameters {
std::ostream &
operator<<(std::ostream &os,
           const hybrid_a_star::parameters::penalties &penalty_params) {
  os << penalty_params.change_dir << " " << penalty_params.change_head << " "
     << penalty_params.change_steering_index << " " << penalty_params.reversing
     << " " << penalty_params.change_velocity << " "
     << penalty_params.change_steering << " ";
  return os;
}

std::ostream &
operator<<(std::ostream &os,
           const hybrid_a_star::parameters::stop_explore &stop_params) {
  os << stop_params.heading_threshold_error << " "
     << stop_params.distance_threshold_error << " ";
  return os;
}

std::ostream &
operator<<(std::ostream &os,
           const hybrid_a_star::parameters::vehicle &vehicle_params) {
  os << vehicle_params.length << " " << vehicle_params.width << " "
     << vehicle_params.wheelbase << " " << vehicle_params.baselink_to_front
     << " ";
  return os;
}

std::ostream &
operator<<(std::ostream &os,
           const hybrid_a_star::parameters::vehicle_dyn_constraints
               &vehicle_dyn_constraints) {
  os << vehicle_dyn_constraints.max_accel << " "
     << vehicle_dyn_constraints.max_steering_vel << " ";
  return os;
}

void validate(boost::any &v, const std::vector<std::string> &values,
              hybrid_a_star::parameters::penalties *, float) {
  po::validators::check_first_occurrence(v);
  // if number of strings is not equal to six, throw an error
  if (values.size() != 6)
    throw po::too_many_positional_options_error();

  v = boost::any(hybrid_a_star::parameters::penalties{
      boost::lexical_cast<float>(values[0]),
      boost::lexical_cast<float>(values[1]),
      boost::lexical_cast<float>(values[2]),
      boost::lexical_cast<float>(values[3]),
      boost::lexical_cast<float>(values[4]),
      boost::lexical_cast<float>(values[5])});
}

void validate(boost::any &v, const std::vector<std::string> &values,
              hybrid_a_star::parameters::stop_explore *, float) {
  po::validators::check_first_occurrence(v);
  // if number of strings is not equal to two, throw an error
  if (values.size() != 2)
    throw po::too_many_positional_options_error();

  v = boost::any(hybrid_a_star::parameters::stop_explore{
      boost::lexical_cast<float>(values[0]),
      boost::lexical_cast<float>(values[1])});
}

void validate(boost::any &v, const std::vector<std::string> &values,
              hybrid_a_star::parameters::vehicle *, float) {
  po::validators::check_first_occurrence(v);
  // if number of strings is not equal to three, throw an error
  if (values.size() != 3)
    throw po::too_many_positional_options_error();

  v = boost::any(hybrid_a_star::parameters::vehicle{
      boost::lexical_cast<float>(values[0]),
      boost::lexical_cast<float>(values[1]),
      boost::lexical_cast<float>(values[2])});
}

void validate(boost::any &v, const std::vector<std::string> &values,
              hybrid_a_star::parameters::vehicle_dyn_constraints *, float) {
  po::validators::check_first_occurrence(v);
  // if number of strings is not equal to two, throw an error
  if (values.size() != 2)
    throw po::too_many_positional_options_error();

  v = boost::any(hybrid_a_star::parameters::vehicle_dyn_constraints{
      boost::lexical_cast<float>(values[0]),
      boost::lexical_cast<float>(values[1])});
}
} // namespace parameters

} // namespace hybrid_a_star

#endif
