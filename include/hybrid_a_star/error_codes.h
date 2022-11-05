#ifndef ERROR_CODES
#define ERROR_CODES

#include <mbf_msgs/GetPathAction.h>
#include <stdint.h>
/*
 * \file error_codes.h
 * \brief supply list of codes to be used with A_Star class. Currently, the
 * codes are tied to the mbf architecture. However, these can be overriden if
 * the user wanted to use A_Star outside of ROS
 */

namespace hybrid_a_star {

namespace GetPathResult {
static const uint32_t SUCCESS = ::mbf_msgs::GetPathResult::SUCCESS;

static const uint32_t FAILURE =
    ::mbf_msgs::GetPathResult::FAILURE; // Unspecified failure, only used for
                                        // old, non-mfb_core based plugins
static const uint32_t CANCELED =
    ::mbf_msgs::GetPathResult::CANCELED; // The action has been canceled by a
                                         // action client
static const uint32_t INVALID_START = ::mbf_msgs::GetPathResult::INVALID_START;
static const uint32_t INVALID_GOAL = ::mbf_msgs::GetPathResult::INVALID_GOAL;
static const uint32_t NO_PATH_FOUND = ::mbf_msgs::GetPathResult::NO_PATH_FOUND;
static const uint32_t PAT_EXCEEDED = ::mbf_msgs::GetPathResult::PAT_EXCEEDED;
static const uint32_t EMPTY_PATH = ::mbf_msgs::GetPathResult::EMPTY_PATH;
static const uint32_t TF_ERROR = ::mbf_msgs::GetPathResult::TF_ERROR;
static const uint32_t NOT_INITIALIZED =
    ::mbf_msgs::GetPathResult::NOT_INITIALIZED;
static const uint32_t INVALID_PLUGIN =
    ::mbf_msgs::GetPathResult::INVALID_PLUGIN;
static const uint32_t INTERNAL_ERROR =
    ::mbf_msgs::GetPathResult::INTERNAL_ERROR;
static const uint32_t STOPPED =
    ::mbf_msgs::GetPathResult::STOPPED; // The planner execution has been
                                        // stopped rigorously, e.g. Move Base
                                        // Flex has been shut down.

// function to intrepret the the above code into string
inline std::string error_code_to_string(const uint32_t &error_code) {
  std::string error_name;

  switch (error_code) {

  case SUCCESS:
    error_name = "Success";
    break;

  case FAILURE:
    error_name = "Failure";
    break;

  case CANCELED:
    error_name = "Cancelled";
    break;

  case INVALID_START:
    error_name = "Invalid Start";
    break;

  case INVALID_GOAL:
    error_name = "Invalid Goal";
    break;

  case NO_PATH_FOUND:
    error_name = "No Path found";
    break;

  case PAT_EXCEEDED:
    error_name = "No Path found within limited time/expansions";
    break;

  case EMPTY_PATH:
    error_name = "Empty Path";
    break;

  case TF_ERROR:
    error_name = "TF error";
    break;

  case NOT_INITIALIZED:
    error_name = "Not Initialised";
    break;

  case INVALID_PLUGIN:
    error_name = "Invalid Plugin";
    break;

  case INTERNAL_ERROR:
    error_name = "Internal error";
    break;

  case STOPPED:
    error_name = "Stopped";
    break;

  default:
    error_name = "Invalid error token";
  }

  return error_name;
}

} // namespace GetPathResult
} // namespace hybrid_a_star

#endif
