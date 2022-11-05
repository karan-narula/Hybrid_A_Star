#include "hybrid_a_star/parameters.h"

// function to return vehicle motion model for 2D cell-wise navigation
hybrid_a_star::parameters::vehicle_motion
hybrid_a_star::parameters::getPrecomputeVM(
    const hybrid_a_star::parameters::graph &graph_param,
    const hybrid_a_star::parameters::vehicle &v) {
  // construct vector of steering actions based on resolution alone
  float diag_length = std::sqrt(std::pow(graph_param.resolution[0], 2) +
                                std::pow(graph_param.resolution[1], 2));
  std::vector<float> steering_actions{graph_param.resolution[0],
                                      graph_param.resolution[1], diag_length,
                                      diag_length};
  return hybrid_a_star::parameters::vehicle_motion(true, steering_actions,
                                                   graph_param, 1.0, v);
}
void hybrid_a_star::parameters::getPrecomputeVM(
    const hybrid_a_star::parameters::graph &graph_param,
    const hybrid_a_star::parameters::vehicle &v,
    hybrid_a_star::parameters::vehicle_motion &vm) {
  vm.num_actions = 4;
  vm.allow_reverse = true;
  vm.set_values(graph_param, 1.0);

  vm.steering_actions.clear();
  vm.steering_actions.push_back(graph_param.resolution[0]);
  vm.steering_actions.push_back(graph_param.resolution[1]);
  vm.steering_actions.push_back(vm.chord_l_min);
  vm.steering_actions.push_back(vm.chord_l_min);

  // not necessary
  vm.set_min_r(v);
}

// function to return graph parameters based on the yaml file
hybrid_a_star::parameters::graph hybrid_a_star::parameters::getGraphParamYaml(
    const std::string &filename, const std::string &scenario,
    const std::string &default_scenario, int &nrows, int &ncols) {
  // load the .yaml file based on the filename
  YAML::Node node = YAML::LoadFile(filename);

  // check if the default scenario atleast exists
  assert(node[default_scenario]);

  // check if scenario name exists
  assert(node[scenario]);

  parameters::graph new_graph_param;

  // load resolution field
  std::string use_key =
      node[scenario]["resolution"] ? scenario : default_scenario;
  new_graph_param.resolution[0] = node[use_key]["resolution"][0].as<float>();
  new_graph_param.resolution[1] = node[use_key]["resolution"][1].as<float>();
  // heading resolution is given in degrees
  if (node[use_key]["resolution"].size() > 2) {
    new_graph_param.resolution[2] =
        (M_PI / 180) * node[use_key]["resolution"][2].as<float>();
  }

  // load origin field
  use_key = node[scenario]["origin"] ? scenario : default_scenario;
  new_graph_param.origin[0] = node[use_key]["origin"][0].as<float>();
  new_graph_param.origin[1] = node[use_key]["origin"][1].as<float>();

  // load length, number of rows and columns
  use_key = node[scenario]["size"] ? scenario : default_scenario;
  nrows = node[use_key]["size"][0].as<int>();
  ncols = node[use_key]["size"][1].as<int>();
  new_graph_param.height = new_graph_param.resolution[0] * nrows;
  new_graph_param.width = new_graph_param.resolution[1] * ncols;

  return new_graph_param;
}
