// include project's header files
#include "hybrid_a_star/a_star.h"
#include "hybrid_a_star/grid_map_wrapper3d.h"
#include "hybrid_a_star/smoother.h"
#include "hybrid_a_star/struct_filler.h"

// include other standard header files
#include "grid_map_ros/grid_map_ros.hpp"
#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/Marker.h>

using namespace grid_map;

// forward declarations
bool getParameters(
    int argc, char **argv, ros::NodeHandle &n, std::string &scenario,
    std::string &filename, std::vector<float> &steering_actions,
    hybrid_a_star::parameters::planner_intrinsics &intrinsics,
    hybrid_a_star::parameters::penalties &penalty_params,
    hybrid_a_star::parameters::stop_explore &stop_params,
    hybrid_a_star::parameters::vehicle &vehicle_params,
    hybrid_a_star::parameters::vehicle_dyn_constraints &vehicle_dyn_constraints,
    float &time_incr, bool &allow_reverse, float (&headings)[2],
    bool &default_heading, float &dvwnf, Smoother &smoother,
    int &max_oneshot_interval);

void publishMessages(ros::NodeHandle &n,
                     hybrid_a_star::path_planning::A_Star<GridMap> &planner,
                     std::vector<std::vector<hybrid_a_star::rob_pos> *> &paths,
                     std::vector<hybrid_a_star::Grid_Map_Wrapper *> &gmws,
                     const std::vector<std::string> &path_types,
                     const float (*color_array)[3]);

// function to plan and print out statistics
template <typename cost_map_type>
std::vector<hybrid_a_star::rob_pos>
planAndPrint(hybrid_a_star::path_planning::A_Star<cost_map_type> &planner,
             hybrid_a_star::Grid_Map_Wrapper &gmw,
             const hybrid_a_star::rob_pos &start_pos,
             const hybrid_a_star::rob_pos &goal_pos,
             const std::string method_name, const std::string &filename,
             const std::string &scenario, const bool &show_statistics = true,
             const bool &show_graph = true) {
  planner.setStartGoal(start_pos, goal_pos);
  planner.search();
  planner.produce_path();
  std::vector<hybrid_a_star::rob_pos> path = planner.getPath();
  printf("Path from %s has %sbeen found with %u iterations and %d expansions. "
         "Path's size: %zu. Time taken to find path is %d milliseconds. Error "
         "code is %u: \n",
         method_name.c_str(), planner.isPathFound() ? "" : "not ",
         planner.getNumIter(), planner.getNumExpand(), path.size(),
         planner.getSearchTime(), planner.getErrorCode());

  // visualise path via populating the grid with different number and printing
  // it out at the end
  if (show_graph && planner.isPathFound()) {
    std::cout << std::endl;
    auto it = path.rbegin();
    int i;
    for (i = 1; it != path.rend(); ++it, i++) {
      // printf("Position is (%f, %f, %f)\n", it->x, it->y, it->theta);
      gmw.setValue(*it, -1 * i);
    }
    gmw.print();
    std::cout << std::endl << std::endl;
    // reload costmap
    gmw.loadCostmap(filename, scenario);
  }

  return path;
}

int main(int argc, char **argv) {
  // create variables to hold and read parameters specified by the user
  std::vector<float> steering_actions; // store the steering actions
  std::string scenario;                // store the scenario name
  std::string filename; // filename of the .yaml file containing configuration
                        // of scenario
  hybrid_a_star::parameters::planner_intrinsics
      intrinsics; // for chord length, etc
  hybrid_a_star::parameters::penalties
      penalty_params; // penalties for the global travel cost
  hybrid_a_star::parameters::stop_explore
      stop_params; // thresholds dictating when the goal has been reached
  hybrid_a_star::parameters::vehicle
      vehicle_params; // parameter describing shape of the vehicle
  hybrid_a_star::parameters::vehicle_dyn_constraints
      vehicle_dyn_constraints; // paramaeters describing motion constraints
  float time_incr;             // time increment during vehicle motion
  bool allow_reverse;          // allow the car to go backwards
  float headings[2]; // to hold the heading of the starting and ending position
  bool default_heading; // if heading is not specified, default must be read
                        // from .yaml file
  float dvwnf;          // default value when not found
  Smoother smoother;
  int max_oneshot_interval = 200;
  // initialise, node, node handle and rate
  int temp = 0;
  ros::init(temp, NULL, "hybrid_a_star_planner");
  ros::NodeHandle n;

  // retrieve arguments from user input
  bool flag_return =
      getParameters(argc, argv, n, scenario, filename, steering_actions,
                    intrinsics, penalty_params, stop_params, vehicle_params,
                    vehicle_dyn_constraints, time_incr, allow_reverse, headings,
                    default_heading, dvwnf, smoother, max_oneshot_interval);
  if (flag_return)
    return 1;

  // create graph
  int nrows, ncols;
  hybrid_a_star::parameters::graph graph_param =
      hybrid_a_star::parameters::getGraphParamYaml(
          filename, scenario, hybrid_a_star::parameters::default_scenario,
          nrows, ncols);
  dvwnf *= sqrt(pow(graph_param.resolution[0], 2) +
                pow(graph_param.resolution[1], 2));

  // create motion model for vanilla a-star and hybrid a-star
  hybrid_a_star::parameters::vehicle_motion vm_vani =
      hybrid_a_star::parameters::getPrecomputeVM(graph_param, vehicle_params);
  hybrid_a_star::parameters::vehicle_motion vm_hybrid(
      allow_reverse, steering_actions, graph_param,
      intrinsics.default_chord_l_factor, vehicle_params,
      vehicle_dyn_constraints, time_incr);

  // create graph to perform search for vanilla a-star and hybrid a-star
  std::string static_obs_layer("OG");
  std::vector<std::string> layer_names{static_obs_layer};
  std::vector<float> times{0.};

  hybrid_a_star::Grid_Map_Wrapper gmw_twoD(
      graph_param, vehicle_params, vm_vani,
      hybrid_a_star::parameters::two_d_pre_model, layer_names, times,
      static_obs_layer);
  hybrid_a_star::Grid_Map_Wrapper3D gmw_hybrid(
      graph_param, vehicle_params, vm_hybrid,
      &hybrid_a_star::kinematic_models::non_holo_car_steer_dyn_fixed_dt,
      layer_names, times, static_obs_layer);
  gmw_twoD.loadCostmap(filename, scenario);
  gmw_hybrid.loadCostmap(filename, scenario);
  // get costmap and try to get the layer name
  // GridMap& cost_map = gmw_twoD.getCostmap();
  // const std::vector<std::string>& layer_names = cost_map.getLayers();

  // get the starting and goal pose
  hybrid_a_star::rob_pos start_pos, goal_pos;
  bool success;
  hybrid_a_star::getStartGoalYaml(gmw_hybrid, start_pos, goal_pos, filename,
                                  scenario, success);
  if (!default_heading) {
    start_pos.theta = headings[0];
    goal_pos.theta = headings[1];
  }

  // create vanilla a-star planner
  hybrid_a_star::stopping_crit::gen_stop_func<GridMap> twoD_stop =
      &hybrid_a_star::stopping_crit::xyInd<GridMap>;
  std::vector<hybrid_a_star::heuristics::gen_heuristic_func<GridMap>>
      a_star_heuristic{hybrid_a_star::parameters::vani_heuristic<GridMap>};
  hybrid_a_star::global_cost::gen_globalcost_func twoD_cost_func =
      &hybrid_a_star::global_cost::raw_distance;
  hybrid_a_star::path_planning::A_Star<GridMap> vani_planner(
      gmw_twoD, a_star_heuristic, twoD_stop, twoD_cost_func, penalty_params,
      stop_params, false, intrinsics.max_num_iterations);

  // create dijsktra planner
  std::vector<hybrid_a_star::heuristics::gen_heuristic_func<GridMap>>
      dij_heuristic{hybrid_a_star::parameters::dijsktra_heuristic<GridMap>};
  hybrid_a_star::stopping_crit::gen_stop_func<GridMap> no_stop =
      &hybrid_a_star::stopping_crit::no_stop<GridMap>;
  hybrid_a_star::path_planning::A_Star<GridMap> dij_pre_planner(
      gmw_twoD, dij_heuristic, no_stop, twoD_cost_func, penalty_params,
      stop_params, false, intrinsics.max_num_iterations);
  hybrid_a_star::path_planning::A_Star<GridMap> dij_planner(
      gmw_twoD, dij_heuristic, twoD_stop, twoD_cost_func, penalty_params,
      stop_params, false, intrinsics.max_num_iterations);

  // create hybrid a-star planner
  hybrid_a_star::stopping_crit::gen_stop_func<GridMap> hybrid_stop =
      &hybrid_a_star::stopping_crit::xyInd_headRaw<GridMap>;
  hybrid_a_star::global_cost::gen_globalcost_func hybrid_cost_func =
      &hybrid_a_star::global_cost::chord_length;
  // hybrid_a_star::global_cost::gen_globalcost_func hybrid_cost_func =
  // &hybrid_a_star::global_cost::penal_steer_reverse_add;
  std::vector<hybrid_a_star::heuristics::gen_heuristic_func<GridMap>>
      hybrid_heuristics{hybrid_a_star::parameters::vani_heuristic<GridMap>};
  if (allow_reverse) {
    hybrid_heuristics.push_back(
        &hybrid_a_star::heuristics::reedsheeps_length<GridMap>);
  } else {
    hybrid_heuristics.push_back(
        &hybrid_a_star::heuristics::dubins_length<GridMap>);
  }
  hybrid_a_star::path_planning::A_Star<GridMap> hybrid_planner(
      gmw_hybrid, hybrid_heuristics, hybrid_stop, hybrid_cost_func,
      penalty_params, stop_params, intrinsics.precompute_flag,
      intrinsics.oneshot_flag, intrinsics.max_num_iterations);
  hybrid_planner.setOneshotInterval(max_oneshot_interval);
  // check if user wanted precomputation
  std::vector<hybrid_a_star::path_planning::A_Star<GridMap> *> precomp_planners{
      &dij_pre_planner};
  // std::vector<hybrid_a_star::path_planning::A_Star<GridMap>*>
  // precomp_planners{&vani_planner};
  std::vector<
      hybrid_a_star::precompute::gen_default_func_when_not_found<GridMap>>
      funcs_when_not_found{hybrid_a_star::precompute::default_return<GridMap>};
  if (intrinsics.precompute_flag) {
    // dij_planner.setStartGoal(goal_pos, start_pos);
    // vani_planner.setStartGoal(goal_pos, start_pos);
    hybrid_planner.setPrecompPlanners(&precomp_planners, &funcs_when_not_found,
                                      dvwnf);
  }

  // plan and retrieve paths
  auto hybrid_path =
      planAndPrint<GridMap>(hybrid_planner, gmw_hybrid, start_pos, goal_pos,
                            "hybrid A-star", filename, scenario, true, false);
  // checking if nodes were re-expanded
  bool reexpanded;
  auto reexpanded_counter = path_planning::nodesReexpanded(
      hybrid_planner.getExpandedCounter(), reexpanded);
  printf("Nodes were %sreexpanded\n", reexpanded ? "" : "not ");
  // what happens if the global cost function changed, would the path get
  // altered as well?
  // hybrid_cost_func = &hybrid_a_star::global_cost::penal_headRaw_add;
  // hybrid_path =
  //    planAndPrint<GridMap>(hybrid_planner, gmw_hybrid, start_pos, goal_pos,
  //                          "hybrid A-star", filename, scenario, true, false);
  auto vani_path =
      planAndPrint<GridMap>(vani_planner, gmw_twoD, start_pos, goal_pos,
                            "vanilla A-star", filename, scenario, true, false);
  auto dij_path =
      planAndPrint<GridMap>(dij_planner, gmw_twoD, start_pos, goal_pos,
                            "Dijsktra", filename, scenario, true, false);
  // get number of oneshot points in the path
  int num_oneshot_points = hybrid_planner.getNumOneshotPts();
  // get smoothed path of hybrid a star
  auto smooth_timer_start = std::chrono::steady_clock::now();
  float obs_time, curv_time, smooth_time;
  auto &smoothed_hybrid_path =
      smoother.smoothPath(hybrid_path, gmw_hybrid, num_oneshot_points, false);
  std::cout << "Time taken to smooth the path is: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - smooth_timer_start)
                   .count()
            << " milliseconds\n";
  smoother.getTimes(obs_time, curv_time, smooth_time);
  printf("Time taken for optimising the obstacle component: %f milliseconds "
         "while the curvature component is: %f millseconds while the "
         "smoothness component is: %f milliseconds\n",
         obs_time, curv_time, smooth_time);

  // publish messages to ROS to visualise in RVIZ
  std::vector<std::vector<hybrid_a_star::rob_pos> *> paths{
      &hybrid_path, &vani_path, &dij_path,
      &smoothed_hybrid_path}; // for visualising paths
  std::vector<hybrid_a_star::Grid_Map_Wrapper *> gmws{
      &gmw_twoD, &gmw_hybrid}; // for visualising the grids
  std::vector<std::string> path_types{"hybrid_a_star", "vani_a_star",
                                      "dijsktra",
                                      "smoothed_hybrid"}; // for topic names
  float color_array[4][3];
  // fill in color based on parameter server
  n.param<float>("/test_bed/visualisation/lines/hybrid_a_star/r",
                 color_array[0][0], 0.0);
  n.param<float>("/test_bed/visualisation/lines/hybrid_a_star/g",
                 color_array[0][1], 0.0);
  n.param<float>("/test_bed/visualisation/lines/hybrid_a_star/b",
                 color_array[0][2], 1.0);
  n.param<float>("/test_bed/visualisation/lines/vani_a_star/r",
                 color_array[1][0], 0.0);
  n.param<float>("/test_bed/visualisation/lines/vani_a_star/g",
                 color_array[1][1], 1.0);
  n.param<float>("/test_bed/visualisation/lines/vani_a_star/b",
                 color_array[1][2], 0.0);
  n.param<float>("/test_bed/visualisation/lines/dijsktra/r", color_array[2][0],
                 0.5);
  n.param<float>("/test_bed/visualisation/lines/dijsktra/g", color_array[2][1],
                 0.0);
  n.param<float>("/test_bed/visualisation/lines/dijsktra/b", color_array[2][2],
                 0.5);
  n.param<float>("/test_bed/visualisation/lines/smooth_hybrid/r",
                 color_array[3][0], 244.0 / 255.0);
  n.param<float>("/test_bed/visualisation/lines/smooth_hybrid/g",
                 color_array[3][1], 164.0 / 255.0);
  n.param<float>("/test_bed/visualisation/lines/smooth_hybrid/b",
                 color_array[3][2], 96.0 / 255.0);
  publishMessages(n, hybrid_planner, paths, gmws, path_types, color_array);

  return 0;
}

// function to publish messages to ROS for visualisation via RVIZ
void publishMessages(ros::NodeHandle &n,
                     hybrid_a_star::path_planning::A_Star<GridMap> &planner,
                     std::vector<std::vector<hybrid_a_star::rob_pos> *> &paths,
                     std::vector<hybrid_a_star::Grid_Map_Wrapper *> &gmws,
                     const std::vector<std::string> &path_types,
                     const float (*color_array)[3]) {
  // make sure that the size is the same between the paths and path_types
  assert(paths.size() == path_types.size());

  // frequency of publishing
  ros::Rate r(1);

  // fixed frame id for all messages
  std::string frame_id("map");

  // create publisher for paths
  ros::Publisher pub_paths[path_types.size()];
  ros::Publisher vis_paths[path_types.size()];
  for (std::size_t i = 0; i < path_types.size(); ++i) {
    // create publishers for paths (so that python subscriber can read)
    pub_paths[i] = n.advertise<geometry_msgs::PoseArray>(
        "pose_array_" + path_types[i], 100);
    // create visualisation of the path for rviz (line)
    vis_paths[i] =
        n.advertise<visualization_msgs::Marker>("line_" + path_types[i], 100);
  }

  // create publisher for starting and goal pose arrows
  ros::Publisher pub_nodes =
      n.advertise<visualization_msgs::Marker>("nodes", 100);

  // create publisher for grid
  ros::Publisher pub_grid =
      n.advertise<grid_map_msgs::GridMap>("grid_map", 10, true);

  // get the costmap from the graph structure
  GridMap &map = gmws[0]->getCostmap();
  map.setFrameId(frame_id);

  // get value of visualisation parameters from the parameter server if it
  // exists
  float point_width, point_height, point_color_r, point_color_g, point_color_b;
  n.param<float>("/test_bed/visualisation/points/width", point_width, 0.4);
  n.param<float>("/test_bed/visualisation/points/height", point_height, 0.4);
  n.param<float>("/test_bed/visualisation/points/r", point_color_r, 1.0f);
  n.param<float>("/test_bed/visualisation/points/g", point_color_g, 0.0f);
  n.param<float>("/test_bed/visualisation/points/b", point_color_b, 0.0f);

  float arrow_scale_x, arrow_scale_y, arrow_scale_z, arrow_color_r,
      arrow_color_b, arrow_color_g;
  n.param<float>("/test_bed/visualisation/arrows/scale_x", arrow_scale_x, 3.0);
  n.param<float>("/test_bed/visualisation/arrows/scale_y", arrow_scale_y, 1.0);
  n.param<float>("/test_bed/visualisation/arrows/scale_z", arrow_scale_z, 1.0);
  n.param<float>("/test_bed/visualisation/arrows/r", arrow_color_r, 0.0f);
  n.param<float>("/test_bed/visualisation/arrows/g", arrow_color_g, 0.0f);
  n.param<float>("/test_bed/visualisation/arrows/b", arrow_color_b, 0.0f);

  float line_width;
  n.param<float>("/test_bed/visualisation/lines/line_width", line_width, 0.0f);

  // create visualisation message of for different paths
  visualization_msgs::Marker points[path_types.size()];
  visualization_msgs::Marker line_strip[path_types.size()];
  for (std::size_t i = 0; i < path_types.size(); ++i) {
    // default configurations
    points[i].header.frame_id = line_strip[i].header.frame_id = frame_id;
    points[i].ns = line_strip[i].ns = "paths_" + path_types[i];
    points[i].id = 0;
    line_strip[i].id = 1;

    points[i].type = visualization_msgs::Marker::POINTS;
    line_strip[i].type = visualization_msgs::Marker::LINE_STRIP;

    points[i].action = line_strip[i].action = visualization_msgs::Marker::ADD;

    points[i].pose.orientation.w = line_strip[i].pose.orientation.w = 1.0;

    // width and height of points
    points[i].scale.x = point_width;
    points[i].scale.y = point_height;

    // linewidth of the line connecting the points
    line_strip[i].scale.x = line_width;

    // points are red while path is dependent on array of colors
    points[i].color.r = point_color_r;
    points[i].color.g = point_color_g;
    points[i].color.b = point_color_b;
    points[i].color.a = 1.0;
    line_strip[i].color.r = *(*(color_array + i) + 0);
    line_strip[i].color.g = *(*(color_array + i) + 1);
    line_strip[i].color.b = *(*(color_array + i) + 2);
    line_strip[i].color.a = 1.0;

    // the vertices of the points and lines are from the path itself
    std::vector<hybrid_a_star::rob_pos> &current_path = *paths[i];
    for (auto it = current_path.begin(); it != current_path.end(); ++it) {
      geometry_msgs::Point p;
      p.x = it->x;
      p.y = it->y;
      p.z = 0;

      points[i].points.push_back(p);
      line_strip[i].points.push_back(p);
    }
  }

  // create visualisation msg for starting and goal node
  visualization_msgs::Marker start_arrow, goal_arrow;
  {
    // fill in default properties: frame, namespace, id, shape type and action
    start_arrow.header.frame_id = goal_arrow.header.frame_id = frame_id;
    start_arrow.ns = goal_arrow.ns = "end_nodes";
    start_arrow.id = 0;
    goal_arrow.id = 1;
    start_arrow.type = goal_arrow.type = visualization_msgs::Marker::ARROW;
    start_arrow.action = goal_arrow.action = visualization_msgs::Marker::ADD;

    // fill in scale of the marker (1x1x1 means 1 metre on each side)
    start_arrow.scale.x = goal_arrow.scale.x = arrow_scale_x;
    start_arrow.scale.y = goal_arrow.scale.y = arrow_scale_y;
    start_arrow.scale.z = goal_arrow.scale.z = arrow_scale_z;

    // fill in color, black
    start_arrow.color.r = goal_arrow.color.r = arrow_color_r;
    start_arrow.color.g = goal_arrow.color.g = arrow_color_g;
    start_arrow.color.b = goal_arrow.color.b = arrow_color_b;
    start_arrow.color.a = goal_arrow.color.a = 1.0;

    // never auto delete marker
    // start_arrow.lifetime = goal_arrow.lifetime = ros::Duration();

    // set the position of the markers based on start and goal pose (pick from
    // one planner)
    const hybrid_a_star::rob_pos &start_pos = planner.getStartPos();
    const hybrid_a_star::rob_pos &goal_pos = planner.getGoalPos();
    start_arrow.pose.position.x = start_pos.x;
    start_arrow.pose.position.y = start_pos.y;
    start_arrow.pose.position.z = 0;
    start_arrow.pose.orientation =
        tf::createQuaternionMsgFromYaw(start_pos.theta);
    goal_arrow.pose.position.x = goal_pos.x;
    goal_arrow.pose.position.y = goal_pos.y;
    goal_arrow.pose.position.z = 0;
    goal_arrow.pose.orientation =
        tf::createQuaternionMsgFromYaw(goal_pos.theta);
  }

  // keep publishing it over and over
  int num_messages_pub = 0;
  while (ros::ok()) {
    // get current time
    ros::Time time = ros::Time::now();

    // publish the paths
    for (std::size_t i = 0; i < path_types.size(); ++i) {
      points[i].header.stamp = line_strip[i].header.stamp = time;
      vis_paths[i].publish(points[i]);
      vis_paths[i].publish(line_strip[i]);
    }

    // publish the starting and goal arrows
    start_arrow.header.stamp = goal_arrow.header.stamp = time;
    pub_nodes.publish(start_arrow);
    pub_nodes.publish(goal_arrow);
    // alternatively, use the same publisher but different namspace
    // vis_paths[0].publish(start_arrow);
    // vis_paths[0].publish(goal_arrow);

    // publish the gridmap
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    pub_grid.publish(message);

    // increment and output to screen
    num_messages_pub++;
    // ROS_INFO("Publishing the same message for the %dth time",
    // num_messages_pub);

    // sleep
    r.sleep();
  }
}

// function to get parameters for the test bed from command line
bool getParameters(
    int argc, char **argv, ros::NodeHandle &n, std::string &scenario,
    std::string &filename, std::vector<float> &steering_actions,
    hybrid_a_star::parameters::planner_intrinsics &intrinsics,
    hybrid_a_star::parameters::penalties &penalty_params,
    hybrid_a_star::parameters::stop_explore &stop_params,
    hybrid_a_star::parameters::vehicle &vehicle_params,
    hybrid_a_star::parameters::vehicle_dyn_constraints &vehicle_dyn_constraints,
    float &time_incr, bool &allow_reverse, float (&headings)[2],
    bool &default_heading, float &dvwnf, Smoother &smoother,
    int &max_oneshot_interval) {
  // quickly convert radians to degrees first
  stop_params.heading_threshold_error *= 180 / M_PI;
  vehicle_dyn_constraints.max_steering_vel *= 180 / M_PI;

  // temporary store the heading parameters (given in degrees)
  std::vector<float> temp_headings{0, 0};

  // temporary variable to store if user prefer use of command line or through
  // yaml file
  bool command_line_flag;

  // temporary variables to store the parameters required for path smoother
  unsigned int num_iterations;
  float min_turn_r, step_size, max_obs_dist;
  std::vector<float> weights{0, 0, 0};

  po::options_description desc("Allowed options");
  desc.add_options()("help,H", "produce list of options and usage details")(
      "command_line", po::value<bool>(&command_line_flag)->default_value(true),
      "Load parameters via commandline or through .yaml file and parameter "
      "server (configured in launch file). Defaults to true (prefer "
      "commandline)")(
      "scenario,S",
      po::value<std::string>(&scenario)->default_value(
          hybrid_a_star::parameters::default_scenario),
      "Give the name of the scenario being simulated. It must correspond to "
      "one of the options in .yaml file. Default to `default`")(
      "dvwnf", po::value<float>(&dvwnf)->default_value(dvwnf),
      "The default value to take if a cell was not evaluated during the "
      "precomputation")(
      "steering_actions,A",
      po::value<std::vector<float>>(&steering_actions)
          ->multitoken()
          ->default_value(std::vector<float>{-10, 0, 10}, "-10, 0, 10"),
      "List of length n containing steering actions in degrees for hybrid "
      "a-star. Default is [-10, 0, 10]")(
      "config_file,F",
      po::value<std::string>(&filename)->default_value(std::string(
          "./src/zio_navigation/hybrid_a_star/scripts/scenarios.yaml")),
      "Give name of .yaml file to look up scenario and simulate algorithms on. "
      "Default is `scenarios.yaml` in scripts folder based on absolute path on "
      "local machine")(
      "penalties,P",
      po::value<hybrid_a_star::parameters::penalties>(&penalty_params)
          ->multitoken()
          ->default_value(penalty_params),
      "Six penalties that may be of use in the hybrid a-star algorithm. The "
      "unit is in number of diagonals of the cells. The penalties are (in "
      "order) for change in direction, change in heading, change in steering "
      "index, reversing, change in velocity and change in steering angle.")(
      "thresholds,T",
      po::value<hybrid_a_star::parameters::stop_explore>(&stop_params)
          ->multitoken()
          ->default_value(stop_params),
      "Two thresholds for comparing the current node against the goal such "
      "that if within threshold, the goal is said to have been reached. These "
      "are (in order) heading threshold (in degrees) and distance threshold "
      "(in metres)")(
      "vehicle_params,V",
      po::value<hybrid_a_star::parameters::vehicle>(&vehicle_params)
          ->multitoken()
          ->default_value(vehicle_params),
      "Dimension of the vehicle: length and width in metres")(
      "vehicle_dyn_constraint,D",
      po::value<hybrid_a_star::parameters::vehicle_dyn_constraints>(
          &vehicle_dyn_constraints)
          ->multitoken()
          ->default_value(vehicle_dyn_constraints),
      "Dynamic constraint of vehicle motion: acceleration and steering "
      "velocity in m/s^2 and degrees/sec")(
      "time_incr", po::value<float>(&time_incr)->default_value(0.0),
      "Time increment during vehicle motion. Default to 0.0")(
      "chord_factor,C",
      po::value<float>(&intrinsics.default_chord_l_factor)
          ->default_value(intrinsics.default_chord_l_factor),
      "Factor to scale the default chord length whose value is the diagonal of "
      "the cell. Default to 1.0.")(
      "precompute",
      po::value<bool>(&intrinsics.precompute_flag)
          ->default_value(intrinsics.precompute_flag),
      "Allow precomputation with standard 2D a-star/dijstkra to speed up the "
      "search? Default to true.")(
      "oneshot",
      po::value<bool>(&intrinsics.oneshot_flag)
          ->default_value(intrinsics.oneshot_flag),
      "Enable attempt of getting one-shot path to the goal at low frequency? "
      "Default to false.")("max_oneshot_interval",
                           po::value<int>(&max_oneshot_interval)
                               ->default_value(max_oneshot_interval),
                           "Maximum number of expansions before performing "
                           "analytical expansion/oneshot. Default to 200.")(
      "allow_reverse,R", po::value<bool>(&allow_reverse)->default_value(false),
      "Allow the vehicle to move backwards? Default is false.")(
      "max_iter,I",
      po::value<unsigned int>(&intrinsics.max_num_iterations)
          ->default_value(intrinsics.max_num_iterations),
      "Max number of iterations to search. Default to 1e5.")(
      "headings", po::value<std::vector<float>>(&temp_headings)->multitoken(),
      "List of length two containing the start and ending nodes' headings in "
      "degrees. Default will be scenario-specific.")(
      "num_opt_steps",
      po::value<unsigned int>(&num_iterations)->default_value(100),
      "Number of optimisation step for smoothing the path. Defaults too 100")(
      "min_turn_r", po::value<float>(&min_turn_r)->default_value(1),
      "Minimum turning radius of the vehicle. Defaults to one.")(
      "step_size", po::value<float>(&step_size)->default_value(0.1),
      "Step size for gradient descent (some times referred to as learning "
      "rate). Defaults to 0.1.")(
      "max_obs_dist", po::value<float>(&max_obs_dist)->default_value(1),
      "Maximum distance for penalising obstacles when smoothing path")(
      "weights",
      po::value<std::vector<float>>(&weights)->multitoken()->default_value(
          std::vector<float>{0.2, 0.2, 0.2}, "0.2, 0.2, 0.2"),
      "List of length 3 containing weights for the optimisation given in "
      "order: obstacle, curvature and "
      "smoothness. They all default to 0.2.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc,
                                   po::command_line_style::unix_style ^
                                       po::command_line_style::allow_short),
            vm);
  po::notify(vm);

  // test it by printing out stuffs
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return true;
  }

  // if parameters were instead set via parameter server then
  if (!command_line_flag) {
    int temp;
    // get experiment's parameters
    n.getParam("/test_bed/experiment/config_file", filename);
    n.getParam("/test_bed/experiment/scenario", scenario);
    n.getParam("/test_bed/experiment/headings", temp_headings);
    // get planner's parameters
    n.getParam("/test_bed/planner/precompute", intrinsics.precompute_flag);
    n.getParam("/test_bed/planner/oneshot", intrinsics.oneshot_flag);
    n.getParam("/test_bed/planner/max_oneshot_interval", max_oneshot_interval);
    n.getParam("/test_bed/planner/allow_reverse", allow_reverse);
    n.getParam("/test_bed/planner/chord_factor",
               intrinsics.default_chord_l_factor);
    n.getParam("/test_bed/planner/time_incr", time_incr);
    n.getParam("/test_bed/planner/max_iter", temp);
    n.getParam("/test_bed/planner/dvwnf", dvwnf);
    n.getParam("/test_bed/planner/steering_actions", steering_actions);
    intrinsics.max_num_iterations = (unsigned int)temp;
    // get threshold parameters
    n.getParam("/test_bed/threshold/heading",
               stop_params.heading_threshold_error);
    n.getParam("/test_bed/threshold/distance",
               stop_params.distance_threshold_error);
    // get vehicle parameters
    n.getParam("/test_bed/vehicle/length", vehicle_params.length);
    n.getParam("/test_bed/vehicle/width", vehicle_params.width);
    n.getParam("/test_bed/vehicle/wheelbase", vehicle_params.wheelbase);
    // get penalty parameters
    n.getParam("/test_bed/penalties/change_in_dir", penalty_params.change_dir);
    n.getParam("/test_bed/penalties/change_in_head",
               penalty_params.change_head);
    n.getParam("/test_bed/penalties/change_in_steer_index",
               penalty_params.change_steering_index);
    n.getParam("/test_bed/penalties/reversing", penalty_params.reversing);
    n.getParam("/test_bed/penalties/change_in_velocity",
               penalty_params.change_velocity);
    n.getParam("/test_bed/penalties/change_in_steering",
               penalty_params.change_steering);
    // get dynamic constraint parameters
    n.getParam("/test_bed/vehicle_dyn_constraints/max_accel",
               vehicle_dyn_constraints.max_accel);
    n.getParam("/test_bed/vehicle_dyn_constraints/max_steering_vel",
               vehicle_dyn_constraints.max_steering_vel);
    // get the smoothing parameters for the path
    n.getParam("/test_bed/smoothing/num_iter", temp);
    n.getParam("/test_bed/smoothing/min_turn_r", min_turn_r);
    n.getParam("/test_bed/smoothing/step_size", step_size);
    n.getParam("/test_bed/smoothing/max_obs_dist", max_obs_dist);
    n.getParam("/test_bed/smoothing/weights", weights);
    num_iterations = (unsigned int)temp;
  }
  // test if number of arguments for headings are correct
  if (vm.count("headings")) {
    // throw error if heading vector is not of the right size
    if (temp_headings.size() != 2)
      throw po::too_many_positional_options_error();
  }

  // set the smoother parameters
  smoother.setNumIter(num_iterations);
  smoother.setMinTurnR(min_turn_r);
  smoother.setStep(step_size);
  smoother.setObsThresh(max_obs_dist);
  smoother.setWeights(weights[0], weights[1], weights[2]);
  // update part of vehicle params
  vehicle_params.updateBTF();

  // start printing the parameters that were read
  std::cout << "The following are the list of parameters loaded:\n";
  std::cout << "\tScenario chosen is: " << scenario << std::endl;
  std::cout << "\tFile name is: " << filename << std::endl;
  std::cout << "\tDefault value when not found is: " << dvwnf << std::endl;
  std::cout << "\tSteering actions picked are (in degrees): [";
  {
    auto it = steering_actions.begin();
    std::cout << *it;
    *it *= M_PI / 180;
    it++;
    for (; it != steering_actions.end(); ++it) {
      std::cout << "," << *it;
      // convert to radians
      *it *= M_PI / 180;
    }
    std::cout << "]\n";
  }

  printf("\tPrecompute flag is: %s\n",
         intrinsics.precompute_flag ? "true" : "false");
  printf("\tOneshot flag is: %s\n", intrinsics.oneshot_flag ? "true" : "false");
  printf("\tMax oneshot interval is: %d\n", max_oneshot_interval);
  printf("\tAllowing reverse is: %s\n", allow_reverse ? "true" : "false");
  printf("\tChord length factor is: %f\n", intrinsics.default_chord_l_factor);
  printf("\tTime increment is: %f\n", time_incr);
  printf("\tMax number of iterations to search over is: %d\n",
         intrinsics.max_num_iterations);
  printf(
      "\tThe penalty structure contains:\n\t\tchange_dir: %f\n\t\tchange_head: "
      "%f\n\t\tchange_steering_index: %f\n\t\treversing: "
      "%f\n\t\tchange_velocity: %f\n\t\tchange_steering: %f\n",
      penalty_params.change_dir, penalty_params.change_head,
      penalty_params.change_steering_index, penalty_params.reversing,
      penalty_params.change_velocity, penalty_params.change_steering);
  printf("\tThe threshold structure contains:\n\t\theading_threshold_error: "
         "%f\n\t\tdistance_threshold_error: %f\n",
         stop_params.heading_threshold_error,
         stop_params.distance_threshold_error);
  printf("\tThe vehicle's dimensions are:\n\t\tlength: %f\n\t\twidth: "
         "%f\n\t\twheelbase: %f\n\t\tbaselink to front: %f\n",
         vehicle_params.length, vehicle_params.width, vehicle_params.wheelbase,
         vehicle_params.baselink_to_front);
  printf("\tThe dynamic constraint structure contains:\n\t\tmax_accel: "
         "%f\n\t\tmax_steering_vel: %f\n",
         vehicle_dyn_constraints.max_accel,
         vehicle_dyn_constraints.max_steering_vel);
  // quickly convert degrees back to radians
  stop_params.heading_threshold_error *= M_PI / 180;
  vehicle_dyn_constraints.max_steering_vel *= M_PI / 180;
  if (vm.count("headings") || !command_line_flag) {
    headings[0] = temp_headings[0] * M_PI / 180;
    headings[1] = temp_headings[1] * M_PI / 180;
    printf("\tHeading of the starting node is (in degrees): %f\n",
           headings[0] * 180 / M_PI);
    printf("\tHeading of the goal node is (in degrees): %f\n",
           headings[1] * 180 / M_PI);

    default_heading = false;
  } else {
    printf("\tHeadings will be taken from the scenario in .yaml file\n");
    default_heading = true;
  }
  printf("\tSmoothing class contains:\n\t\tnumber of steps: %d\n\t\tminimum "
         "turning radius: %f\n\t\tstep size: %f\n\t\tMax Obstacle distance "
         "penalising threshold: %f\n\t\tWeights: [%f,%f,%f]\n",
         num_iterations, min_turn_r, step_size, max_obs_dist, weights[0],
         weights[1], weights[2]);

  std::cout << std::endl << std::endl;
  return false;
}
