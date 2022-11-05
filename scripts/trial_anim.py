import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import hybrid_astar
import graph_display
import math
from collections import namedtuple 
import argparse
import yaml 
import time 

#function to display comparison of all algorithms based on the currently employed grid, starting position and goal position
display_live_expansions = ['hybrid', 'vanilla', 'dijsktra']
def display_static_results(discrete_graph, hybrid_graph, start_pos, goal_pos, nodes_limit = 10000, save_name = 'dead_end_trial', precompute_obj = hybrid_astar.BasePrecompute(), allow_reverse = True, expanded_type= display_live_expansions[0]):
    #first run the algorithm themselves
       
    #run hybrid A-star
    if allow_reverse:
        func_move = hybrid_astar.move_alldirs
        func_heuristic = hybrid_astar.reeds_shepp_length 
    else:
        func_move = hybrid_astar.move_forward 
        #func_heuristic = hybrid_astar.reeds_shepp_length 
        func_heuristic = hybrid_astar.dubins_length 
    
    t = time.time()
    came_from_hybrid, cost_so_far_hybrid, near_goal_hybrid, raw_pos_hybrid = \
            hybrid_astar.a_star_search_optimise(hybrid_graph, [hybrid_astar.Euclidean_distance, func_heuristic], 
                                                func_move, hybrid_astar.xyInd_headRaw, start_pos, goal_pos,
                                                precompute_class=precompute_obj)
    elapsed_time = time.time() - t
    print("Hybrid A-star took {} seconds to search".format(elapsed_time))
    assert near_goal_hybrid is not None, "Path for Hybrid A-star not found"
    print("Cost for reaching goal is {}\n".format(cost_so_far_hybrid[hybrid_graph.getIndexAll(near_goal_hybrid)]))
    path_hybrid = hybrid_astar.get_path(hybrid_graph, came_from_hybrid, near_goal_hybrid, start_pos)

    #dijkstra's algorithm
    t = time.time()
    came_from_dij, cost_so_far_dij, near_goal_dij, raw_pos_dij = \
    hybrid_astar.a_star_search_optimise(discrete_graph, [hybrid_astar.no_heuristic], hybrid_astar.Euclidean_distance,
                               hybrid_astar.xyInd, start_pos, goal_pos)
    elapsed_time = time.time() - t
    print("Dijsktra took {} seconds to search".format(elapsed_time))
    assert near_goal_dij is not None, "Path for Dijsktra's not found"
    print("Cost for reaching goal is {}\n".format(cost_so_far_dij[discrete_graph.getIndexAll(near_goal_dij)]))
    path_dij = hybrid_astar.get_path(discrete_graph, came_from_dij, near_goal_dij, start_pos)

    #vanilla A-star
    t = time.time()
    came_from_vani, cost_so_far_vani, near_goal_vani, raw_pos_vani = \
    hybrid_astar.a_star_search_optimise(discrete_graph,[hybrid_astar.Euclidean_distance], hybrid_astar.Euclidean_distance,
                               hybrid_astar.xyInd, start_pos, goal_pos)
    elapsed_time = time.time() - t
    print("Vanilla A-star took {} seconds to search".format(elapsed_time))
    assert near_goal_vani is not None, "Path for vanilla A-star not found"
    print("Cost for reaching goal is {}\n".format(cost_so_far_dij[discrete_graph.getIndexAll(near_goal_dij)]))
    path_vani = hybrid_astar.get_path(discrete_graph, came_from_vani, near_goal_vani, start_pos)
    
    fig, ax = plt.subplots(1,1, figsize=[10,10])
    #set up the grid
    graph_display.set_up_grid(ax, discrete_graph, label='off')
    #draw the obstacles
    handle_obstacles = graph_display.draw_obstacles(ax, discrete_graph, 'm')
    #draw expanded nodes from hybrid A-star
    if expanded_type.lower() == display_live_expansions[0]:
        handle_expanded = graph_display.draw_expanded_nodes(ax, hybrid_graph, came_from_hybrid, start_pos, raw_pos_hybrid, nodes_limit=nodes_limit)
    #draw expanded nodes from vanilla A-star
    elif expanded_type.lower() == display_live_expansions[1]:
        handle_expanded = graph_display.draw_expanded_nodes(ax, discrete_graph, came_from_vani, start_pos, raw_pos_vani, nodes_limit=nodes_limit)
    #draw expanded nodes from dijsktra
    elif expanded_type.lower() == display_live_expansions[2]:
        handle_expanded = graph_display.draw_expanded_nodes(ax, discrete_graph, came_from_dij, start_pos, raw_pos_dij, nodes_limit=nodes_limit)

    #draw the starting and goal position with orientations
    handle_node = graph_display.draw_start_goal(ax, start_pos, goal_pos, color='r', markersize=20)
    #draw path from vanilla A-star
    handle_path_vani = graph_display.draw_path(ax, discrete_graph, came_from_vani, near_goal_vani, start_pos, path_color = 'k', label='Vanilla Astar')
    #draw path from Dijsktra
    handle_path_dij = graph_display.draw_path(ax, discrete_graph, came_from_dij, near_goal_dij, start_pos, path_color = 'b', label='Dijsktra')
    #draw path from hybrid A-star
    handle_path_hybrid = graph_display.draw_path(ax, hybrid_graph, came_from_hybrid, near_goal_hybrid, start_pos, path_color = 'g', label='Hybrid A-star')
    #legend for the graphical objects
    ax.legend(handles=[handle_obstacles, handle_expanded, handle_node, handle_path_vani, handle_path_dij, handle_path_hybrid])
    
    fig.savefig(save_name)
    plt.show()



#------------------------------------------Set up scene------------------------------------------------
def set_up_scene(discrete_graph, hybrid_graph, scenario_name, pos_configs):
    #fill in the grid
    grid = np.zeros(discrete_graph.grid.shape)
    obs_loc = scene_dict[scenario_name]['obs_loc']
    for loc in obs_loc:
        grid[loc[0]:loc[1]+1, loc[2]:loc[3]+1] = 1.0
    
    discrete_graph.setGrid(grid)
    hybrid_graph.setGrid(grid)

    #fill in the starting and goal position
    temp_pos = discrete_graph.getPosition(scene_dict[scenario_name]['start_ind'][:-1])
    start_pos = hybrid_astar.rob_pos(x=temp_pos[0], y=temp_pos[1], theta=pos_configs.start)
    temp_pos = discrete_graph.getPosition(scene_dict[scenario_name]['goal_ind'][:-1])
    goal_pos = hybrid_astar.rob_pos(x=temp_pos[0], y=temp_pos[1], theta=pos_configs.goal)

    return start_pos, goal_pos


#------------------------------------------Scenario-based plot/vid-------------------------------------
headings = namedtuple('heading_config_deg', ['start', 'goal'])
possible_actions = ['save pic', 'play vid']
base_scenarios = ['empty grid', 'dead end']
heading_configs = {base_scenarios[0]: headings(start=-90, goal=90),
                   base_scenarios[1]: headings(start=0, goal=90)}

def scenario_plot_or_vid(scenario_name = base_scenarios[0], pos_configs = (heading_configs[base_scenarios[0]].start, heading_configs[base_scenarios[0]].goal), steering_actions= [-10,0,10], num_cells_penalties = (hybrid_astar.penalty_change_dir, hybrid_astar.penalty_change_head, 100),  chord_factor=1.0, allow_reverse = True, precompute_flag = True, action = possible_actions[0], nodes_limit = 10000, display_live = display_live_expansions[0]):
    #default it to the case of empty grid if scenario not found
    if scenario_name.lower() not in base_scenarios:
        scenario_name = base_scenarios[0]
        print("Requested scenario has not been implemented yet; {} scenario picked instead".format(scenario_name))
    
    #default to saving picture if action not found
    if action.lower() not in possible_actions:
        action = possible_actions[0]
        print("Requested action not found; {} action picked instead".format(action))
    
    #show expanded nodes from which method: hybrid, vanilla or simply dijsktra?
    if display_live.lower() not in display_live_expansions:
        display_live = display_live_expansions[0]
        print("Requested display type not found; choosing {} instead".format(display_live))

    
    #get default parameters for grids from the size
    origin = [0,0]
    resolution = scene_dict[scenario_name]['resolution']
    desired_grid_dim = scene_dict[scenario_name]['size']
    length = [desired_grid_dim[1]*resolution[1], desired_grid_dim[0]*resolution[0]]
    
    #set up the discrete graph for vanilla A-star and Dijskra
    discrete_graph = hybrid_astar.DiscreteGrid(origin=origin, length=length, resolution=resolution)
    #set up hybrid graph
    chord_l = chord_factor*math.sqrt(resolution[0]**2 + resolution[1]**2)
    hybrid_graph = hybrid_astar.GridGraph(origin=origin, length=length, resolution=resolution, resolution_heading=resolution[2], steering_actions= steering_actions, chord_l = chord_l, allow_reverse=allow_reverse)

    #convert the pos_configs to be in radians instead (expected is degrees)
    pos_configs = headings(*map(lambda x: x*math.pi/180, pos_configs))
    
    #setting up the starting positions and the grid
    start_pos, goal_pos = set_up_scene(discrete_graph, hybrid_graph, scenario_name, pos_configs) 
                
    #configure the penalties of A-star/Hybrid A-start
    hybrid_astar.penalty_change_dir = num_cells_penalties[0]*(math.sqrt(hybrid_graph.resolution[0]**2 + hybrid_graph.resolution[1]**2))
    hybrid_astar.penalty_change_head = num_cells_penalties[1]*(math.sqrt(hybrid_graph.resolution[0]**2 + hybrid_graph.resolution[1]**2))

    default_value_when_not_found = num_cells_penalties[2]*(math.sqrt(hybrid_graph.resolution[0]**2 + hybrid_graph.resolution[1]**2))
    #default_func_when_not_found = hybrid_astar.reeds_shepp_length
    default_func_when_not_found = None 
    #default_factor = 5.0
    #assim_mode = 'penalty'
    assim_mode = 'append'
    default_factor = 1.0

    #configure precomputation of heuristics if enabled
    if precompute_flag:
        precompute_obj = hybrid_astar.TwoD_AStar(default_func_when_not_found=default_func_when_not_found, 
                                                 default_factor=default_factor, assim_mode = assim_mode,
                                                 default_value_when_not_found=default_value_when_not_found)
    else:
        precompute_obj = hybrid_astar.BasePrecompute()

    #if asked to save picture, then save it
    if action == possible_actions[0]:
        file_name = "../figs/{}_sa-{}_hs-{}_p-{}_cf-{}_ar-{}_pf-{}_en-{}.png".format(scenario_name, steering_actions, map(lambda x: round(x*180/math.pi, 2), pos_configs), 
                                                                                     num_cells_penalties, chord_factor, allow_reverse, precompute_flag, display_live) 
        display_static_results(discrete_graph, hybrid_graph, start_pos, goal_pos, save_name=file_name, precompute_obj=precompute_obj, 
                               allow_reverse=allow_reverse, expanded_type=display_live, nodes_limit=nodes_limit)
    
    #if asked to play live video then do it
    elif action == possible_actions[1]:
        if allow_reverse:
            func_move = hybrid_astar.move_alldirs
            func_heuristic = hybrid_astar.reeds_shepp_length 
        else:
            func_move = hybrid_astar.move_forward 
            func_heuristic = hybrid_astar.dubins_length 
        
         
        #hybrid A-star
        if display_live.lower() == display_live_expansions[0]:
            graph_display.LiveUpdate(start_pos, goal_pos, hybrid_graph, [hybrid_astar.Euclidean_distance, hybrid_astar.reeds_shepp_length],
                                     hybrid_astar.move_alldirs, hybrid_astar.xyInd_headRaw, duration=1, fig_size=(12,11), use_optimise=True,
                                     precompute_class = precompute_obj) 
        #vanilla A-star
        elif display_live.lower() == display_live_expansions[1]:
            graph_display.LiveUpdate(start_pos, goal_pos, discrete_graph, [hybrid_astar.Euclidean_distance], 
                                     hybrid_astar.Euclidean_distance, hybrid_astar.xyInd, duration=10, fig_size=(12,11), use_optimise=True)
        
        #Dijsktra's
        elif display_live.lower() == display_live_expansions[2]:
            graph_display.LiveUpdate(start_pos, goal_pos, discrete_graph, [hybrid_astar.no_heuristic], 
                                     hybrid_astar.Euclidean_distance, hybrid_astar.xyInd, duration=10, fig_size=(12,11), use_optimise=True)


def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

#function to read scenarios file and return dictionary
def read_scene_yaml(filename="scenarios.yaml"):
    with open(filename, 'r') as stream:
        try:
            scene_dict = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            assert 0, "Something wrong with file: {}".format(exc)

    #get default sceneic configuration
    assert 'default' in scene_dict, "The scenarios configuration must contain the default scene" 
    default_config = scene_dict['default']
    
    #fill in missing field in non-default scenarios + fill in heading_configs
    heading_configs = {}
    for key in scene_dict:
        if key == 'default':
            heading_configs[key] = headings(start=scene_dict[key]['start_ind'][2], goal=scene_dict[key]['goal_ind'][2])
            pass

        #check if some default configurations are not present in the particular scene and prefill them
        for default_key in default_config:
            if default_key not in scene_dict[key]:
                scene_dict[key][default_key] = default_config[default_key]
        
        heading_configs[key] = headings(start=scene_dict[key]['start_ind'][2], goal=scene_dict[key]['goal_ind'][2])

    return scene_dict, scene_dict.keys(), heading_configs  

#main function when this is called from command line
if __name__ == '__main__':
    #read the scenarios yaml file to get the default value of things
    scene_dict, base_scenarios, heading_configs = read_scene_yaml()

    #make argument parser to get arguments from the user
    parser = argparse.ArgumentParser(description='Show sample solutions or live performance of path planning algorithms')
    parser.add_argument("-sn", "--scenario_name", help="Gives the name of the scenario being simulated. Currently only two are implemented: [empty grid] and [dead end];"+
                        " default: [empty grid]", default= base_scenarios[0], choices = base_scenarios)
    parser.add_argument("-hs", "--headings", help = "List of length two containing the start and ending nodes' headings in degrees. Default will be scenario specific",
                        default = None, type=float, nargs=2)
    parser.add_argument("-sa", "--steering_actions", help="List of length n containing steering actions in degrees for the Hybrid A-Star. Default is [-10,0,10]",
                        default=[-10,0,10], type=float, nargs="+")
    
    parser.add_argument("-p", "--penalties", help = "List of three penalties for the hybrid A-star algorithm. The unit is in number of diagonals of the cell. " +
                        "The penalties are (in order) for change in direction, for change in heading and default cost if the cell was not visited during the precomputation",
                        default = [10,10,100], type=float, nargs=3)
    parser.add_argument("-ar", "--allow_reverse", help="Allow the vehicle to go in reverse?. Default to True.", choices=[True, False], 
                        default=True, type=str2bool)
    parser.add_argument("-pf", "--precompute_flag", help="Allow precomputation to speed up the Hybrid A-star search? Default to True.",
                        default=True, type=str2bool)
    parser.add_argument("-en", "--expanded_nodes", help="Choose which algorithm's expanded nodes you want to see", choices=display_live_expansions, 
                        default = display_live_expansions[0])
    parser.add_argument("-cf", "--chord_factor", help="Factor to scale the default chord length whose value is the diagonal of the cell", default=1.0,
                        type=float)
    parser.add_argument("-nl", "--nodes_limit", help="Number of expanded nodes to be shown for static picture", default=10000, type=long)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-lu", "--live_update", help="Will show live expansion of nodes from the chosen algorithm", const=possible_actions[1], 
                       default=possible_actions[0], action= 'store_const')
    group.add_argument("-sp", "--save_pic", help="Will save the picture of the planned paths with the names indicative of parsed parameters. This option is"+
                       " chosen by default instead of -lu", const=possible_actions[0], nargs='?')


    args = parser.parse_args()
    
    #if no heading is specified, default is read from the scenario dictionary
    if args.headings is None:
        args.headings = [scene_dict[args.scenario_name]['start_ind'][2],  scene_dict[args.scenario_name]['goal_ind'][2]]
    
    #pass the parsed arguments to the function to save picture or play live video
    scenario_plot_or_vid(scenario_name=args.scenario_name, pos_configs=args.headings, steering_actions=args.steering_actions, num_cells_penalties=args.penalties, 
                         chord_factor=args.chord_factor, allow_reverse=args.allow_reverse, precompute_flag=args.precompute_flag, nodes_limit=args.nodes_limit, action=args.live_update,
                         display_live=args.expanded_nodes)


