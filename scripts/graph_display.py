import hybrid_astar
import math
#from __future__ import print_function
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import OrderedDict 

#helper function to obtain a list of indices that have obstacles
def obstacles_index_list(graph):
    index_list = []
    for i in range(graph.nrows):
        for j in range(graph.ncols):
            if graph.isOccupiedAtIndex((i,j)):
                index_list.append((i,j))

    return index_list 

# function for statically displaying the results from A-star search. (breaks down to multiple functions for modularity)
#function to draw grid lines + set axis limits
def set_up_grid(ax, graph, label='on'):
    ax.set_xlim(graph.top_left_pos[0], graph.top_left_pos[0]+graph.ncols*graph.resolution[0])
    ax.set_ylim(graph.top_left_pos[1]+graph.nrows*graph.resolution[1], graph.top_left_pos[1])
    ax.set_xticks([graph.top_left_pos[0]+i*graph.resolution[0] for i in range(graph.ncols)])
    ax.set_yticks([graph.top_left_pos[1]+i*graph.resolution[1] for i in range(graph.nrows)])    
    ax.grid(True)
    #if label is off, have no numbers and leave blanks
    if label.lower() != 'on':
        ax.tick_params(axis='both', which='both', bottom=False, left=False, labelbottom=False, labelleft=False)

#function to draw starting and goal position
def draw_start_goal(ax, start_pos, goal_pos, **kwargs):
    handle_pos = ax.plot(start_pos.x, start_pos.y, marker='x', label='Start/End position', **kwargs)
    ax.plot(goal_pos.x, goal_pos.y, marker='x', **kwargs)
    #draw quivers to show the starting and ending orientations
    if start_pos.theta is not None and goal_pos.theta is not None:
        q = ax.quiver([start_pos.x, goal_pos.x], [start_pos.y, goal_pos.y], list(map(math.cos, [start_pos.theta, goal_pos.theta])), 
                  list(map(math.sin, [start_pos.theta-math.pi, goal_pos.theta-math.pi])), pivot='tail')
        ax.quiverkey(q, X=0.3, Y= 0.9, U=2, label='Orientation of start/end nodes', labelpos='E')

    return handle_pos[0]


#function to draw obstacles
def draw_obstacles(ax, graph, wall_color):
    obstacles_patches = []
    for row in range(graph.nrows):
        for col in range(graph.ncols):
            if graph.grid[row,col]:
                #get the position first
                center = graph.getPosition((row,col))
                bottom_left = (center[0]-graph.resolution[0]/2, center[1]-graph.resolution[1]/2)
                rect = Rectangle(bottom_left, graph.resolution[0], graph.resolution[1])
                obstacles_patches.append(rect)
                
    pc = PatchCollection(obstacles_patches, facecolor = wall_color, label='obstacles')
    ax.add_collection(pc)
    #create a patch for legend of obstacles
    wall_patch = mpatches.Patch(color=wall_color, label='obstacles')
    return wall_patch

#draw expanded nodes
def draw_expanded_nodes(ax, graph, came_from,  start_pos, raw_pos, nodes_limit=None, line_color='y'):
    if nodes_limit is None:
        nodes_limit = graph.total_cells
    
    counter = 0   
    came_from[graph.getIndexAll(start_pos)] = start_pos 
    for pos_index in came_from:
        #reach limit then break
        if counter > nodes_limit:
            break

        #connect the current node to its parents      
        ax.plot([raw_pos[pos_index].x, came_from[pos_index].x], [raw_pos[pos_index].y, came_from[pos_index].y], color = line_color, linestyle='-') 
        
        counter += 1
    
    nodes_limit = counter 

    came_from[graph.getIndexAll(start_pos)] = None

    #create dummy handle to appear in the legend
    some_line = mlines.Line2D([], [], color=line_color, markersize=15, label='{} Expanded Nodes'.format(nodes_limit))

    return some_line 

#draw path connecting the goal to the starting node
def draw_path(ax, graph, came_from, near_goal, start_pos, path_color='k', label='Path', get_all_handles=False):
    path = hybrid_astar.get_path(graph, came_from, near_goal, start_pos)
    handles_path = []
    for i in range(len(path)-1):
        handle = ax.plot([path[i].x, path[i+1].x],[path[i].y, path[i+1].y], color=path_color, linestyle='-', label=label)
        handles_path.append(handle[0])

    if get_all_handles:
        return handles_path 
    else:
        return handles_path[0] 

# it plots: obstacles, connects node to its parents, and an over-all global path
def display_results(came_from, raw_pos, start_pos, goal_pos, near_goal, graph, fig_size=(12,14), wall_color = 'y', show_all_parents=True, nodes_limit=None):
    #set up the plot
    fig, ax = plt.subplots(1, 1, figsize = fig_size)
    set_up_grid(ax, graph)

    
    #draw patches to represent obstacles
    draw_obstacles(ax, graph, wall_color)          
    
    #draw line segments connecting the expanded nodes
    if show_all_parents:
        draw_expanded_nodes(ax, graph, came_from, start_pos, raw_pos, nodes_limit)
    
    #draw line connecting the start to the end
    draw_path(ax, graph, came_from, near_goal, start_pos)
    
    #plot the starting and end points
    draw_start_goal(ax, start_pos, goal_pos)
    
    
    #show at the end
    plt.show()



#class to show live update of A-star algorithm, i.e. it would show the current neighbors being explored and update the parents when need be
class LiveUpdate:
    def __init__(self, start_pos, goal_pos, graph, heuristic_funcs, travel_func, stop_search_criteria, fig_size=(12,14), wall_color = 'm', duration = 10, display = True, use_optimise = True, precompute_class = hybrid_astar.BasePrecompute()):
        #store the stuff internally for access in other functions
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.graph = graph
        self.fig_size = fig_size 
        self.wall_color = wall_color
        self.duration = duration
        self.display = display
        self.use_optimise = use_optimise 
        self.precompute_class = precompute_class 

        #store the function handles
        self.heuristic_funcs = heuristic_funcs 
        self.travel_func = travel_func
        self.stop_search_criteria = stop_search_criteria 

        #initialise the figure + other stuff
        self.init_vars()
        

        #call the animator to run the playback
        self.play_back()

    
    #function to initialise variables used through out
    def init_vars(self):
        fig, ax = plt.subplots(1, 1, figsize = self.fig_size)
        self.fig = fig
        self.ax = ax
        
        start_index = self.graph.getIndexAll(self.start_pos)
        
        #create an data structures that will be frequently used in both astar and plot updates
        #use priority queue for popping off nodes
        if self.use_optimise:
            self.frontier = hybrid_astar.PriorityQueue()
            
            #putting the starting node
            self.frontier.put(self.start_pos, 0, start_index)

            self.closed_list = {}
        else:
            self.frontier = hybrid_astar.PriorityQueueBB()
        
            #putting the starting node
            self.frontier.put(self.start_pos, 0)

        #create dictionary of keys for the cost and the nodes' parents
        self.came_from = OrderedDict()
        self.came_from[start_index] = None

        self.cost_so_far = {}
        self.cost_so_far[start_index] = 0

        #near goal, if never reached then set to None
        self.near_goal = None
        self.finished_anim_flag = False 

        #just runs the precompute method of the precompute class
        self.precompute_class.precompute(self.goal_pos, self.start_pos, self.graph)

        #dictionary of graphic handles to the nodes in closed list
        self.node_handles = {}
        self.node_handles[start_index] = self.ax.plot([], [], color = 'y', linestyle = '--')[0]
        
        #set up axis limits and grid lines
        set_up_grid(self.ax, self.graph, label='off')
                    
        #plot the starting and end points
        draw_start_goal(self.ax, self.start_pos, self.goal_pos, markersize=10, color='b')
        
        #draw patches to represent obstacles
        draw_obstacles(self.ax, self.graph, self.wall_color)
        
        #handles for quivers of the current expanded nodes, size should be based on the size of the actions
        self.quiver_handles = []
        len_movement = len(self.graph.actions) 
        for i in range(len_movement):
            self.quiver_handles.append(self.ax.quiver(self.graph.origin[0], self.graph.origin[1], math.cos(0), math.sin(0), pivot='tail', visible=0, color='k'))


    #function to draw a clear frame of static objects (before the first frame is drawn)
    def init_func(self):

        #create list of handles to clear
        updated_handles = []
        updated_handles.extend(self.node_handles.values())
        updated_handles.extend(self.quiver_handles)

        return updated_handles
    

    #a generator mimicking functionality of a star such that it returns currently expanded neighbors and the parent
    def gen_neighbors(self):
        #counter = 0
        #run_until = 500
        while not self.frontier.empty(): # and counter < run_until:
            #counter += 1
            #pop the lowest cost node from the queue
            current = self.frontier.get()
            current_index = self.graph.getIndexAll(current)
            
            #early exit
            if self.stop_search_criteria(current, self.goal_pos, self.graph):
                self.near_goal = current
                yield current, [] 
                break

            neighbors_update = []

            #iterate through the neighbors
            for neighbor_index, neighbor in self.graph.get_neighbors(current):
                new_cost = self.cost_so_far[current_index] + self.travel_func(current, neighbor)
                #check distance between the neighbor and the current node (should technically, be the chord length
                #print("Chorld length is {} while the distance between neighbor and current node is {}".format(self.graph.chord_l, math.sqrt((current.x - neighbor.x)**2 + (current.y - neighbor.y)**2 )))

                #if the node is not present in the open list or lower cost can be achieved
                if neighbor_index not in self.cost_so_far or new_cost < self.cost_so_far[neighbor_index]:
                    #get the list of heuristics from the list of heuristic functions
                    heuristics_list = [heuristic_func(self.goal_pos, neighbor, self.graph) for heuristic_func in self.heuristic_funcs]
                    #then, update the priority based on the maxmimum of the heuristics
                    neighbor_twoD_index = self.graph.getIndex2D(neighbor)
                    priority = new_cost +  self.precompute_class.asimilate_heuristics(neighbor_twoD_index, self.goal_pos, neighbor, self.graph, heuristics_list)
                    
                    self.frontier.put(neighbor, priority)
                    self.came_from[neighbor_index] = current
                    self.cost_so_far[neighbor_index] = new_cost
                    neighbors_update.append((neighbor_index, neighbor))
            
            yield current, neighbors_update



        #come heres at the end of the loop, check if near goal has been set
        if self.near_goal is None:
            self.finished_anim_flag = True
            yield current, [] 

        

    #modified version of above but based on more optimised implementation of A* hybrid       
    def gen_neighbors_optimised(self):
        #counter = 0
        #run_until = 500
        
        while not self.frontier.empty(): # and counter < run_until:
            #counter += 1
            #pop the lowest cost node from the queue
            current = self.frontier.get()
            current_index = self.graph.getIndexAll(current)
            
            #add current to closed list
            self.closed_list[current_index] = current 
            
            #early exit
            if self.stop_search_criteria(current, self.goal_pos, self.graph):
                self.near_goal = current
                yield current, []
                break

            #iterate through possible actions
            for action in self.graph.actions:
            
                neighbor_index, neighbor = self.graph.get_neighbor(current, action)
                #if occuluded, not within bound or already in closed list then move on
                if neighbor_index is None or neighbor_index in self.closed_list:
                    continue
                
                #compute new cost
                new_cost = self.cost_so_far[current_index] + self.travel_func(current, neighbor)
                
                #check if node is present in open list
                in_open = neighbor_index in self.cost_so_far 

                

                if not in_open or new_cost < self.cost_so_far[neighbor_index]:
                    #get the list of heuristics from the list of heuristic functions
                    heuristics_list = [heuristic_func(self.goal_pos, neighbor, self.graph) for heuristic_func in self.heuristic_funcs]
                    #then, update the priority based on the maxmimum of the heuristics
                    neighbor_twoD_index = self.graph.getIndex2D(neighbor)
                    priority = new_cost +  self.precompute_class.asimilate_heuristics(neighbor_twoD_index, self.goal_pos, neighbor, self.graph, heuristics_list)
                    
                    self.cost_so_far[neighbor_index] = new_cost
                    self.came_from[neighbor_index] = current
                    
                    #if it existed, then remove it first
                    if in_open:
                        self.frontier.remove(neighbor_index)
                    
                    self.frontier.put(neighbor, priority, neighbor_index)

            
                    yield current, [(neighbor_index, neighbor)] 
        
        #come heres at the end of the loop, check if near goal has been set
        if self.near_goal is None:
            self.finished_anim_flag = True
            yield current, [] 
                               

    #function to run during the animation
    def animate(self, data):
        current, neighbors_update = data
        
        #set all arrows to be invisible
        for i in range(len(self.quiver_handles)):
            plt.setp(self.quiver_handles[i], visible=0)
        
        #currently updated handles
        updated_handles = []

        #if the handle has been created simply update, otherwise create one
        for i, to_be_unpacked  in enumerate(neighbors_update):
            neighbor_index, neighbor = to_be_unpacked
            if neighbor_index in self.node_handles:
                self.node_handles[neighbor_index].set_data([current.x, neighbor.x], [current.y, neighbor.y])
            else:
                self.node_handles[neighbor_index] = self.ax.plot([current.x, neighbor.x], [current.y, neighbor.y], color='y', linestyle='-')[0]
            
            #update quiver (to be completed later)
            if neighbor.theta:
                self.quiver_handles[i].set_offsets((neighbor.x, neighbor.y))
                self.quiver_handles[i].U[:] = math.cos(neighbor.theta)
                self.quiver_handles[i].V[:] = math.sin(neighbor.theta - math.pi)
                self.quiver_handles[i].set_visible(1)

            #updated_handles.append(self.node_handles[neighbor_index][0])
        
        updated_handles.extend(self.node_handles.values())
        if current.theta:
            updated_handles.extend(self.quiver_handles)
        
        #if the end has been reached (either through near_goal or flag)
        if self.near_goal is not None or self.finished_anim_flag:
            #draw overlaying path on top
            handles_path = draw_path(self.ax, self.graph, self.came_from, self.near_goal, self.start_pos, path_color='g', get_all_handles = True)
            updated_handles.extend(handles_path)
        #return updated_handles
        return updated_handles

    #set up the animation
    def play_back(self):
        #set up the animation
        if self.use_optimise:
            frames = self.gen_neighbors_optimised
        else:
            frames = self.gen_neighbors 

        self.anim = FuncAnimation(self.fig, func=self.animate, frames=frames, init_func=self.init_func, interval=self.duration, blit=True, repeat=False, save_count=self.graph.total_cells*100)
        
        if self.display: 
            plt.show()


