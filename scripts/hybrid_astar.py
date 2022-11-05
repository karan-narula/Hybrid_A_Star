import heapq
import math
import numpy as np 
from collections import namedtuple, OrderedDict
import itertools
import reeds_shepp
import dubins

#-------------------------------Priority Queues ---------------------------------------
class PriorityQueueBB:
    def __init__(self):
        #for the priority queue
        self.elements = []

    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


class PriorityQueue:
    def __init__(self):
        #for the priority queue
        self.elements = []
        #dictionary for removal based on index (since absolute position will differ)
        self.look_up = {}

    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority, key):
        heapq.heappush(self.elements, (priority, item))
        self.look_up[key] = (priority, item)

    def get(self):
        return heapq.heappop(self.elements)[1]

    
    def ismember(self, key):
        return key in self.look_up  

    
    def remove(self, key):
        #print(self.elements)
        #print(priority, item)
        self.elements.remove(self.look_up[key])





#-------------------------------Kinematic Models + config ---------------------------------------
nh_car_steer = namedtuple('robot_config', 'L')
rob_pos = namedtuple('position', ['x','y','theta', 'dir'])
default_fields = [None,]*(len(rob_pos._fields)-1)
default_fields.append(0)
rob_pos.__new__.__defaults__ = tuple(default_fields)

default_car = nh_car_steer(L=2)

def non_holo_car_steer(cur_pos, chord_l, steer, car_config):
    direction = 0 if chord_l > 0 else 1
    next_pos = rob_pos(x = cur_pos.x + chord_l*math.cos(cur_pos.theta), y=cur_pos.y + chord_l*math.sin(cur_pos.theta), theta=cur_pos.theta + (chord_l/car_config.L)*math.tan(steer*math.pi/180), dir=direction)
    return next_pos


#get the possible heading resolution from here, user may specify something too coarse
def get_heading_resolu_nh_car_steer(steering_actions, chord_l, car_config):
    increments = list(map(lambda x: math.fabs((chord_l/car_config.L)*math.tan(x*math.pi/180)), steering_actions))
    if 0 in increments:
        increments.remove(0)
    return min(increments)






#-------------------------------Grid structure ---------------------------------------
class RectGrid:
    def __init__(self, origin = (0,0), length = (1.2,2.0), resolution = (0.05, 0.05)):
        #some error checking
        assert len(length) == 2, 'Grid dimensions must have 2 elements'
        if not hasattr(resolution, '__iter__'):
            resolution = (resolution, resolution)
        assert len(resolution) >= 2, 'resolution should be given atleast for both x and y directions'
        assert length[0] >= resolution[0] and length[1] >= resolution[1], "Grid dimensions must be more than cell's resolution"

        #store the arguments
        self.origin = origin
        self.resizeGrid(length, resolution)
        
        self.actions = None

    #set the whole grid
    def setGrid(self, grid):
        assert self.grid.shape == grid.shape, "Dimension must be consistent"
        self.grid = grid


    #resize the grid
    def resizeGrid(self, length, resolution):
        #stores resolution
        self.resolution = (float(resolution[0]), float(resolution[1]))

        #calculate the number of rows and columns of cells
        self.ncols = int(math.ceil(round(length[0]/resolution[0], 5)))
        self.nrows = int(math.ceil(round(length[1]/resolution[1], 5)))

        #the coordinate of the topleft corner of the grid (corresponds to index 0)
        self.top_left_pos = (self.origin[0] - (self.ncols/2.0)*resolution[0], self.origin[1] - (self.nrows/2.0)*resolution[1])  

        #create array of all zeros with the above dimension
        self.grid = np.zeros((self.nrows, self.ncols))

        #adjust the length accordingly
        self.length = (self.ncols*resolution[0], self.nrows*resolution[1])

        #total number of cells
        self.total_cells = self.nrows*self.ncols

    #resets origin
    def setOrigin(self, origin):
        self.origin = origin
        
        #the coordinate of the topleft corner of the grid (corresponds to index 0)
        self.top_left_pos = (self.origin[0] - (self.ncols/2.0)*self.resolution[0], self.origin[1] - (self.nrows/2.0)*self.resolution[1])  

    
    #set value of the grid at a position
    def setGridAtPos(self, position, value):
        row_num, col_num = self.getIndex(position, return_type= 'tuple')
        self.grid[row_num, col_num] = value


    #set value of the grid at an index
    def setGridAtIndex(self, index, value):
        #check if index is a list of row, and column number or a single number
        if type(index) is list or type(index) is tuple:
            assert len(index) == 2, "should contain only row number and column number"
            self.grid[index[0], index[1]] = value
        else:
            row, col = divmod(index, self.ncols)
            self.grid[row,col] = value

 
    #fetch the value of the cell given position
    def getGridAtPos(self, position):
        row_num, col_num = self.getIndex(position, return_type= 'tuple')
        return self.grid[row_num, col_num]


    #fetch the value of the cell given index
    def getGridAtIndex(self, index):
        #check if index is a list of row, and column number or a single number
        if type(index) is list or type(index) is tuple:
            assert len(index) == 2, "should contain only row number and column number"
            return self.grid[index[0], index[1]]
        else:
            row, col = divmod(index, self.ncols)
            return self.grid[row,col]


    #get the cell index given an arbitary position 
    def getIndex(self, position, return_type = 'value'):
        if position[1] == self.top_left_pos[1]:
            row_num = 0
        else:
            diffy = position[1] - self.top_left_pos[1]
            row_num = int(math.ceil(diffy/self.resolution[1])) - 1
        
        if position[0] == self.top_left_pos[0]:
            col_num = 0
        else:
            diffx = position[0] - self.top_left_pos[0]
            col_num = int(math.ceil(diffx/self.resolution[0])) - 1
        
        
        if return_type == 'value':
            return (row_num)*self.ncols + col_num
        else:
            return (row_num, col_num)

    #identical to above
    def getIndex2D(self, position):
       return self.getIndex(position)

    #get the center position of the cell given index
    def getPosition(self, index):
        #check if index is a list of row and column or a single number
        if type(index) is list or type(index) is tuple:
            assert len(index) == 2, "should contain only row number and column number"
            return [self.top_left_pos[0] + (index[1]+0.5)*self.resolution[0], self.top_left_pos[1] + (index[0]+0.5)*self.resolution[1]]
        else:
            row, col = divmod(index, self.ncols)
            return [self.top_left_pos[0] + (col+0.5)*self.resolution[0], self.top_left_pos[1] + (row+0.5)*self.resolution[1]]
    
    
    #check if a particular index is occupied
    def isOccupiedAtIndex(self, index):
        return self.getGridAtIndex(index) > 0
    
    #check if a particular poisiton is occupied
    def isOccupiedAtPos(self, position):
        return self.isOccupiedAtIndex(self.getIndex(position))




class GridGraph(RectGrid) :
    def __init__(self, origin = (0,0), length = (1.2,2.0), resolution = (0.05, 0.05), resolution_heading = 1, chord_l = 0.1, steering_actions = [-40,0,40], kinematic_model=non_holo_car_steer, car_config=default_car, allow_reverse = False):
        #initialise the inherited class
        RectGrid.__init__(self, origin, length, resolution)
        
        self.chord_l = chord_l
        #check if chordlength satisfies the constraint for hybrid A*
        #assert self.chord_l > math.sqrt(resolution[0]**2 + resolution[1]**2), "Chord length should be greater than the hypotenuse of rectangular grid"

        
        #steering angles given are in degrees
        self.movement = steering_actions
        
        #store the minimum turning radius 
        self.min_turn_r = car_config.L/math.tan(max(list(map(lambda x: math.pi*math.fabs(x)/180, steering_actions))))

        #heading resolution in degrees
        max_resolution_heading = get_heading_resolu_nh_car_steer(steering_actions, chord_l, car_config)
        self.resolution_heading = resolution_heading*math.pi/180
        if self.resolution_heading > max_resolution_heading:
            self.resolution_heading = max_resolution_heading

        self.nlayers = int(math.ceil(2*math.pi/self.resolution_heading))
        self.bottom = -math.pi

        #update the total number of cells
        self.total_cells *= self.nlayers
        
        #store the kinematic model and the configuration of the car
        self.kinematic_model = kinematic_model

        self.car_config = car_config

        self.allow_reverse = allow_reverse
        
        #allow for reverse chord if user wanted
        if self.allow_reverse:
            temp  = [self.movement, [-self.chord_l, self.chord_l]]  
        else:
            temp = [self.movement, [self.chord_l]]

        self.actions = list(itertools.product(*temp))

    #allow for resetting actions/movements
    def set_movements(self, movement):
        self.movement = movement 
        
        #allow for reverse chord if user wanted
        if self.allow_reverse:
            temp  = [self.movement, [-self.chord_l, self.chord_l]]  
        else:
            temp = [self.movement, [self.chord_l]]

        self.actions = list(itertools.product(*temp))


    #check if position is in bounds
    def inBounds(self, position):
        index = self.getIndex(position, 'tuple')
        return index[0] >= 0 and index[0] < self.nrows and index[1] >= 0 and index[1] < self.ncols

    
    #check if position is passable (i.e. no obstacle in grid)
    def passable(self, position):   

        return not self.isOccupiedAtPos(position)


    #alternate passable test function which divides the line connecting current node into neighbors into small segments
    def passable_trial(self, current_pose, next_pose, div=10):
        dx = next_pose[0] - current_pose[0]
        dy = next_pose[1] - current_pose[1]
        l = math.sqrt(dx**2 + dy**2)   #this is simply chord_l !, can just pass that to avoid extra comp
        theta = math.atan2(dy,dx)

        #check if anything in between is an obstacle, go along the heading of current_position at small increments
        for i in range(1,div):
            hypo = i*l/div
            position = (current_pose[0] + hypo*math.cos(theta), current_pose[1] + hypo*math.sin(theta))
            if not self.passable(position):
                return False


        return True 
                



##    #get index all just returns get index of RectGrida
##    def getIndexAll(self, position, return_type = 'value'):
##        return self.getIndex(position, return_type)


    #get all the index for the layer as well
    def getIndexAll(self, position, return_type = 'value'):
        prelim_index = self.getIndex((position.x,position.y), return_type)
        #bind theta to be between -pi and pi
        theta = position.theta
        if theta < -math.pi:
            theta += 2*math.pi
        elif theta > math.pi:
            theta -= 2*math.pi

        #convert to index now
        if theta == self.bottom:
            lay_num = 0
        else:
            difft = theta - self.bottom
            lay_num = int(math.ceil(difft/self.resolution_heading)) - 1

        if return_type == 'value':
            return (lay_num)*self.nrows*self.ncols + prelim_index 
        else:
            return (prelim_index[0], prelim_index[1], lay_num)

    #get 2D index
    def getIndex2D(self, position):
       return self.getIndex((position.x, position.y))

    #generate list of neighbors
    def get_neighbors(self, rob_pose):
        neighbors = []
        neighbors_index = []


        for action in self.actions:
            n_index, next_pose = self.get_neighbor(rob_pose, action)
            if n_index is not None:
                neighbors.append(next_pose)
                neighbors_index.append(n_index)

        
        return zip(neighbors_index, neighbors)



    #generate a single neighbor based on current movement
    def get_neighbor(self, rob_pose, movement):
        steer, chord_l = movement
        next_pose = self.kinematic_model(rob_pose, chord_l, steer, self.car_config)
        if self.inBounds((next_pose.x, next_pose.y)) and self.passable_trial((rob_pose.x, rob_pose.y), (next_pose.x, next_pose.y)):
            return self.getIndexAll(next_pose), next_pose 
        else:
            return None, None


class DiscreteGrid(RectGrid):
    def __init__(self, origin = (0,0), length = (1.2,2.0), resolution = (0.05, 0.05), movement = [-1,0,1], allow_reverse = None):
        RectGrid.__init__(self, origin, length, resolution)
        self.movement = movement 
        self.allow_reverse = allow_reverse
       
        self.actions = list(itertools.product(*[self.movement, self.movement]))
        if (0,0) in self.actions:
            self.actions.remove((0,0))
        
        #can turn instantenously 90 degrees
        self.min_turn_r = 0

    #check if it is inbounds
    def inBounds(self, index):
        return index[0] >= 0 and index[0] < self.nrows and index[1] >= 0 and index[1] < self.ncols

    #check if index is passable (i.e. not occupied)
    def passable(self, index):
        return not self.isOccupiedAtIndex(index)
    
    #get index all just returns get index of RectGrid
    def getIndexAll(self, position, return_type = 'value'):
        return self.getIndex(position, return_type)


    #generate list of neighbors
    def get_neighbors(self, rob_pose):
        neighbors = []
        neighbors_index = []

        rob_index = self.getIndex(rob_pose, 'tuple')
        
        for action in self.actions:
            n_index, next_pose = self.get_neighbor(rob_pose, action, rob_index)
            if n_index:
                neighbors.append(next_pose)
                neighbors_index.append(n_index)


        return zip(neighbors_index, neighbors)

    
    #get a single neighbor
    def get_neighbor(self, rob_pose, action, rob_index=None):
        if rob_index is None:
            rob_row, rob_col = self.getIndex(rob_pose, 'tuple')
        else:
            rob_row, rob_col = rob_index
            

        move_x, move_y = action
        n_index = (rob_row + move_y, rob_col + move_x)
        next_pose = rob_pos(*self.getPosition(n_index))
        if self.inBounds(n_index) and self.passable(n_index):
            return (n_index[0])*self.ncols + n_index[1], next_pose 
        else:
            return None, None


#-------------------------------Heuristic functions ---------------------------------------
#euclidean distance 
def Euclidean_distance(c_pose, n_pose, graph=None):
    return math.sqrt((c_pose.x-n_pose.x)**2 + (c_pose.y-n_pose.y)**2)


#no heuristic (return 0), algorithm converges to Dijsktra's algorithm
def no_heuristic(c_pose, n_pose, graph=None):
    return 0

#--------Pre-calculate Heuristics--------
#the base class for others to inherit and overwrite (should be using ABCs in reality)
class BasePrecompute():
    def __init__(self, default_value_when_not_found = 0, default_func_when_not_found = None, default_factor=1.0, assim_mode='append'):
        self.assim_mode = assim_mode 
        self.default_value_when_not_found = default_value_when_not_found 
        self.default_factor = default_factor 
        if default_func_when_not_found is None:
            self.default_func_when_not_found = lambda *x: default_value_when_not_found
        else:
            self.default_func_when_not_found = default_func_when_not_found 
    
    def precompute(self, goal_pos, start_pos, graph):
        self.cost_so_far = {}


    def get(self, index, c_pose, n_pose, graph):
        return self.default_factor*self.cost_so_far.get(index, self.default_func_when_not_found(c_pose, n_pose, graph))


    #simply do maximum or penalises
    def asimilate_heuristics(self, index, c_pose, n_pose, graph, heuristics_list):
        precompute_heuristics = self.get(index, c_pose, n_pose, graph)
        if self.assim_mode == 'append':
            heuristics_list.append(precompute_heuristics)
            return max(heuristics_list)
        elif self.assim_mode == 'penalty':
            return max(heuristics_list) + precompute_heuristics 
        else:
            return max(heuristics_list)



#2D A-star (to help with dead-ends)
class TwoD_AStar(BasePrecompute):
    def __init__(self, default_value_when_not_found=0, default_func_when_not_found = None, default_factor=1.0, assim_mode='append'):
        BasePrecompute.__init__(self, default_value_when_not_found, default_func_when_not_found, default_factor, assim_mode)

    def precompute(self, goal_pos, start_pos, graph):
        #create equivalent discrete graph
        discrete_graph = DiscreteGrid(origin=graph.origin, length=graph.length, resolution=graph.resolution)
        discrete_graph.setGrid(graph.grid)
        _, self.cost_so_far, _, _ =  a_star_search_optimise(discrete_graph, [Euclidean_distance], Euclidean_distance, xyInd, goal_pos, start_pos)


#2D Disktra (to help with dead-ends)
class TwoD_Dijsktra(BasePrecompute):
    def __init__(self, default_value_when_not_found=0, default_func_when_not_found = None, default_factor=1.0, assim_mode='append'):
        BasePrecompute.__init__(self, default_value_when_not_found, default_func_when_not_found, default_factor, assim_mode)

    def precompute(self, goal_pos, start_pos, graph):
        #create equivalent discrete graph
        discrete_graph = DiscreteGrid(origin=graph.origin, length=graph.length, resolution=graph.resolution)
        discrete_graph.setGrid(graph.grid)
        _, self.cost_so_far, _, _ =  a_star_search_optimise(discrete_graph, [no_heuristic], Euclidean_distance, xyInd, goal_pos, start_pos)
        


#----non-holonomic heuristic but no obstacles------
def reeds_shepp_length(c_pose, n_pose, graph):
    return reeds_shepp.path_length(c_pose, n_pose, graph.min_turn_r)


def dubins_length(c_pose, n_pose, graph):
    return dubins.shortest_path(c_pose, n_pose, graph.min_turn_r).path_length()


#-------------------------------Global cost---------------------------------------
#define for moving forward and in all directions
penalty_change_head = 0.05 #5 cm for every 1 degree
penalty_change_dir = 0.1 #10 cm for change of direction
def move_forward(c_pose, n_pose):
    return Euclidean_distance(c_pose, n_pose) + math.fabs(c_pose.theta-n_pose.theta)*penalty_change_head 

def move_alldirs(c_pose, n_pose):
    return move_forward(c_pose, n_pose) + penalty_change_dir*abs(c_pose.dir - n_pose.dir) 




#-------------------------------Stopping Criteria ---------------------------------------
#this one checks if the index of the xy matches and heading error is below certain threshold (degrees)
def xyInd_headRaw(current, goal, graph, heading_threshold_error = 3, debug_mode = False):
    heading_discrep = current.theta - goal.theta
    if heading_discrep > math.pi:
        heading_discrep -= 2*math.pi
    elif heading_discrep < -math.pi:
        heading_discrep += 2*math.pi
    
    if debug_mode:
        print("Current node's position {} while node's position {}".format((current.x,current.y, current.theta), (goal.x, goal.y, goal.theta)))
        print("Current's index {} while goal's index {}".format(graph.getIndex(current), graph.getIndex(goal)))
        print("Discrepancy in heading is {}".format(math.fabs(heading_discrep)*180/math.pi))
    
    return graph.getIndex(current) == graph.getIndex(goal) and math.fabs(heading_discrep)*180/math.pi < heading_threshold_error


#checks only the x and y discrete index
def xyInd(current, goal, graph):
    return graph.getIndexAll(current) == graph.getIndexAll(goal)



#-------------------------------A Star ---------------------------------------
#function for normal a-star
def a_star_search(graph, heuristic_funcs, travel_func, stop_search_criteria, start, goal, debug_mode=True, precompute_class=BasePrecompute()):
    #use priority queue for popping off nodes
    frontier = PriorityQueueBB()
    
    #putting the starting node
    frontier.put(start, 0)

    #create dictionary of keys for the cost and the nodes' parents
    start_index = graph.getIndexAll(start)
    came_from = OrderedDict()
    came_from[start_index] = None
    raw_pos = {}
    raw_pos[start_index] = start 
    cost_so_far = {}
    cost_so_far[start_index] = 0
       

    #near goal, if never reached then set to None
    near_goal = None
    
    #for counting how many times were spent in the loop
    num_loops = 0
    
    #just runs the precompute method of the precompute class
    precompute_class.precompute(goal, start, graph)
    
    #iterate through the openlist until its empty 
    while not frontier.empty():
        num_loops += 1

        #pop the lowest cost node from the queue
        current = frontier.get()
        current_index = graph.getIndexAll(current)
        

        #early exit
        if stop_search_criteria(current, goal, graph):
            near_goal = current
            break

        #iterate through the neighbors
        for neighbor_index, neighbor in graph.get_neighbors(current):
            new_cost = cost_so_far[current_index] + travel_func(current, neighbor)
            #if the node is not present in the open and closed list or lower cost can be achieved
            if neighbor_index not in cost_so_far or new_cost < cost_so_far[neighbor_index]:
                #get the list of heuristics from the list of heuristic functions
                heuristics_list = [heuristic_func(goal, neighbor, graph) for heuristic_func in heuristic_funcs]
                #then, update the priority based on the maxmimum of the heuristics
                neighbor_twoD_index = graph.getIndex2D(neighbor)
                priority = new_cost +  precompute_class.asimilate_heuristics(neighbor_twoD_index, goal, neighbor, graph, heuristics_list)
                
                #update other useful information for extracting path, cost and expanded nodes
                cost_so_far[neighbor_index] = new_cost 
                frontier.put(neighbor, priority)
                came_from[neighbor_index] = current
                raw_pos[neighbor_index] = neighbor

    
    print("Number of times in loop: {}".format(num_loops))

    return came_from, cost_so_far, near_goal, raw_pos 


#optimising the version above, adhering to the papers
#NOTE: EXPLICIT CLOSED LIST WAS NOT MAINTAINED. THE KEYS IN came_from, cost_so_far AND OTHERS REPRESENT ELEMENTS BOTH
#IN THE OPEN AND CLOSED LIST. FOR CONSISTENT ADMISSIBLE HEURISTIC, ITEMS IN CLOSED LIST NEEDS NOT BE REOPENED 
def a_star_search_optimise(graph, heuristic_funcs, travel_func, stop_search_criteria, start, goal, precompute_class = BasePrecompute()):
    #use priority queue for popping off nodes
    frontier = PriorityQueue()
    
    #putting the starting node
    start_index = graph.getIndexAll(start)
    frontier.put(start, 0, start_index)

    #create dictionary of keys for the cost and the nodes' parents
    
    came_from = OrderedDict()
    came_from[start_index] = None
    
    raw_pos = {}
    raw_pos[start_index] = start 
    
    cost_so_far = {}
    cost_so_far[start_index] = 0
    
    closed_list = {}
    
    #near goal, if never reached then set to None
    near_goal = None
    
    #for counting how many times were spent in the loop
    num_loops = 0
    num_expanded = 0
 
    #just runs the precompute method of the precompute class
    precompute_class.precompute(goal, start, graph)
    
    #iterate through the openlist until its empty 
    while not frontier.empty():
        num_loops+=1 
        
        #pop the lowest cost node from the queue
        current = frontier.get()
        current_index = graph.getIndexAll(current)
        
        #add current to closed list
        closed_list[current_index] = current

        #early exit
        if stop_search_criteria(current, goal, graph):
            near_goal = current
            break

        #iterate through possible actions
        for action in graph.actions:
            neighbor_index, neighbor = graph.get_neighbor(current, action)
            #if occuluded, not within bound or already in closed list then move on
            if neighbor_index is None or neighbor_index in closed_list:
                continue
            
            #compute new cost
            new_cost = cost_so_far[current_index] + travel_func(current, neighbor)
            
            #check if node is present in open list
            in_open = neighbor_index in cost_so_far 

            #if the node is not present in open list  or lower cost can be achieved
            if not in_open or new_cost < cost_so_far[neighbor_index]:
                #update other useful information for extracting path, cost and expanded nodes
                cost_so_far[neighbor_index] = new_cost
                came_from[neighbor_index] = current
                raw_pos[neighbor_index] = neighbor
                
                #get the list of heuristics from the list of heuristic functions
                heuristics_list = [heuristic_func(goal, neighbor, graph) for heuristic_func in heuristic_funcs]
                #append the list with the result from precomputation
                neighbor_twoD_index = graph.getIndex2D(neighbor)
                #heuristics_list.append(precompute_class.get(neighbor_twoD_index, goal, neighbor, graph))
                #then, update the priority based on the maxmimum of the heuristics
                #priority = new_cost + max(heuristics_list)
                priority = new_cost +  precompute_class.asimilate_heuristics(neighbor_twoD_index, goal, neighbor, graph, heuristics_list)

                #if it existed, then remove it first
                if in_open:
                    frontier.remove(neighbor_index)
                
                frontier.put(neighbor, priority, neighbor_index)
                
                num_expanded +=1 
    
    print("Number of times in loop: {}, Number of nodes expanded: {}".format(num_loops, num_expanded))


    return came_from, cost_so_far, near_goal, raw_pos 



#function to return the path given the came_from dictionary
def get_path(graph, came_from, near_goal, start_pos):
    c_node = near_goal
    path = []
    if near_goal and graph.getIndexAll(c_node) in came_from:
        path.append(c_node)
        while c_node != start_pos:
            #connect node to its parents
            c_node = came_from[graph.getIndexAll(c_node)]
            path.append(c_node)


    return path 


