# hybrid_a_star
Here's my implementation of the Hybrid A* algorithm. The rough structure of the repository is as follows:
- Python implementation of the algorithm can be found in the `/scripts/` folder specifically in `hybrid_a_star.py`. The algorithm can be tested and visualised with `trial.anim` by configuring `scenarios.yaml` file (use `--help` function with the script to see the options) or looking at the Jupyter notebook `Test bed.ipynb`.
- The following is a high-level structure of the header files for the C++ implementation:
  -  `/include/hybrid_a_star/parameters.h`: Contains the collection of parameters organised into structures that are used in this repository including the robot's position, graph, vehicle's dimensions, vehicle's constraints, thresholds for stopping criteria, motion penalties, and planner.
  - `/include/hybrid_a_star/utils.h`: Contains utility functions in distinct namespaces to be supplied to A* algorithm for calculating heuristics, calculating costs, expanding nodes (kinematic models), and checking when goal is reached (stopping criteria).
  - `/include/hybrid_a_star/base_graph.h`: Contains an abstract class with template cost map type to be inherited and implemented as a graph structure used for search by the A* algorithm.
  - `/include/hybrid_a_star/grid_map_wrapper.h`: Contains a derived class from `Base_Graph` in `base_graph.h` with [grid map](http://wiki.ros.org/grid_map) as the cost map. The class allows for planning only in 2D (x and y coordinates), i.e. typical A* and Dijkstra's. 
  - `/include/hybrid_a_star/grid_map_wrapper3d.h`: Contains a derived class from `Grid_Map_Wrapper` in `grid_map_wrapper.h` but also considering vehicle's heading.
  - `/include/hybrid_a_star/smoother.h`: Contains an implementation of path smoother based on obstacle term, curvature term and smoothness term using gradient descent.
  - `/include/hybrid_a_star/a_star.h`: Contains the implemented search algorithm with configurable behaviour based on the graph structure, parameters and supplied function pointers (kinematic models, heuristics, costs, and stopping criteria).
- Visualisation and test can be found in `/src/test_bed.cpp` and `/test/test_gridmap_move.cpp`. The test bed can be launched via `/launch/test_bed.launch` after compilation and configured via `/config/test_bed.yaml` and `/config/scenarios.yaml`.
