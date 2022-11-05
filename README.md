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

The main features of this implementation include the bility to:
- perform search with another custom costmap type by:
  - creating a wrapper class inheriting from the `Base_Graph` abstract class
  - writing cost, heuristics, stopping criteria and kinematic model functions specific to the custom costmap type (A* class accepts templated function pointers to these functions)
- supply multiple heuristic function pointers in a vector to influence the behaviour of the search algorithm. For example, when no heuristics are provided, the algorithm reverts to Dijkstra's.
- stop the search early via stopping criteria. This is quite useful for faster planning when reaching the exact goal isn't as important such as when planning to an intermediate goal in one cycle of local planning.
- use expanded nodes from another instance of A* as heuristics for the current search, i.e. using result from the relaxed planning graph in lower-dimensional problem as a better informed heuristic in higher-dimensional planning.
- perform analyical expansion via Dubin's or ReedsShepp's path to goal at increase frequency as the node gets closer to the goal. This increases the chance of successful planning in reaching the exact goal and speeds up the search.

## Citation
This package has been used to produce results in the following papers:
- Narula, K., Worrall, S., & Nebot, E. (2020, September). Two-level hierarchical planning in a known semi-structured environment. In 2020 IEEE 23rd International Conference on Intelligent Transportation Systems (ITSC) (pp. 1-6). IEEE.
```
@inproceedings{narula2020two,
  title={Two-level hierarchical planning in a known semi-structured environment},
  author={Narula, Karan and Worrall, Stewart and Nebot, Eduardo},
  booktitle={2020 IEEE 23rd International Conference on Intelligent Transportation Systems (ITSC)},
  pages={1--6},
  year={2020},
  organization={IEEE}
}
```

- Shan, M., Narula, K., Wong, Y. F., Worrall, S., Khan, M., Alexander, P., & Nebot, E. (2020). Demonstrations of cooperative perception: Safety and robustness in connected and automated vehicle operations. Sensors, 21(1), 200.
```
@article{shan2020demonstrations,
  title={Demonstrations of cooperative perception: Safety and robustness in connected and automated vehicle operations},
  author={Shan, Mao and Narula, Karan and Wong, Yung Fei and Worrall, Stewart and Khan, Malik and Alexander, Paul and Nebot, Eduardo},
  journal={Sensors},
  volume={21},
  number={1},
  pages={200},
  year={2020},
  publisher={MDPI}
}
```

- Shan, M., Narula, K., Worrall, S., Wong, Y. F., Perez, J. S. B., Gray, P., & Nebot, E. (2022). A Novel Probabilistic V2X Data Fusion Framework for Cooperative Perception. arXiv preprint arXiv:2203.16964.
```
@article{shan2022novel,
  title={A Novel Probabilistic V2X Data Fusion Framework for Cooperative Perception},
  author={Shan, Mao and Narula, Karan and Worrall, Stewart and Wong, Yung Fei and Perez, Julie Stephany Berrio and Gray, Paul and Nebot, Eduardo},
  journal={arXiv preprint arXiv:2203.16964},
  year={2022}
}
```
