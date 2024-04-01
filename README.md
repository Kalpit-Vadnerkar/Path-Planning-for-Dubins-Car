A simple implementation of RRT, RRT*, and Hybrid A* path planning algorithms for Dubins car in a 2D environment with obstacles. 

## Overview of the Project

The objective is to plan the motion of a car in 2D environment that is not confined to the roads,i.e an off road environment. The car is considered to be a Dubins Car which is subject to non-holonomic constraints where RRT* algorithm is implemented for path planning, which can effectively generate paths to navigate through the 2D environment.

![alt text](https://github.com/Kalpit-Vadnerkar/Path-Planning-for-Dubins-Car/blob/master/Picture1.png?raw=true)

Dubins car is a type of vehicle used in motion planning that is constrained to move along circular arcs with a fixed turning radius. The Dubins car model can be used to design path planning algorithms for autonomous vehicles, where the maximum steering angle imposes a minimum turning radius on the vehicle.

## Comparison between RRT and RRT* exploration

![alt text](https://github.com/Kalpit-Vadnerkar/Path-Planning-for-Dubins-Car/blob/master/RRT.png?raw=true)          ![alt text](https://github.com/Kalpit-Vadnerkar/Path-Planning-for-Dubins-Car/blob/master/RRT_Star.png?raw=true)

## Usage

# pathfinding with RRT* + Dubins Path
$ python rrt_star.py

# Map exploration for Dubins car using RRT* algorithm
$ python rrt_star_exploration.py

# pathfinding with RRT + Dubins Path
$ python rrt.py

# Map exploration for Dubins car using RRT algorithm
$ python rrt_exploration.py

# pathfinding with Hybrid A* + Dubins Path
$ python hybrid_astar.py

## Changing the Map
The project uses cases.py in the "Path-Planning-for-Dubins-Car\dpp\test_cases" folder. Rename the files to "cases.py" to use a different map.

## Conclusion
In conclusion, the RRT* path finding algorithm offers a great solution for motion planning in complex environments with obstacles. However, it is important to note that randomness plays a crucial role in the algorithm's performance, and removing the seed value can result in variations in the path. Additionally, running the algorithm with the same input can lead to different outputs, which has a significant effect on the vehicle's path. Despite these limitations, the RRT* algorithm offers an effective way to find optimized paths. It is essential to consider the expense of computation and time required to achieve the optimal path, as these factors can impact the overall performance of the algorithm.

