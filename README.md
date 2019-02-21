# PathPlanning-AStar-ROS

The objective of this project is to plan a path for a robot from a given starting point to a destination. A* planning algorithm is used to find a route from a default start point to a default goal. 

The heuristic that is used for the estimated cost between the current node and the goal, is the Euclidean distance between the current location and the goal. Once planned, the robot is commanded to execute the plan to go from this start to goal.
