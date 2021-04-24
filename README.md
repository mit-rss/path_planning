

| Deliverable                        | Due Date |
|------------------------------------|-------------------------------------|
| Briefing (8 min presentation + 3 min Q&A) (slides on  [github pages](https://github.mit.edu/rss/website2021))                      | Monday, April 26 at 1:00PM EDT |
| Gradescope Code Submission         | Friday, April 30 at 11:59PM EDT |
| Report (on [team github pages website](https://github.mit.edu/rss/website2021))                         | Friday, April 30 at 11:59PM EDT     |
| Pushed Code                        | Friday, April 30 at 11:59PM EDT     |
| [Team Member Assessment](https://forms.gle/pqzn9hnBvi791FELA)             | Friday, April 30 at 11:59PM EDT     |

# Lab 6: Path Planning

Table of Contents

* [Introduction](https://github.com/mit-rss/path_planning#introduction)
* [Grading](https://github.com/mit-rss/path_planning#introduction)
  * [Gradescope Evaluation](https://github.com/mit-rss/path_planning#gradescope-evaluation-10-points) (10 points)
  * [Briefing Evaluation](https://github.com/mit-rss/path_planning#briefing-evaluation-see-technical-briefing-rubric-for-grading-details) (see technical briefing rubric for grading details)
  * [Report Evaluation](https://github.com/mit-rss/path_planning#report-evaluation-see-technical-report-rubric-for-grading-details) (see technical report rubric for grading details)
* [Submission](https://github.com/mit-rss/path_planning#submission)
* [Logistics and Setup](https://github.com/mit-rss/path_planning#logistics-and-setup)
* [Part A: Path Planning](https://github.com/mit-rss/path_planning#part-a-path-planning)
  * [Search-based Path Planning](https://github.com/mit-rss/path_planning#search-based-planning)
  * [Sampling-based Path Planning](https://github.com/mit-rss/path_planning#sampling-based-planning)
  * [Tips and Tricks](https://github.com/mit-rss/path_planning#tips-and-tricks)
    * Search Domains
    * Grid Space
    * Circle Space
    * Morphological Dilations/Erosions
    * Motion Heuristics
* [Part B: Pure Pursuit](https://github.com/mit-rss/path_planning#tips-and-tricks)
  * [Tips and Tricks](https://github.com/mit-rss/path_planning#tips-and-tricks-1)
  * [Pure Pursuit Trajectory Utilities](https://github.com/mit-rss/path_planning#trajectory-utilities)
* [Part C: Integration](https://github.com/mit-rss/path_planning#part-c-integration)
* [Part D: TESSE Deployment](https://github.com/mit-rss/path_planning#part-d-tesse-deployment)

## Introduction
Now that you are able to localize your car in the TESSE simulator, it is time to learn how to drive. This laboratory exercise involves two core parts of autonomous operation: planning and control. In other words, given a destination, you will determine the path to the destination and proceed to drive along the path.  

This lab has the following objectives:

- **Part A:** Plan trajectories in a known occupancy grid map from the car’s current position to a goal pose using either a search-based or sampling-based motion planning method.

- **Part B:** Program the car to follow a predefined trajectory in a known occupancy grid map using your particle filter and pure pursuit control.

- **Part C:** Combine the above two goals to enable real-time path planning and execution in the simple `racecar_simulator`. For this part, you will be required also to localize the car by running the particle filter and subscribing to the estimate on `/pf/pose/odom` rather than the ground-truth pose.

- **Part D:** Demonstrate the deployment of your path planning and following in the TESSE simulator. Note: you will use the ground truth pose in TESSE for this section (rather than running localization) in order to lessen variability caused by differing machine specs.

You will have **one and a half** weeks to complete this lab (longer for the report); you should start early! This lab has multiple parts, and furthermore, a simple implementation of a path planning algorithm may not suffice - you are expected to optimize your algorithms. This will take time! 

We are encouraging parallelization by breaking up the components of the lab into distinct parts. Parts A and B can be implemented separately, and then integrated together once they are working individually.

Looking ahead, there will be a final challenge where you will need to be able to quickly and accurately track your planned (and optimized) trajectories. Your team will likely be using the components from this lab with a couple of modifications and tuning, so it is to your advantage to write good algorithms for this lab.
 
## Grading

This section details the grading scheme for Lab 6. 


| Deliverable Grade                        | Weighting |
|------------------------------------|-------------------------------------|
| briefing grade (out of 10)         | 20% |
| report grade (out of 10)           | 40% |
| gradescope submission (out of 10)  | 40% |


### Gradescope Evaluation (10 points)
*We will be making the [leaderboard](https://www.gradescope.com/courses/244521/assignments/1150126/leaderboard) public for this assignment, so teams can see how they stack up!*

In the interest of runtime, each test will be run once per submission. It may take up to 15 minutes for the autograder to evaluate your code. You are free to resubmit as many times as you would like before the deadline.

#### Part A: Path Planning (3 points)
*Initial Condition*: The car will be placed at a set pose in the Stata basement map and given a goal pose.

You will receive full credit if your path remains within `delta_path` of the entire TA solution path.

You will get 0 points if:
- A path is not found within `plan_time_thresh`
- Your path enters occluded space on the map
- Your cumulative distance from the TA path is greater than `delta_path_max`*`path_length`

#### Part B: Pure Pursuit (3 points)
*Initial Condition*: The car will be placed at the start point of a loaded trajectory (`path_planning/trajectories/loop2.traj`) in the Stata basement map with the submitted particle filter running.

Your score will be determined by what percentage of the given path you are able to follow before:
- Exceeding `pursuit_time_thresh`
- Entering occluded space on the map
- Driving further than `delta_pursuit` from the given path

#### Part C: Integration (4 points)

*Initial Condition*: The car will be placed at a set pose in the stata basement map and given a goal pose with the submitted particle filter running.

Grading will be determined as follows:
- The path planning score will be calculated the same as Part A, with a maximum score of 1.5 points.
- The pure pursuit score will be calculated the same as Part B with a maximum score of 1.5 points.
- If you receive a score greater than 2.5 from the path planning and pure pursuit, you are awarded another 1 point (with a maximum of 4 total points).

*Gradescope Parameters:*

| Name | Value |
|----|----|
| `delta_path` | 1.0 m |
| `delta_path_max` | 2.0 m |
| `delta_pursuit` | 1.0 m |
| `plan_time_thresh` | 120 sec |
| `pursuit_time_thresh` | 500 sec |


### Briefing Evaluation (see [technical briefing rubric](https://docs.google.com/document/d/1NmqQP7n1omI9bIshF1Y-MP70gfDkgEeoMjpWv8hjfsY/edit?usp=sharing) for grading details)
When grading the Technical approach and Experimental evaluation portions of your briefing, we will be looking specifically for **illustrative videos of your car planning and tracking trajectories.** Specifically, we would like videos highlighting:
- Start and end point markers (see Trajectory Utilities)
- Visualization of the planned paths (see Trajectory Utilities) from implemented search-based or sample-based planning algorithms
- Visualization of the car following the trajectories
- Deployment behavior of the system in TESSE

### Report Evaluation (see [technical report rubric](https://docs.google.com/document/d/1B6l7vKJFN3CPPcMn8cKKArHUU_Bq_YUZ5KxKoP6qMk0/edit?usp=sharing) for grading details)
When grading the Technical approach and Experimental evaluation portions of your report, we will be looking specifically for the following items:

- **Numerical evidence that your algorithm(s) work in the form of charts/data**
  - Numerical evaluation of the success of your planning algorithm
  - Numerical evidence evaluating the success of your pure pursuit algorithm for tracking hand-drawn and planned trajectories
  - Make sure you mention your method for tuning the controller to closely track trajectories. (Hint: include error plots from `rqt_plot`)
  - A discussion of any shortcomings of your integrated approach. For example, does your pursuit algorithm consistently do poorly in certain areas? Did you need to adjust your path planning algorithm to succeed in the TESSE map?
- **Detailed comparison of motion planning algorithms**
  - Explain path planning algorithms, and the strengths and weaknesses of sample-based versus search-based methods. Which algorithm should work better for the purposes of planning trajectories for your car? What different cost functions would you use? How would you ensure the car would not choose paths close to the wall?
    - Specifically, please discuss the following properties of planning algorithms, along with any others you considered: asymptotic optimality, single- vs multi-query, incorporating dynamics, complexity, and necessity of search after construction. Note that some of these are only applicable to search-based planners and some are only applicable to sampling-based planners.
  - **(Bonus +3 points):** Implement both a sample-based and a search-based path planning algorithm, and numerically compare the strengths and weaknesses of each. Demonstrate your pure pursuit controller on paths generated by ONE algorithm.

## Gradescope Submission
You must submit your localization and path_planning packages together for **Part C** of the lab (do not submit your implementation for TESSE). To ensure that your submission can be built and executed properly in the environment on the autograder, it is important you pay attention to the following:
- **Submission format:** A `.zip` archive of **your catkin workspace’s `/src` directory** containing ONLY the `/localization` and `/path_planning directories`. If you get a ‘server error’ on Gradescope, your submission may be too large. Try deleting the `.git` directories in your packages.
- **ROS package:** The nodes implementing your particle filter, path planner, and pure pursuit algorithm must be called `particle_filter.py`, `path_planning.py`, and `pure_pursuit.py`. Also be sure that `trajectory_loader.py` is in your `/path_planning/src` directory, as we will use it to test your pure pursuit algorithm.
- **Node parameters:** Our Gradescope evaluation is only able to see the ROS params that the template code comes with. If you add more parameters (and fail to give them default values), we will not be able to set them in the Gradescope evaluation, and your tests will fail!
- **Topics:** The `/trajectory/current` topic will be used to evaluate the path found during path planning, and to send a path to the pure pursuit algorithm. The template code should already be set up this way, but just be aware.


Apart from the usual ROS packages like `rospy` and `tf2`, the following Python packages will be installed in the autograder environment:
- `scan_simulator_2d`
- `numpy`
- `scipy`
- `matplotlib`

Feel free to use these, but if you depend on other packages, be aware that your code will not run. Please let the staff know if there are any other packages you would like to see included. Please also keep in mind that the autograder will be running a stock installation of ROS Melodic on Ubuntu 18.04. Any hacks or modifications you may have performed on your personal installation of ROS will not be present in the autograder environment.

## Logistics and Setup
Fork the skeleton code from this repository (https://github.com/mit-rss/path_planning_tesse). Also, please pull the latest version of tesse-ros-bridge (https://github.mit.edu/rss/tesse-ros-bridge).

Each node that needs to be implemented has a template python file and launch file. Each node has parameters set in the launch file and defined in the node code. If you add additional ROS parameters to your ROS nodes, be sure to give them default values. Our Gradescope evaluation is only able to provide the parameters that the template code comes with. If you add more parameters (or fail to give them default values), we will not be able to set them in the Gradescope evaluation and your tests will fail!

The RViz buttons are set up to publish to the following topics:
- “2D Nav Goal” → `/move_base_simple/goal`
- “2D Pose Estimate” → `/initialpose`
- “Publish Point” → `/clicked_point`

NOTE: The path_planning ROS package is called “`lab6`”, so all nodes should be launched/run with this name (eg. `roslaunch lab6 build_trajectory.launch`).

**Map selection**: In this lab, you are asked to plan paths in the Stata Basement environment. By default, running `roslaunch racecar_simulator simulate.launch` will load the old wall-following environment from lab 2 and 3. To change the map used by the `racecar_simulator`, follow these steps:

1.  Open [~/racecar_ws/src/racecar_simulator/launch/simulate.launch](https://github.com/mit-racecar/racecar_simulator/blob/master/launch/simulate.launch). Line 6 reads: `<arg name="map" default="$(find racecar_simulator)/maps/building_31.yaml"/>`
2.  Change line 6 to: `<arg name="map" default="$(find racecar_simulator)/maps/stata_basement.yaml"/>`
3.  Alternatively, you can use this syntax to load a map of your choice in your own custom launchfiles.

## Part A: Path Planning
In this section, you will plan trajectories and display them in RViz. You will use RViz to publish a 2D pose specifying goal positions in your map. Then, your path planning algorithm will construct a collision free trajectory. To ease integration, make sure your inputs and outputs align between your path planning and pure pursuit modules.

**Path Planning Requirements** (`path_planning/src/path_planning.py`)

- This node must use the car’s current position as the starting point for the path planner. When testing and developing, feel free to use the ground truth pose of the car published to “/odom”. Remember, you can move the car around the map using the “2D Pose Estimate” button in RViz.
- The goal position must be set using the “2D Nav Goal” button in RViz.
- Create a simple path planner that will find collision-free paths in the current map. The code is set up to handle paths as `geometry_msgs/PoseArray` messages.
Once a trajectory is generated, you can visualize the path, start, and goal positions on the “`\planned_trajectory\path`”, “`\planned_trajectory\start_point`”, and “`\planned_trajectory\end_pose`” topics in RViz.
- (OPTIONAL) Implement both a sample-based and search-based planning algorithm.

The planner should take as input a map, start location, and goal location. The output of the planner should be a trajectory. The trajectory should be specified in the map coordinate frame. Your team will decide how to design your trajectory. The choice of trajectory representation is up to you. Make sure that you discuss your trajectory representation as a team when you split up tasks! 

Here are a few options for representation of a trajectory:

- Piecewise - (x,y,[theta]) points
- Spline curve
- Clothoid curves

Note that Pure Pursuit can follow rough trajectories, so don’t worry too much if your planned trajectories are not particularly smooth. However, smoother trajectories will perform better overall. 

The biggest difference between search-based planning and sampling-based planning are speed and optimality: search-based methods can guarantee optimal solutions, but usually does so at the expense of computation time.

### Search-based Planning

Search-based planning is a motion planning problem which uses graph search methods to compute paths or trajectories over a discrete representation of the problem. Because a graph is inherently discrete, all graph-search algorithms require a discrete representation.  Search-based planning can then be seen as two problems: how to turn the problem into a graph, and how to search the graph to find the best solution. See the tips and tricks section for some pointers on how to discretize your search space.

A complete solution to the path planning problem will ensure that the best solution can be found, and that the algorithm can detect when no solution exists. Search based algorithms are examples of complete algorithms. 

Some search-based planning algorithms are:
- A*
- Djikstra’s
- BFS/DFS

[This source](https://www.redblobgames.com/pathfinding/a-star/introduction.html) includes a friendly introduction and comparison between A*, Djikstra’s, and BFS. 

### Sampling-based Planning
Complete solutions, which are what search-based planning algorithms can guarantee, are often infeasible for the real life robotics system or inefficient when the possible state space is large. An example is the case for robots with multiple degrees of freedom such as arms. In practice, most algorithms are only resolution complete as the state-space needs to be somewhat discretized for them to operate (e.g., into a grid) and some solutions might be missed as a function of the resolution of the discretization.

Sampling-based planning methods are able to solve problems in continuous space without a graph representation (though certain sampling-based methods do discretize the space). These planners create possible paths by randomly adding points to a tree until some solution is found or time expires. As the probability to find a path approaches 1 when time goes to infinity, sampling-based path planners are probabilistic complete. Sampling-based planners are fast, but can sometimes result in unusual-looking and possibly inefficient paths.

Optional reading for examples of sampling-based planners:
- [Rapidly-exploring Random Trees (RRT and RRT*)](https://arxiv.org/pdf/1105.1186.pdf)
- [Probabilistic Roadmaps (PRM)](http://www.staff.science.uu.nl/~gerae101/pdf/compare.pdf)
- More information about sampling-based planning can be found [here](http://correll.cs.colorado.edu/?p=2012). 

**Remember, for this lab we are requiring you to implement ONLY ONE search-based algorithm OR sampling-based algorithm. You can choose which explicit algorithm you want to implement. Optionally, you can choose to implement one of each (one search-based AND one sampling-based algorithm), for extra credit.**

### Tips and Tricks
This lab is very open ended. There are many possible correct solutions. We encourage you to get creative, and do what makes sense for your team, but here we provide a few pointers to kick off your research phase. This section contains many links to outside resources; please note that **these are all optional readings** for your team to use as you wish.

Here is a good resource for [dubins curves](https://github.com/AndrewWalker/pydubins). You definitely don’t need to implement these by hand, but make sure you understand how they work if you want to use them for generating your paths!

#### Search Domains

The choice of search domain is very important when implementing search-based algorithms such as A* search. Here we present a few possible selections of search domain:

*Grid Space*: One obvious choice is to use some discretized grid of possible states, similar to what you may have seen in 6.01. The upside of this approach is that the state space is reasonably small in 2D, so the search has a good chance of terminating even with a poor heuristic/cost function. The downside is that the paths:
- Do not consider driving feasibility
- Are potentially made up of many small line segments

[Here](http://movingai.com/astar-var.html) are a few search variants on grid space.

*Circle Space*: Another option is to use a search space which is not confined to a grid. One example is the circle-based method demonstrated in [kinodynamic motion](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7353741), [non-holonomic motion](https://mediatum.ub.tum.de/doc/1283837/826052.pdf). This approach is interesting because it generates sparse piecewise linear paths which can be made to (at least approximately) honor nonholonomic driving constraints.

The primary problem with using non-grid search spaces is that the algorithm can easily get stuck in dead ends expanding thousands of nodes without making progress towards the goal. The approach in these papers uses a hack which forces the circles to avoid already explored regions of space. This makes it quickly explore, at the expense of optimality guarantees.

#### The Occupancy Map

For each planning environment you consider (Stata Basement and the TESSE city), you will receive information about the location of obstacles in the form of an _occupancy map_ on the `/map` channel. This message will be an [OccupancyGrid](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) with a property `data` that lists the occupancy values of map cells or pixels. In order to check the occupancy of a real world point, you will want to convert it into the pixel coordinate frame and then index to `data` appropriately. Suppose the message received on this channel is called `msg`: to convert from pixel coordinates (u, v) to real coordinates (x, y), you should multiply (u, v) through by `msg.info.resolution`, and then apply the rotation and translation specified by  `msg.info.origin.orientation` and `msg.info.origin.position`. To convert the other way (x, y) -> (u, v), reverse these operations. Note that if you reshape the `data` array into a 2D numpy matrix `grid` of dimension `(msg.info.height, msg.info.width)`, then you should index into it as `grid[v, u]` (indices swapped).

#### Morphological Dilations/Erosions

Search algorithms often tend to cut corners close since they are attempting to minimize distance or time. Sometimes the path it chooses will be collision free in your domain space representation, however, in real life, the path is infeasible for the car because of its dimensions (a car is not a point mass). Additionally, the close-cut corners of the path can be problematic for the pure pursuit controller, which also will attempt to cut corners.  

![Stata Basement Dilated](https://github.com/mit-rss/path_planning/blob/master/media/MorphDilationLab6.jpg)


Provided basement map (left) and eroded map (right). Disk element, 10px radius.

To avoid all of these potentially very bad collisions, one method is to “dilate” the obstacles so that nearby states are considered off-limits to the planning algorithm even though they technically are collision-free. You may choose to check out these possible functions: [disks](http://scikit-image.org/docs/dev/api/skimage.morphology.html?highlight=disk#disk), [dilations](http://scikit-image.org/docs/dev/api/skimage.morphology.html?highlight=dilation#dilation), and [erosions](https://scikit-image.org/docs/dev/api/skimage.morphology.html?highlight=erosion#skimage.morphology.erosion). You can do these processes offline and just use the adjusted map for your planning algorithms. 

#### Motion Heuristics

The most obvious and simplest heuristic is Euclidean distance between pairs of start and end states. However, this can cause issues since it does not consider the vehicle dynamics or any possible collisions in between the start/end states. 

A different possible heuristic is [dubins curves](https://en.wikipedia.org/wiki/Dubins_path), which more accurately estimates nonholonomic motion (other sources: [1](http://planning.cs.uiuc.edu/node821.html), [2](https://pypi.org/project/dubins/), helping make paths more accurate to the vehicle dynamics. Performing a lower dimensional search such as Dijkstra’s algorithm on the grid map can help avoid finding paths that don’t consider obstacles, but this introduces some computational complexity (source: [3](http://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)).

## Part B: Pure Pursuit

In this section, you will first take a manually-defined path and implement a pure pursuit controller to follow it. A sample path is provided in the skeleton code, but you are afforded the freedom to create your own trajectories and, if desired, trajectory representations.

**Pure Pursuit Requirements (`path_planning/src/pure_pursuit.py`)**

1. Manually define or load a saved path. We have provided a framework to create these with clicks on the Rviz map (described below in Trajectory Utilities). See how to do this with the load_trajectry.launch file. You can visualize the loaded trajectory path, start point, and end pose on the “/loaded_trajectory/path”, “/loaded_trajectory/start_point”, “loaded_trajectory/end_pose” topics in RViz.
2. Implement a pure pursuit algorithm.
3. Visualize a simulated car following the path.
4. (OPTIONAL) If you choose to implement two types of path planning algorithms for extra credit, you only need to show your car tracking the trajectory for one of the implemented algorithms. You can choose based on your analysis which planning algorithm and trajectory you want to ultimately use to complete integration, but you need to be able to justify your choice with some quantitative data. 

Once you have a path, the next step is to determine the necessary control to follow that path. In pure pursuit, the primary challenge is to find the lookahead point - the intersection between the circle defined by your lookahead distance, and the path (and handle associated edge cases!). In general this problem has many possible solutions which may result in different behavior in the various edge cases. See the Tips and Tricks section below for one fairly simple method which has proven to work well in practice. 

Since the path’s coordinates are in the map frame, functional localization is a prerequisite to solving this part of the lab (but not the TESSE integration) - you will need to run your team's solution to Lab 5 and subscribe to its output in the simple racecar simulator.

### Tips and Tricks

**Pure Pursuit**

Pure pursuit for trajectory tracking is a tried and true method (if you're interested in learning more, [this](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) and [this](https://www.researchgate.net/publication/319714221_Pathfinder_-_Development_of_Automated_Guided_Vehicle_for_Hospital_Logistics/download) discus its implementation using piecewise linear segments for a trajectory). The proposed method of determining the lookahead point is fairly robust and handles various edge cases nicely - there are two main steps.

- Find the point on the trajectory nearest to the car 
  - Here is a [stack overflow](https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment/1501725#1501725) post that walks through the math of computing this for a single segment.
  - You will need to find the closest point for every trajectory segment on every timestep, so the process should be efficient. Once you have an initial implementation working, try changing it to use vectorized numpy commands to simultaneously compute the nearest point on each segment of your path to the car very efficiently for 100,000s of line segments. Then you can find the numpy argmin function to get the closest point (and segment) for that time step.
- Once you have the closest trajectory segment, you need to find the goal/lookahead point.
  - Find the intersection between the circle around the car (using radius = lookahead distance, circle center = car’s position) and your piecewise linear trajectory that is forwards along the path with respect to the point found in step (1). 
    - [This](https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm/86428#86428) algorithm finds an intersection between a circle and line segment.
  - Your search for the lookahead point that lies on a trajectory segment should start at the segment containing the nearest point from part (1), that way you only have to iterate through a couple of path segments before finding the nearest intersection.

Watch out for different edge cases (for example, when a segment has two intersection points on your circle!!) These edge cases can be discovered intuitively, and should also be apparent in implementing the above steps. 

Like tuning gains for a PID to change the behavior, you may want to change your lookahead distance based on the path you are following to adjust its behavior. This is a common practice, especially if the robot’s speed is also varying. Consider using:
- Long lookaheads while following long, low-curvature sections
- Short lookaheads while following paths with tight curvature

### Trajectory Utilities

We have provided utility functions to help build and load trajectories (piecewise linear segments; see `utils.py` to understand the `LineTrajectory` Class further) so that your team can parallelize and test pure pursuit without relying on trajectory outputs from path planning. These utilities are RViz-based.

*Trajectory Builder*: Run `roslaunch lab6 build_trajectory.launch` to build the trajectory. You can build trajectories using the “Publish Point” button in RViz. The `.traj` file will be saved in the `/trajectories` folder with a timestamp (so they won’t overwrite each other). A trajectory will not be saved/published unless it has >= 3 points. You can visualize this trajectory in RViz under the “`/built_trajectory/path`” topic.

*Trajectory Loader*: Run `roslaunch lab6 load_trajectory.launch` to load the trajectory. This will visualize and publish (to “`/loaded_trajectory/path`” topic, which you can listen to in pure pursuit). A default trajectory for the Stata basement map has been provided in the `/trajectories` folder. Edit the `load_trajectory` launch file to load specific trajectories you built with the trajectory builder. Note, there is some uniqueness to this utility in that it publishes exactly once, so make sure you add the visualization topics and save them after running `load_trajectory` the first time, then for all successive runs you should be able to see the visualization. 

The information published to the “`/loaded_trajectory/path`”, “`/built_trajectory/path`”, “`/planned_trajectory/path`” namespace topics are simply to ease visualization. Your pure pursuit code should follow trajectories published to the “`/trajectory/current`” topic, which takes PoseArray messages. The `LineTrajectory.toPoseArray()` and `LineTrajectory.fromPoseArray()` functions have been provided to allow you to convert between these types.

## Part C: Integration
Once you have completed both path planning and pure pursuit, you should combine them so that you can plan and follow paths in real time. First, test this in RViz simulation: run your particle filter to get the start-point of your paths (`/pf/pose/odom`), and specify the end-point manually with the “2D Nav Goal” button in RViz. You should be able to reliably drive your car around the stata basement by clicking points in RViz to specify trajectories and by planning your own algorithms to get to the goal point on the map. Once you have demonstrated this capability in the simple simulator, it's time to deploy your system in TESSE!

## Part D: TESSE Deployment

### Integration with TESSE

Once you have gotten your pipeline set up and working correctly, it's time to move to TESSE simulation. Use the executable from Lab 4 ([link](https://drive.google.com/drive/u/0/folders/1n7ij3FpRSNwqV4pUSIgsSelsMNCJYk4z)). Also, make sure you've pulled the latest version of tesse-ros-bridge ([link](https://github.mit.edu/rss/tesse-ros-bridge)). Again, **you do not need to run localization in TESSE!!!** You can use the ground-truth pose from the `/tesse/odom` topic (this is set for you in `plan_trajectory_tesse.launch` and `follow_trajectory_tesse.launch`). Remember from lab 5 that you should take care to transform the pose on this topic into the appropriate coordinate frame (reference [Piazza @309](https://piazza.com/class/kkvsdmaisb51g6?cid=309) for an example of such a transformation).

When TESSE is running, a map of the environment is published to the `/map` topic, just as in the simple `racecar_simulator`. However, because it must represent a larger area, the map for TESSE is scaled differently and uses a different coordinate frame than the Stata basement map. In the [maps](https://github.com/mit-rss/path_planning/tree/master/maps) folder of this repository, you may find *copies* of the source `[map_name].yaml` files which define the occupancy grid and scaling parameters for the map of each environment. These parameters should also be accessible via the metadata of messages on `/map`, e.g. the position and rotation of the origin are defined by `map_msg.info.origin`. Pay attention also to the `map_msg.info.resolution` parameter to make sure you are scaling the map correctly!

Unlike the Stata basement, the TESSE environment contains small unmapped obstacles, which could be ignored for localization but may impede an apparently unoccupied path. To help you overcome this, you will also find in this repository an extra map [maps/city_roads.png](https://github.com/mit-rss/path_planning/blob/master/maps/city_roads.png), which marks unoccupied road space, rather than wall locations, in the TESSE environment. You may find this more useful than the built-in `/map` topic for planning collision-free paths. You are welcome to experiment with generating your own modified versions of these maps (see the above section on *Morphological Dilations*).

<div class="row">
<img src="https://github.com/mit-rss/path_planning/blob/master/maps/city.png" width="400">
<img src="https://github.com/mit-rss/path_planning/blob/master/maps/city_roads.png" width="400">
</div>

*(Left: map broadcast to the `/map` topic. Right: map of free road space (`city_roads.png`).)* 


### Experimental Evaluation

Your goal for Part D is to get your path planning and pure pursuit algorithm working in TESSE such that you are able to plan a path from point A to point B, then follow it with your car. How you choose to demonstrate your working code (and the starting and ending points you use) is up to you! For example, you might look to Part A: Path Planning -- [Logistics and Setup](https://github.com/mit-rss/path_planning_tesse/edit/master/README.md) for some information on RViz topics that may be helpful.

Finally, note that the provided visualization tools are not exhaustive. There may be other things that are useful to visualize when developing your planning and pursuit code! Apply the experimental robotics skills you've acquired in previous labs to formulate and report a set of tasks and metrics in TESSE which demonstrate your system's capabilities and limitations to their fullest extent. At a minimum, you should demonstrate that you can plan and follow a path which rounds at least two consecutive corners of the road network. **Start out at a slow speed -- going fast makes the task more challenging.**

For inspiration, here is the route followed during the 2020 RSS final race:

<img src="https://github.com/mit-rss/path_planning/blob/master/media/2020_path.png" width="400">

### Troubleshooting

**Depending on your machine's specs, you may get different behavior on your machine than a teammate does on theirs.** Check the publish rate of your drive commands (you should be publishing to `/tesse/drive`) with `rostopic hz /tesse/drive`. If you are getting a rate of less than 20 hz, your drive commands will not be updating quickly enough-- considider switching to VDI (more info to come), and check in with a TA for help! You would be able to get pure pursuit to run and your car to not crash by tuning waypoints and other parameters, but these parameters will likely need to be different for you vs. the rest of your team. 
