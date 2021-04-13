## Autonomous Ground Vehicle
> ROS Distro: Melodic <br>
**Technologies used : Robot Operating System (ROS), 2D mapping, 3D mapping, Autonomous Navigation, Perception, Robotic Manipulation Pick and Place** <br>
- For SLAM, we made use of gmapping algorithm and made a map in RViz. For localization, we used probabilistic localization system for a robot moving in 2D [amcl](http://wiki.ros.org/amcl) <br>
- ![](https://github.com/Shaxpy/Project-AGV/blob/master/project_img/nav.png =300x300)
- For Navigation, we tuned the **base_local_planner in the move_base package** of ROS taking into account global and local costmaps along with the trajectory planner! [move_base](http://wiki.ros.org/move_base) <br>
- For Perception, we used [find_object_2d](http://wiki.ros.org/find_object_2d) <br>
- ![](https://github.com/Shaxpy/Project-AGV/blob/master/project_img/find_obj.jpg =300x300)
- Pick and Place, we used OMPL motion planning library,we tried many asymptotically optimal planners that we used in MoveIt and ended up optimising with (T-RRT)
- ![](https://github.com/Shaxpy/Project-AGV/blob/master/project_img/place.jpg =300x300)
### Some References while completing this project 
#### BUG Algorithms

http://msl.cs.uiuc.edu/~lavalle/cs497_2001/book/uncertain/node3.html#:~:text=The%20BUG%20algorithms%20make%20the,obstacles%20are%20unknown%20and%20nonconvex.&text=This%20allows%20the%20robot%20to,Euclidean%20distance%20to%20the%20goal

https://arxiv.org/pdf/1808.05050.pdf

https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf

#### Autonomous Nav Algos [Repo Containing Different Algos]

https://github.com/AtsushiSakai/PythonRobotics

#### Obstacle Avoidance and Path Planning for Smart Indoor Agents [PDF]

https://scholarworks.rit.edu/cgi/viewcontent.cgi?article=10672&context=theses

#### Exploring ROS using a 2 Wheeled Robot [YT Playlist - ConstructSIM]

https://www.youtube.com/watch?v=PyC4Vj3NUUY&list=PLK0b4e05LnzY2I4sXWTOA4_82cMh6tL-5&index=5
