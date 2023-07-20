# RP-Project_Robstacles
Our project consists of implementing a planner that follows an A* algorithm that allows us to perform a safe path; and a path follower that with the information provided by the planner allows us to reach the destination.
This project is executed for the robot programming course, held by Professor Giorgio Grisetti. 
Following the instructions given to us by our professor, the project will be structured in the following phases: 
- Implementation of the A* algorithm in C++;
- Implementation of an RVIZ interface, where the path is published and visualized accordingly;
- Once the path is visualized, a rigid body follows it at a specific rate, starting from the initial to the final configuration.

### How to make the repository work
To make sure everything is working properly make sure you have Ubuntu 20.04 with
ROS Noetic. Install catkin_tools, create a catkin workspace ('{name_of_the_workspace}' folder) 
```bash
cd {name_of_the_workspace}
mkdir src
```
Clone this
repository in the `src` folder. Make sure you are compiling in *Release* mode
by properly setting your catkin workspace:
```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Build your code by running the following command:
```bash
catkin build
```
Among the files of this repository, 'path_planning/' and 'path_follower/' are the main ROS packages. These must be copied in 'src/'.  
After this passage repeat  
```bash
catkin build
```
To play with the ROS A* planner run this command in '{name_of_the_workspace}':  
```bash
roslaunch path_planning astar.launch
```
Otherthan the execution of the ROS nodes, there will be open an rviz environment.  
The following topic must be selected:  
* **map**
* **Path**
* **RobotModel**
* **initialpose**
* **move_base_simple/goal**

The messages for the last two topics are published directly from the RVIZ user interface. The path is drawn on the map, and the 'RobotModel' starts following it at a constant rate.  
Our implementation allows to change **initialpose**, and **move_base_simple/goal** in real time, in order to have the robot starting to follow a new path, without waiting for the completion of the previous one.



### Project Contributors

- Andrea Giuseppe Di Francesco;
- Vincenzo Vitale
