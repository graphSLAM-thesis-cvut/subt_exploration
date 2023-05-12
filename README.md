# subt exploration
Subterranean exploration project in ROS
![logo](https://github.com/graphSLAM-thesis-cvut/subt_exploration/blob/main/media/logo.jpg)
# Dependencies
## For simulation
You need to have Nvidia VGA to be able to run simulation <br> 
if you run the simulator from docker, you need to install docker: <br>
https://docs.docker.com/engine/install/ubuntu/ <br>
Also go through post-installation steps to be able to run docker without sudo: <br>
https://docs.docker.com/engine/install/linux-postinstall/ <br>
Install NVIDIA docker: <br>
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

## Other dependencies:
Install other dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```

## To install simulation:
```
./build_docker.sh
```

# To run simulation and elevation/traversability mapping with gt localization
Before running, you need to install the package with `catkin build` <br>
Afterwards, run:
```
roscore
rosparam set use_sim_time true
roslaunch subt_exploration simulation_with_gt.launch
roslaunch subt_exploration my_elevation.launch
roslaunch subt_exploration rviz.launch
```
Note that the position is retrieved from the odometry for now. It is needed to install a simulation and run it from docker for everything to work properly. <br/>
The parameters such as topics, for elevation map, traversability and planning can be found and adjusted in `config/my_mapping/my_mapping.yaml`

# To run simulation and elevation/traversability mapping with liorf localization
module `thesis` is needed <br>
Before running, you need to install the package with `catkin build` <br>
Afterwards, run:
```
roscore
rosparam set use_sim_time true
roslaunch subt_exploration simulation.launch
roslaunch thesis liorf_subt.launch
roslaunch subt_exploration my_elevation.launch
roslaunch subt_exploration rviz.launch
```
(doesn't work right now for some reason)
Note that the position is retrieved from the odometry for now. It is needed to install a simulation and run it from docker for everything to work properly. <br/>
The parameters such as topics, for elevation map, traversability and planning can be found and adjusted in `config/my_mapping/my_mapping.yaml`

# To see how the exploration stack works:
```
rosservice call /explore_once
```
if all the frontiers are already planned to in the past, you can clean the memory using:
```
rosservice call /clean_visited
```
On `/explore_once` service, the node will detect the nearest frontier and plan towards it, and publish a plan. The path following should be done by some other node

# For path following
For now, just ask Seva for a tracker package, because it's not uploaded anywhere. You can always use your tracker.
With Seva's version of the package:
```
roslaunch rds_path_tracker_base test.launch
```