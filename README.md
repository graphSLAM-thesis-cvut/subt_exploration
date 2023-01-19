# subt exploration
Subterranean exploration project in ROS
# Dependencies
## For simulation
You need to have Nvidia VGA to be able to run simulation <br> 
if you run simulator from docker, you need to install docker: <br>
https://docs.docker.com/engine/install/ubuntu/ <br>
Also go through post-installation steps to be able to run docker without sudo: <br>
https://docs.docker.com/engine/install/linux-postinstall/ <br>
Install nvidia docker: <br>
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

# To run simulation and elevation/traversability mapping
Before runniing, you need to install the package with `catkin build` <br/>
Afterwards, run:
```
roscore
rosparam set use_sim_time true
roslaunch subt_exploration simulation_with_gt.launch
rosrun subt_exploration odom_from_tf
roslaunch subt_exploration my_elevation.launch
roslaunch subt_exploration rviz.launch
```
Note that the position is retrieved from the odometry for now. It is needed to install a simulation and run it from docker for everything to work propperly. <br/>
The parameters such as topics, for elevation map, traversability and palnning can be found and adjusted in `config/my_mapping/my_mapping.yaml`

# To see how the exploration stack works:
```
rosservice call /explore_once
```
if all the frintiers are already planned to, you can clean the memory using:
```
rosservice call /clean_visited
```
On `/explore_once` service, the node will detect the nearenst frontier and plan towards it, and publish a plan. The path following should be done by some other node

# For path following
For now, just ask Seva for a tracker package, cause it's not uploaded anywhere. You can always use your tracker.