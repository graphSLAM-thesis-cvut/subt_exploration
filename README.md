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
## For exploration stack
Install other dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```

## To run simulation:
```
roscore
rosparam set use_sim_time true
roslaunch subt_exploration simulation_with_gt.launch
roslaunch subt_exploration my_elevation.launch
roslaunch subt_exploration rviz.launch
```

## To see how the exploration stack works:
```
rosservice call /explore_once
```
if all the frintiers are already planned to, you can clean the memory using:
```
rosservice call /clean_visited
```

## For path following
For now just ask Seva for a tracker package, cause it's not uploaded anywhere. You can always use your tracker.