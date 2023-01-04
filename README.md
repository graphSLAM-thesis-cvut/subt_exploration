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
## For move_base:
```
sudo apt install
```
## For exploration:
```
git clone git@github.com:rohithjayarajan/frontier_exploration.git
```