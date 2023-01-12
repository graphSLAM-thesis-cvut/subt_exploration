#!/usr/bin/bash
rosparam set use_sim_time true
rosbag play -l points.bag
