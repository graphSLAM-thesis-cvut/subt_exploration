// Copyright (c) 2021, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static std::string topic = "/COSTAR_HUSKY/odom";
static std::string parent_frame = "/COSTAR_HUSKY/odom";
static std::string child_frame = "/COSTAR_HUSKY";

void odom_callback(const nav_msgs::OdometryConstPtr& odom){
  static tf::TransformBroadcaster br;
  tf::Transform tf;
  geometry_msgs::Pose odom_pose = odom->pose.pose;

//  tf.setOrigin(tf::Vector3(odom_pose.position.x, odom_pose.position.y, odom_pose.position.z));
//  tf::Quaternion quat;
//  tf::quaternionMsgToTF(odom_pose.orientation, quat);
//  tf.setRotation(quat);
  tf::poseMsgToTF(odom_pose, tf);

  tf::StampedTransform stamped_tf(tf, odom->header.stamp, parent_frame, child_frame);

  br.sendTransform(stamped_tf);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "odom2tf");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("odom_topic", topic);
  private_nh.getParam("parent_frame", parent_frame);
  private_nh.getParam("child_frame", child_frame);

  std::cout << "Topic: " << topic << std::endl;
  std::cout << "Parent frame: " << parent_frame << std::endl;
  std::cout << "Child frame: " << child_frame << std::endl;

  ros::Subscriber odom_sub = nh.subscribe(topic, 10, odom_callback);
  ros::spin();

  return 0;
}