#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_gt_publisher");
	ros::NodeHandle n;
    ros::NodeHandle pnh("~");
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener transformListener_(tf_buffer_);


	// initial position
	double x = 0.0; 
	double y = 0.0;
	double z = 0.0;
	double th = 0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	// message declarations
	std::string odomFrame = "odom";
	std::string baseFrame = "base_link";

	
	pnh.getParam("odomFrame", odomFrame);
	pnh.getParam("baseFrame", baseFrame);

	std::cout << "Using Frames:" << std::endl;
	std::cout << "Odom Frame: " << odomFrame << std::endl;
	std::cout << "Base Frame: " << baseFrame << std::endl;


	while (ros::ok()) {
		geometry_msgs::TransformStamped transformTf;
        ros::Time timeStamp;
        timeStamp.fromSec(0.0);
        try
        {
            transformTf = tf_buffer_.lookupTransform(odomFrame, baseFrame, timeStamp, ros::Duration(5.0));
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            std::cerr << "Failed to get a transform" << std::endl;
            continue;
        }
        float x_coord = transformTf.transform.translation.x;
        float y_coord = transformTf.transform.translation.y;
        float z_coord = transformTf.transform.translation.z;
		ros::Time current_time = transformTf.header.stamp;

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = odomFrame;
		odom.child_frame_id = baseFrame;

		// position
		odom.pose.pose.position.x = x_coord;
		odom.pose.pose.position.y = y_coord;
		odom.pose.pose.position.z = z_coord;
		odom.pose.pose.orientation = transformTf.transform.rotation;

		//velocity
		odom.twist.twist.linear.x = 0.0;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = 0.0;

		last_time = current_time;

		odom_pub.publish(odom);

		loop_rate.sleep();
	}
	return 0;
}