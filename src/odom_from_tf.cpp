#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_gt_publisher");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_gt", 10);
	tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener transformListener_(tf_buffer_);


	// initial position
	double x = 0.0; 
	double y = 0.0;
	double z = 0.0;
	double th = 0;

	// velocity
	double vx = 0.4;
	double vy = 0.0;
	double vth = 0.4;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	// const double degree = M_PI/180;

	// message declarations

	nav_msgs::Odometry prev_odom;

	bool init = true;

	while (ros::ok()) {
		current_time = ros::Time::now(); 

		geometry_msgs::TransformStamped transformTf;
        ros::Time timeStamp;
        timeStamp.fromSec(0.0);
        try
        {
            transformTf = tf_buffer_.lookupTransform("COSTAR_HUSKY/odom", "COSTAR_HUSKY", timeStamp, ros::Duration(5.0));
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

		// double dt = (current_time - last_time).toSec();
		// double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		// double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		// double delta_th = vth * dt;

		// x += delta_x;
		// y += delta_y;
		// th += delta_th;

		// geometry_msgs::Quaternion odom_quat;	
		// odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		// odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "COSTAR_HUSKY/odom";
		odom.child_frame_id = "COSTAR_HUSKY";

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

		// publishing the odometry and the new tf
		// broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);

		loop_rate.sleep();
	}
	return 0;
}