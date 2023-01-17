#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include "grid_map_core/GridMapMath.hpp"

// Eigen
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>

#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

#include "my_elevation_mapping.cpp"

#include <std_srvs/Empty.h>
#include "nav_msgs/Path.h"

typedef std::pair<int, int> pairs;
typedef std::pair<float, float> pairf;

using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;

class ElevationMapperRos
{
public:
    ElevationMapperRos(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle) : nodeHandle_(nodeHandle), pnh_(privateNodeHandle)
    {
        tf_buffer_ = new tf2_ros::Buffer();
        transformListener_ = new tf2_ros::TransformListener(*tf_buffer_);
        mapper_ = new ElevationMapper(nodeHandle, privateNodeHandle);
        readPearameters();
        initialize_interface();
    }
    ~ElevationMapperRos()
    {
        nodeHandle_.shutdown();
    }

private:
    ros::NodeHandle nodeHandle_;
    ros::NodeHandle pnh_;

    ElevationMapper *mapper_;

    bool publish_traversability_ = false;

    std::string out_pcl_topic_ = "elevation";
    std::string transformed_pcl_topic_ = "pcl_glob";
    std::string pcl_topic_ = "COSTAR_HUSKY/points";
    std::string map_frame_ = "COSTAR_HUSKY/odom";
    std::string init_submap_frame_ = "COSTAR_HUSKY";
    std::string travers_topic_ = "traversability";
    std::string frontiers_topic_ = "frontiers";
    std::string travers_expanded_topic_ = "travers_expanded";
    std::string path_topic_ = "rrt_path";

    std::vector<pairf> interest_points_;
    std::vector<pairf> explored_interest_points_;

    int map_update_frequency_ = 50;

    // tf::TransformListener transformListener_;
    tf2_ros::Buffer *tf_buffer_;
    tf2_ros::TransformListener *transformListener_;
    ros::Publisher pub_;
    ros::Publisher pub_trav_;
    ros::Publisher pub_pcl_gl_;
    ros::Publisher pub_frontier_;
    ros::Publisher pub_travers_expanded_;
    ros::Publisher pub_path_;
    ros::Subscriber sub_;

    ros::ServiceServer service;

    // methods

    bool readPearameters()
    {
        pnh_.getParam("pcl_topic", pcl_topic_);
        pnh_.getParam("out_pcl_topic", out_pcl_topic_);
        pnh_.getParam("map_frame", map_frame_);
        pnh_.getParam("init_submap_frame", init_submap_frame_);
        pnh_.getParam("traversability_topic", travers_topic_);
        pnh_.getParam("frontiers_topic", frontiers_topic_);
        pnh_.getParam("travers_expanded_topic", travers_expanded_topic_);
        pnh_.getParam("map_update_frequency", map_update_frequency_);
        pnh_.getParam("path_topic", path_topic_);

        std::cout << "pcl_topic: " << pcl_topic_ << std::endl;
        std::cout << "out_pcl_topic: " << out_pcl_topic_ << std::endl;
        std::cout << "map_frame: " << map_frame_ << std::endl;
        std::cout << "init_submap_frame: " << init_submap_frame_ << std::endl;
        std::cout << "traversability topic: " << travers_topic_ << std::endl;
        std::cout << "Frontiers topic: " << frontiers_topic_ << std::endl;
        std::cout << "Frontiers expanded topic: " << travers_expanded_topic_ << std::endl;
        std::cout << "Update frequency: " << map_update_frequency_ << std::endl;
        std::cout << "path_topic " << path_topic_ << std::endl;

        std::cout << "Parameters read" << std::endl;

        return true;
    }

    bool initialize_interface()
    {

        sub_ = nodeHandle_.subscribe(pcl_topic_, 1, &ElevationMapper::cloud_cb, mapper_);
        // Create a ROS publisher for the output point cloud
        pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(out_pcl_topic_, 1);
        pub_trav_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(travers_topic_, 1);
        pub_pcl_gl_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(transformed_pcl_topic_, 1);
        pub_frontier_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(frontiers_topic_, 1);
        pub_travers_expanded_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(travers_expanded_topic_, 1);
        pub_path_ = nodeHandle_.advertise<nav_msgs::Path>(path_topic_, 1);

        service = nodeHandle_.advertiseService("detect_frontiers", &ElevationMapper::frontrier_cb, mapper_);
        std::cout << "Interface initialized" << std::endl;
    }

    pairf getRobotPosition2D()
    {
        pairs answer(-1, -1);
        float coord3D[3];
        getRobotPosition3D(map_frame_, init_submap_frame_, coord3D);
        answer = pairf(coord3D[0], coord3D[1]);
        return answer;
    }

    void getRobotPosition3D(std::string fixed_frame, std::string target_frame, float *output)
    {
        geometry_msgs::TransformStamped transformTf;
        ros::Time timeStamp;
        timeStamp.fromSec(0.0);
        try
        {
            transformTf = tf_buffer_->lookupTransform(map_frame_, init_submap_frame_, timeStamp, ros::Duration(5.0));
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            std::cerr << "Failed to get a transform" << std::endl;
            return;
        }
        float x_coord = transformTf.transform.translation.x;
        float y_coord = transformTf.transform.translation.y;
        float z_coord = transformTf.transform.translation.z;

        try
        {
            output[0] = x_coord;
            output[1] = y_coord;
            output[2] = z_coord;
        }
        catch (std::exception &e)
        {
            std::cerr << "could not assign values to the output array : " << e.what() << std::endl;
        }

        return;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_elevation_map");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ElevationMapperRos mapperNode(nh, pnh);

    // Spin
    ros::spin();
}