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

        initTimeMs = ros::Time::now().toNSec() / 1000000;
        readPearameters();
        initializeInterface();
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


    float resolution_ = 0.1;
    float vis_radius_ = 6.0;
    int vis_radius_cells_ = 60;

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

    uint32_t initTimeMs;
    ros::Time last_update;
    int map_update_frequency_ = 50;

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
        pnh_.getParam("resolution", resolution_);


        pnh_.getParam("vis_radius", vis_radius_);

        std::cout << "pcl_topic: " << pcl_topic_ << std::endl;
        std::cout << "out_pcl_topic: " << out_pcl_topic_ << std::endl;
        std::cout << "map_frame: " << map_frame_ << std::endl;
        std::cout << "init_submap_frame: " << init_submap_frame_ << std::endl;
        std::cout << "traversability topic: " << travers_topic_ << std::endl;
        std::cout << "Frontiers topic: " << frontiers_topic_ << std::endl;
        std::cout << "Frontiers expanded topic: " << travers_expanded_topic_ << std::endl;
        std::cout << "Update frequency: " << map_update_frequency_ << std::endl;
        std::cout << "path_topic " << path_topic_ << std::endl;


        std::cout << "vis_radius: " << vis_radius_ << std::endl;



        vis_radius_cells_ = int(vis_radius_ / resolution_);


        std::cout << "vis_radius_cells: " << vis_radius_cells_ << std::endl;

        std::cout << "Parameters read" << std::endl;

        return true;
    }

    bool initializeInterface()
    {
        sub_ = nodeHandle_.subscribe(pcl_topic_, 1, &ElevationMapperRos::cloud_cb, this);
        // Create a ROS publisher for the output point cloud
        pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(out_pcl_topic_, 1);
        pub_trav_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(travers_topic_, 1);
        pub_pcl_gl_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(transformed_pcl_topic_, 1);
        pub_frontier_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(frontiers_topic_, 1);
        pub_travers_expanded_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(travers_expanded_topic_, 1);
        pub_path_ = nodeHandle_.advertise<nav_msgs::Path>(path_topic_, 1);

        service = nodeHandle_.advertiseService("detect_frontiers", &ElevationMapperRos::frontrier_cb, this);
        std::cout << "Interface initialized" << std::endl;
        return true;
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

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg)
    {

        // std::cout << "ROS: cloud callback" << std::endl;
        // Create a container for the data.
        // Container for original & filtered data
        mapper_->updateRobotPosition(getRobotPosition2D());
        if (ros::Time::now() < last_update) // for going back in time - rosbag file
            last_update = ros::Time::now();
        if ((ros::Time::now() - last_update).toNSec() < 1 / float(map_update_frequency_) * 1000000000)
            return;
        last_update = ros::Time::now();

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

        PointCloudType::Ptr pointCloud(new PointCloudType);
        pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);

        PointCloudType::Ptr pointCloudTransformed(new PointCloudType);

        if (!transformPointCloud(pointCloud, pointCloudTransformed))
        {
            ROS_ERROR_THROTTLE(10, "Point cloud could not be processed (Throttled 10s)");
            return;
        }
        

        sensor_msgs::PointCloud2 output;
        pcl::toPCLPointCloud2(*pointCloudTransformed, pcl_pc);
        pcl_conversions::fromPCL(pcl_pc, output);

        // // Pub_lish the data

        pub_pcl_gl_.publish(output);


        uint32_t cur_time = (pointCloudMsg->header.stamp.toNSec() / 1000000 - initTimeMs); // TODO: shift instead of division

    //   std::cout << "ROS: calling inserter" << std::endl;
        mapper_->insert_cloud(pointCloudTransformed, cur_time);

        

        PointCloudType::Ptr pointCloudGrid(new PointCloudType);

        pointCloudGrid->header.frame_id = map_frame_;
        pointCloudGrid->header = pointCloudTransformed->header;
        
    //   std::cout << "ROS: calling inserter" << std::endl;
        auto robot_position = mapper_->positionToIndex( getRobotPosition2D());
        int Rx = robot_position.first;
        int Ry = robot_position.second;

        for (int i = Rx - vis_radius_cells_; i <= Rx + vis_radius_cells_; i++)
        {
            for (int j = Ry - vis_radius_cells_; j <= Ry + vis_radius_cells_; j++)
            {
                if (!mapper_->isIndexValid(i, j))
                {
                    continue;
                }
                if (!(mapper_->explored_)(i, j))
                {
                    continue;
                }

                pairf xy_coordinate = mapper_->indexToPosition(pairs(i, j));
                float coordinateX = xy_coordinate.first;
                float coordinateY = xy_coordinate.second;
                pcl::PointXYZI p(1.0);
                p.x = coordinateX;
                p.y = coordinateY;
                p.z = mapper_->elevation_.coeff(i, j);
                pointCloudGrid->insert(pointCloudGrid->end(), p);
            }
        }

        pcl::toPCLPointCloud2(*pointCloudGrid, pcl_pc);
        pcl_conversions::fromPCL(pcl_pc, output);

        // Publish the data
        pub_.publish(output);

        PointCloudType::Ptr pcl_trav(new PointCloudType);
        pcl_trav->header.frame_id = map_frame_;
        pcl_trav->header = pointCloudTransformed->header;

        for (int i = Rx - vis_radius_cells_; i <= Rx + vis_radius_cells_; i++)
        {
            for (int j = Ry - vis_radius_cells_; j <= Ry + vis_radius_cells_; j++)
            {
                if (!mapper_->isIndexValid(i, j))
                {
                    continue;
                }
                if (!(mapper_->explored_)(i, j) || (mapper_->traversability_(i, j) == -1.0))
                {
                    continue;
                }
                pairf xy_coordinate = mapper_->indexToPosition(pairs(i, j));
                float coordinateX = xy_coordinate.first;
                float coordinateY = xy_coordinate.second;
                pcl::PointXYZI p(mapper_->traversability_(i, j) > mapper_->slope_th_);
                p.x = coordinateX;
                p.y = coordinateY;
                p.z = 0;
                pcl_trav->insert(pcl_trav->end(), p);
            }
        }

        pcl::toPCLPointCloud2(*pcl_trav, pcl_pc);
        pcl_conversions::fromPCL(pcl_pc, output);

        // Publish the data
        pub_trav_.publish(output);
    }

    bool frontrier_cb(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
  {
    mapper_->map_used_ = true;
    // pairf robotPosition = getRobotPosition2D();
    mapper_->updateRobotPosition(getRobotPosition2D());

    mapper_->detect_frontiers();

    sensor_msgs::PointCloud2 output;
    PointCloudType::Ptr pointCloudFrontier(new PointCloudType);

    pointCloudFrontier->header.frame_id = map_frame_;
    pointCloudFrontier->header.stamp = ros::Time::now().toNSec() / 1000;

    int i = 0;
    for (auto &p : mapper_->current_interest_points_)
    {
      i++;
      // for (auto& p: f){
      pairf xy =  mapper_->indexToPosition(p);
      pcl::PointXYZI pt(float(i) / float( mapper_->current_interest_points_.size()));
      pt.x = xy.first;
      pt.y = xy.second;
      pt.z = 0;
      pointCloudFrontier->insert(pointCloudFrontier->end(), pt);
      // }
    }

    pcl::PCLPointCloud2 pcl_pc;
    pcl::toPCLPointCloud2(*pointCloudFrontier, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, output);
    pub_frontier_.publish(output);

    // publish expanded frontier

    PointCloudType::Ptr pointCloudTravExp(new PointCloudType);
    pointCloudTravExp->header.frame_id = map_frame_;
    pointCloudTravExp->header = pointCloudFrontier->header;

    for (int i = mapper_->robotIndex.first - vis_radius_cells_; i <= mapper_->robotIndex.first + vis_radius_cells_; i++)
    {
      for (int j = mapper_->robotIndex.second - vis_radius_cells_; j <= mapper_->robotIndex.second + vis_radius_cells_; j++)
      {
        if (!mapper_->isIndexValid(i, j))
        {
          continue;
        }
        if (!(mapper_->explored_)(i, j) || (mapper_->traversability_expanded_(i, j) == -1.0))
        {
          continue;
        }
        pairf xy_coordinate = mapper_->indexToPosition(pairs(i, j));
        float coordinateX = xy_coordinate.first;
        float coordinateY = xy_coordinate.second;
        pcl::PointXYZI p(mapper_->traversability_expanded_(i, j) > mapper_->slope_th_);
        p.x = coordinateX;
        p.y = coordinateY;
        p.z = 0;
        pointCloudTravExp->insert(pointCloudTravExp->end(), p);
      }
    }

    pcl::toPCLPointCloud2(*pointCloudTravExp, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, output);

    // Publish the data
    pub_travers_expanded_.publish(output);

    if (mapper_->current_path_.size() > 0)
    {
      nav_msgs::Path path;
      path.header = output.header;

      std::vector<geometry_msgs::PoseStamped> poses;
      int seq = 0;
      for (auto &xy_ind : mapper_->current_path_)
      {
        seq++;
        pairf xy_coordinate = mapper_->indexToPosition(xy_ind);
        geometry_msgs::PoseStamped *pose = new geometry_msgs::PoseStamped();
        pose->header = output.header;
        pose->header.seq = seq;
        geometry_msgs::Pose posee;
        geometry_msgs::Quaternion orientation;
        orientation.w = 0;
        orientation.x = 0;
        orientation.y = 0;
        orientation.z = 0;
        posee.orientation = orientation;
        geometry_msgs::Point point;
        point.x = xy_coordinate.first;
        point.y = xy_coordinate.second;
        point.z = 0;
        posee.position = point;
        pose->pose = posee;
        poses.push_back(*pose);
      }
      path.poses = poses;

      // // Publish the data
      pub_path_.publish(path);
    }

    mapper_->map_used_ = false;
    return true;
  }

    bool transformPointCloud(const PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr &pointCloudTransformed)
    {

        // ros::Time timeStamp = ros::Time::now();
        ros::Time timeStamp;
        timeStamp.fromNSec(1000 * pointCloud->header.stamp);

        const std::string inputFrameId(pointCloud->header.frame_id);

        geometry_msgs::TransformStamped transformTf;
        try
        {
            // transformListener_->waitForTransform(map_frame_, inputFrameId, timeStamp, ros::Duration(0.2), ros::Duration(0.001));
            transformTf = tf_buffer_->lookupTransform(map_frame_, inputFrameId, timeStamp, ros::Duration(5.0));
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        Eigen::Affine3d transform;
        transform = tf2::transformToEigen(transformTf);
        pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
        pointCloudTransformed->header.frame_id = map_frame_;

        // ROS_DEBUG_THROTTLE(5, "Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
        //                    pointCloudTransformed->header.stamp / 1000.0);
        return true;
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