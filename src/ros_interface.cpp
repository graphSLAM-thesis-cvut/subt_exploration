#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
// #include <grid_map_core/GridMap.hpp>
// #include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <grid_map_ros/GridMapRosConverter.hpp>
// #include "grid_map_core/GridMapMath.hpp"

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

#include "subt_params.h"
#include "wavefront_clustering.hpp"

typedef std::pair<int, int> pairs;
typedef std::pair<float, float> pairf;

using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;

enum RobotState
{
   INACTIVE,
   PLANNING,
   FOLLOWING,
   REACHED,
   FINISHED
};

enum ExpolrationState
{
   AUTOMATIC,
   MANUAL
};



class ElevationMapperRos
{
public:
    ElevationMapperRos(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle) : nodeHandle_(nodeHandle), pnh_(privateNodeHandle)
    {
        tf_buffer_ = new tf2_ros::Buffer();
        transformListener_ = new tf2_ros::TransformListener(*tf_buffer_);

        readPearameters();
        float robotPosition[3];
        // getRobotPosition3D(robotPosition, 20.0);

        initTimeMs = ros::Time::now().toNSec() / 1000000;
        // initializeInterface();
        sub_odom_ = nodeHandle_.subscribe(conf.odom_topic_, 1, &ElevationMapperRos::odom_cb, this);
        ROS_INFO("Waiting for first odometry message...");
    }
    ~ElevationMapperRos()
    {
        nodeHandle_.shutdown();
    }

private:
    ros::NodeHandle nodeHandle_;
    ros::NodeHandle pnh_;

    ElevationMapper *mapper_;

    subt_params::Params conf;

    RobotState state_ = INACTIVE;
    ExpolrationState explorationState_ = MANUAL;

    // tf::TransformListener transformListener_;
    tf2_ros::Buffer *tf_buffer_;
    tf2_ros::TransformListener *transformListener_;
    ros::Publisher pub_elevation_;
    ros::Publisher pub_trav_;
    ros::Publisher pub_pcl_gl_;
    ros::Publisher pub_frontier_;
    ros::Publisher pub_travers_expanded_;
    ros::Publisher pub_travers_expanded_plan_;
    ros::Publisher pub_path_;
    ros::Publisher pub_clearity_;
    ros::Publisher pub_robot_start_;

    ros::Subscriber sub_;
    ros::Subscriber sub_odom_;

    ros::ServiceServer serviceExploreOnce;
    ros::ServiceServer serviceCleanVisited;
    ros::ServiceServer serviceStartExploration;
    ros::ServiceServer serviceFinishExploration;

    
    std_srvs::Empty::Request emptyRequest_;
    std_srvs::Empty::Response emptyResponse_;

    float robot_position[3];

    uint32_t initTimeMs;
    ros::Time last_update;
    ros::Time lastExplorationTime;

    bool odom_received = false;

    // methods

    bool readPearameters()
    {
        conf = subt_params::load(pnh_);
        std::cout << "Parameters read" << std::endl;
        return true;
    }

    bool initializeInterface()
    {
        sub_ = nodeHandle_.subscribe(conf.pcl_topic_, 1, &ElevationMapperRos::cloud_cb, this);
        // Create a ROS publisher for the output point cloud
        pub_elevation_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(conf.out_pcl_topic_, 1);
        pub_trav_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(conf.travers_topic_, 1);
        pub_pcl_gl_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(conf.transformed_pcl_topic_, 1);
        pub_frontier_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(conf.frontiers_topic_, 1);
        pub_travers_expanded_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(conf.travers_expanded_topic_, 1);
        pub_travers_expanded_plan_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(conf.travers_expanded_plan_topic_, 1);
        pub_path_ = nodeHandle_.advertise<nav_msgs::Path>(conf.path_topic_, 1);
        pub_clearity_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(conf.clearity_topic_, 1);
        pub_robot_start_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("nearest_point", 1);

        serviceExploreOnce = nodeHandle_.advertiseService("explore_once", &ElevationMapperRos::explore_once_cb, this);
        serviceCleanVisited = nodeHandle_.advertiseService("clean_visited", &ElevationMapperRos::clean_visited, this);
        serviceStartExploration = nodeHandle_.advertiseService("start_exploration", &ElevationMapperRos::start_exploration_cb, this);
        serviceFinishExploration = nodeHandle_.advertiseService("finish_exploration", &ElevationMapperRos::finish_exploration_cb, this);


        std::cout << "Interface initialized" << std::endl;
        return true;
    }

    void odom_cb(const nav_msgs::Odometry &odometryMsg)
    {
        float x = odometryMsg.pose.pose.position.x;
        float y = odometryMsg.pose.pose.position.y;
        float z = odometryMsg.pose.pose.position.z;
        robot_position[0] = x;
        robot_position[1] = y;
        robot_position[2] = z;

        if (!odom_received){
            mapper_ = new ElevationMapper(nodeHandle_, pnh_, robot_position, conf);
            ROS_INFO("Received first odometry message!");
            odom_received = true;
            initializeInterface();
        }

        mapper_->updateRobotPosition(pairf(x, y));
        int pathLength = mapper_->current_path_.size();
        if (pathLength > 0){
            pairf goal = mapper_->indexToPosition(mapper_->current_path_[pathLength - 1]);
            if (std::sqrt( (x-goal.first)*(x-goal.first) + (y-goal.second)*(y-goal.second) ) < 0.5  ){
                state_=REACHED;
                if (explorationState_==AUTOMATIC){
                    explore_once();
                }
            }
        }
        ROS_INFO_THROTTLE(5, "Seconds since last update: %f (THROTTLE 5 seconds). NOW: %f LET: %f", (ros::Time::now() - lastExplorationTime).toSec(), ros::Time::now().toSec(), lastExplorationTime.toSec() );
        if (((ros::Time::now() - lastExplorationTime).toSec() > conf.followWaypointTimeout) && explorationState_==AUTOMATIC){
            mapper_->explored_interest_points_.pop_back();
            explore_once();
        }
        
    }

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg)
    {
        if (mapper_->map_used_){
            return;
            std::cout << "Skipping update as the map is being used" << std::endl;
        }
        // mapper_->updateRobotPosition(getRobotPosition2D());

        if (ros::Time::now() < last_update) // for going back in time - rosbag file
            last_update = ros::Time::now();
        if ((ros::Time::now() - last_update).toNSec() < 1 / float(conf.map_update_frequency_) * 1000000000)
            return;
        last_update = ros::Time::now();

        // converting msg to pcl
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);
        PointCloudType::Ptr pointCloud(new PointCloudType);
        pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);

        // transforming pointcloud to the world frame
        PointCloudType::Ptr pointCloudTransformed(new PointCloudType);
        if (!transformPointCloud(pointCloud, pointCloudTransformed))
        {
            ROS_ERROR_THROTTLE(10, "Point cloud could not be processed (Throttled 10s)");
            return;
        }

        // publish pointcloud in global coordinates
        publish_pcl(pointCloudTransformed, pub_pcl_gl_);

        // inserting pointcloud to the elevation map
        uint32_t cur_time = (pointCloudMsg->header.stamp.toNSec() / 1000000 - initTimeMs); // TODO: shift instead of division
        mapper_->insert_cloud(pointCloudTransformed, cur_time);

        // publish elevation map and traversability
        publish_elevation();
        publish_traversability(mapper_->traversability_, pub_trav_);
    }

    bool explore_once_cb(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
        lastExplorationTime = ros::Time::now();
        ROS_INFO("EXPLORARION ITERATION %f", lastExplorationTime.toSec());
        state_=PLANNING;
        ROS_INFO("EO 1");
        mapper_->map_used_ = true;

        ROS_INFO("EO 2");

        mapper_->detectFrontiers();
        publish_traversability(mapper_->traversability_expanded_, pub_travers_expanded_);


        ROS_INFO("EO 3");
        mapper_->detectInterestPoints();
        publish_interest_points();


        ROS_INFO("EO 4");
        mapper_->planToInterestPoint();
        publish_traversability(mapper_->traversability_expanded_, pub_travers_expanded_plan_);
        publish_path();
        publish_clearity();
        state_=FOLLOWING;

        publish_robot_positions();

        mapper_->map_used_ = false;
    }

    void explore_once(){
        explore_once_cb(emptyRequest_, emptyResponse_);
    }

    bool clean_visited(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
        std::vector<pairf> new_points;
        mapper_->explored_interest_points_ = new_points;
        return true;
    }


    bool start_exploration_cb(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
        ROS_INFO("STARTING EXPLORATION");
        // state_=PLANNING;
        explorationState_ = AUTOMATIC;
        explore_once();
    }

    bool finish_exploration_cb(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {

        ROS_INFO("FINISHING EXPLORATION");
        // state_=INACTIVE;
        explorationState_ = MANUAL;
    }



    bool transformPointCloud(const PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr &pointCloudTransformed)
    {
        ros::Time timeStamp;
        timeStamp.fromNSec(1000 * pointCloud->header.stamp);
        ros::Time lookupStamp = ros::Time::now();
        const std::string inputFrameId(pointCloud->header.frame_id);

        geometry_msgs::TransformStamped transformTf;
        try
        {
            transformTf = tf_buffer_->lookupTransform(conf.map_frame_, inputFrameId, timeStamp, ros::Duration(5.0));
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        Eigen::Affine3d transform;
        transform = tf2::transformToEigen(transformTf);
        pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
        pointCloudTransformed->header.frame_id = conf.map_frame_;
        pointCloudTransformed->header.stamp = pointCloud->header.stamp;

        ROS_INFO_THROTTLE(5, "Point cloud transformed to frame %s for time stamp %f. Transform timestamp %f. Current time: %f. LookupTime: %f.", conf.map_frame_.c_str(),
                           pointCloudTransformed->header.stamp / 1000000.0, transformTf.header.stamp.toSec(), ros::Time::now().toSec(), lookupStamp.toSec());
        return true;
    }

    bool publish_elevation()
    {
        std::vector<std::vector<float>> points;

        for (int i = mapper_->robotIndex.first - conf.vis_radius_cells_; i <= mapper_->robotIndex.first + conf.vis_radius_cells_; i++)
        {
            for (int j = mapper_->robotIndex.second - conf.vis_radius_cells_; j <= mapper_->robotIndex.second + conf.vis_radius_cells_; j++)
            {
                if (!mapper_->isIndexValid(i, j))
                {
                    continue;
                }
                if (!(mapper_->explored_(i, j)))
                {
                    continue;
                }
                pairf xy_coordinate = mapper_->indexToPosition(pairs(i, j));
                float coordinateX = xy_coordinate.first;
                float coordinateY = xy_coordinate.second;
                float elev = mapper_->elevation_(i, j);
                points.push_back(std::vector<float>({coordinateX, coordinateY, elev, 1}));
            }
        }

        publish_pcl(points, pub_elevation_);
        return true;
    }

    bool publish_traversability(Eigen::MatrixXf &trav, ros::Publisher pub)
    {
        std::vector<std::vector<float>> points;

        for (int i = mapper_->robotIndex.first - conf.vis_radius_cells_; i <= mapper_->robotIndex.first + conf.vis_radius_cells_; i++)
        {
            for (int j = mapper_->robotIndex.second - conf.vis_radius_cells_; j <= mapper_->robotIndex.second + conf.vis_radius_cells_; j++)
            {
                if (!mapper_->isIndexValid(i, j))
                {
                    continue;
                }
                if (!(mapper_->explored_)(i, j) || ((mapper_->traversability_)(i, j) == -1.0))
                {
                    continue;
                }
                pairf xy_coordinate = mapper_->indexToPosition(pairs(i, j));
                float coordinateX = xy_coordinate.first;
                float coordinateY = xy_coordinate.second;
                float travers = float(trav(i, j) > mapper_->conf.slope_th_);
                points.push_back(std::vector<float>({coordinateX, coordinateY, 0, travers}));
            }
        }

        publish_pcl(points, pub);
        return true;
    }

    bool publish_traversability_expanded(Eigen::MatrixXf &trav, pairs start_point, ros::Publisher pub)
    {
        std::vector<std::vector<float>> points;
        std::vector<std::vector<pairs>> clustered_points;

        wf_clustering::cluster_map(trav, mapper_->conf.slope_th_, start_point, mapper_->explored_, clustered_points);
        
        if (clustered_points.size() == 0){
            ROS_INFO("Publishing traversability: Unexpected: nothing is clustered!");
            return true;
        }
        for (size_t i = 0; i < clustered_points[0].size(); i++)
        {
            int indexX = clustered_points[0][i].first;
            int indexY = clustered_points[0][i].second;
            pairf xy_coordinate = mapper_->indexToPosition(pairs(indexX, indexY));
            float coordinateX = xy_coordinate.first;
            float coordinateY = xy_coordinate.second;
            float travers = float(trav(indexX, indexY) > mapper_->conf.slope_th_);
            points.push_back(std::vector<float>({coordinateX, coordinateY, 0, travers}));
        }
        if (clustered_points.size() == 1){
            ROS_INFO("Publishing traversability: Unexpected: no obstacles clustered!");
            return true;
        }
        for (size_t i = 0; i < clustered_points[1].size(); i++)
        {
            int indexX = clustered_points[1][i].first;
            int indexY = clustered_points[1][i].second;
            pairf xy_coordinate = mapper_->indexToPosition(pairs(indexX, indexY));
            float coordinateX = xy_coordinate.first;
            float coordinateY = xy_coordinate.second;
            float travers = float(trav(indexX, indexY) > mapper_->conf.slope_th_);
            points.push_back(std::vector<float>({coordinateX, coordinateY, 0, travers}));
        }
        if (clustered_points.size() > 2){
            ROS_INFO("Publishing traversability: Unexpected: there are more then two cluster!");
            return true;
        }

        publish_pcl(points, pub);
        return true;
    }

    bool publish_clearity()
    {
        std::vector<std::vector<float>> points;

        for (int i = mapper_->robotIndex.first - conf.vis_radius_cells_; i <= mapper_->robotIndex.first + conf.vis_radius_cells_; i++)
        {
            for (int j = mapper_->robotIndex.second - conf.vis_radius_cells_; j <= mapper_->robotIndex.second + conf.vis_radius_cells_; j++)
            {
                if (!mapper_->isIndexValid(i, j))
                {
                    continue;
                }
                if (!(mapper_->explored_)(i, j) || std::fabs(mapper_->clearity_(i, j)) > 40)
                {
                    continue;
                }
                pairf xy_coordinate = mapper_->indexToPosition(pairs(i, j));
                float coordinateX = xy_coordinate.first;
                float coordinateY = xy_coordinate.second;
                float clearity = mapper_->clearity_(i, j);
                points.push_back(std::vector<float>({coordinateX, coordinateY, 0, clearity}));
            }
        }

        publish_pcl(points, pub_clearity_);
        return true;
    }

    bool publish_robot_positions()
    {
        std::vector<std::vector<float>> points;
        pairf position = mapper_->indexToPosition(mapper_->last_requested_index);
        pairf position_plan = mapper_->indexToPosition(mapper_->planning_point_);
        pairf goal = mapper_->indexToPosition(mapper_->last_goal);
        points.push_back(std::vector<float>({position.first, position.second, 0.2, 1}));
        points.push_back(std::vector<float>({position_plan.first, position_plan.second, 0.25, 0.5}));
        points.push_back(std::vector<float>({goal.first, goal.second, 0.15, 0.75}));

        publish_pcl(points, pub_robot_start_);
        return true;
    }

    bool publish_interest_points()
    {
        std::vector<std::vector<float>> points;

        int i = 0;
        for (auto &p : mapper_->current_interest_points_)
        {
            i++;
            // for (auto& p: f){
            pairf xy = mapper_->indexToPosition(p);
            float intensity = (float(i) / float(mapper_->current_interest_points_.size()));
            points.push_back(std::vector<float>({xy.first, xy.second, 0, intensity}));
        }

        publish_pcl(points, pub_frontier_);
        return true;
    }

    bool publish_pcl(std::vector<std::vector<float>> points, ros::Publisher pub)
    {
        PointCloudType::Ptr pointCloud(new PointCloudType);
        pointCloud->header.frame_id = conf.map_frame_;
        pointCloud->header.stamp = ros::Time::now().toNSec() / 1000;

        for (auto &pt : points)
        {
            pcl::PointXYZI p(pt[3]);
            p.x = pt[0];
            p.y = pt[1];
            p.z = pt[2];
            pointCloud->insert(pointCloud->end(), p);
        }

        publish_pcl(pointCloud, pub);
        return true;
    }

    bool publish_pcl(PointCloudType::Ptr pointCloud, ros::Publisher pub)
    {

        pcl::PCLPointCloud2 pcl_pc;
        sensor_msgs::PointCloud2 output;
        pcl::toPCLPointCloud2(*pointCloud, pcl_pc);
        pcl_conversions::fromPCL(pcl_pc, output);

        // Publish the data
        pub.publish(output);
        return true;
    }

    bool publish_path()
    {
        if (mapper_->current_path_.size() > 0)
        {
            nav_msgs::Path path;
            path.header.frame_id = conf.map_frame_;
            path.header.stamp = ros::Time::now();

            std::vector<geometry_msgs::PoseStamped> poses;
            int seq = 0;
            for (auto &xy_ind : mapper_->current_path_)
            {
                seq++;
                pairf xy_coordinate = mapper_->indexToPosition(xy_ind);
                geometry_msgs::PoseStamped *pose = new geometry_msgs::PoseStamped();
                pose->header = path.header;
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