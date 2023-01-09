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
// #include <utility>


typedef std::pair<int, int> pairs;


// std::string out_pcl_topic_ = "/my_elevation_map/pcl";
// // std::string pcl_topic = "/cropbox/output";
// std::string pcl_topic_ = "COSTAR_HUSKY/points";
// // tf::Buffer buffer_;
// tf::TransformListener transformListener_;

// std::string map_frame_ = "COSTAR_HUSKY/odom";



// float origin1_ = 0;
// float origin2_ = 0;

// int size1_ = 20;
// int size2_ = 20;
// float resolution_ = 0.1;
// bool rosbag_ = false;

// int n_cells1_ = int(size1_/resolution_);
// int n_cells2_ = int(size2_/resolution_);

// Eigen::MatrixXf elevation_;
// Eigen::MatrixXi explored_;

using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;

class ElevationMapper{
  public:
    ElevationMapper(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle): nodeHandle_(nodeHandle), pnh_(privateNodeHandle){
      readPearameters();
      tf_buffer_ = new tf2_ros::Buffer();
	    transformListener_ = new tf2_ros::TransformListener(*tf_buffer_);
      initTimeMs = ros::Time::now().toNSec()/1000000;
    }
    ~ElevationMapper(){
      nodeHandle_.shutdown();
    }

  private:
    ros::NodeHandle nodeHandle_;
    ros::NodeHandle pnh_;
    float origin1_ = 0;
    float origin2_ = 0;

    int size1_ = 20;
    int size2_ = 20;
    float resolution_ = 0.1;
    int n_cells1_ = -1;
    int n_cells2_ = -1;
    bool rosbag_ = false;
    uint32_t initTimeMs;

    std::string out_pcl_topic_ = "/my_elevation_map/pcl";
    std::string transformed_pcl_topic_ = "/my_elevation_map/pcl_glob";
    std::string pcl_topic_ = "COSTAR_HUSKY/points";
    std::string map_frame_ = "COSTAR_HUSKY/odom";

    Eigen::MatrixXf elevation_;
    Eigen::MatrixXi elevation_time_estimated_; // To get the highest point from the cell. TODO: maybe substitude with kd-tree ?
    Eigen::MatrixXf current_measurement_; // To get the highest point from the cell. TODO: maybe substitude with kd-tree ?
    Eigen::MatrixXi explored_;

    // tf::TransformListener transformListener_;
    tf2_ros::Buffer* tf_buffer_;
	  tf2_ros::TransformListener* transformListener_;
    ros::Publisher pub_;
    ros::Publisher pub_pcl_gl_;
    ros::Subscriber sub_;

    bool readPearameters(){
      pnh_.getParam("pcl_topic", pcl_topic_);
      pnh_.getParam("origin1", origin1_);
      pnh_.getParam("origin2", origin2_);
      pnh_.getParam("size1", size1_);
      pnh_.getParam("size2", size2_);
      pnh_.getParam("resolution", resolution_);
      pnh_.getParam("out_pcl_topic", out_pcl_topic_);
      pnh_.getParam("map_frame", map_frame_);
      pnh_.getParam("rosbag", rosbag_);

      std::cout << "pcl_topic: " << pcl_topic_ << std::endl;
      std::cout << "origin1: " << origin1_ << std::endl;
      std::cout << "origin2: " << origin2_ << std::endl;
      std::cout << "size1: " << size1_ << std::endl;
      std::cout << "size2: " << size2_ << std::endl;
      std::cout << "resolution: " << resolution_ << std::endl;
      std::cout << "out_pcl_topic: " << out_pcl_topic_ << std::endl;
      std::cout << "map_frame: " << map_frame_ << std::endl;
      std::cout << "rosbag: " << rosbag_ << std::endl;

      n_cells1_ = int(size1_/resolution_);
      n_cells2_ = int(size2_/resolution_);

      std::cout << "INITIALIZING MATRICIES" << std::endl;
      elevation_.resize(n_cells1_, n_cells2_);
      explored_.resize(n_cells1_, n_cells2_);
      elevation_time_estimated_.resize(n_cells1_, n_cells2_);
      current_measurement_.resize(n_cells1_, n_cells2_);
      

      std::cout << "n_cells1_: " << n_cells1_ << std::endl;
      std::cout << "n_cells2_: " << n_cells2_ << std::endl;

      for (size_t i = 0; i < n_cells1_; i++)
      {
        for (size_t j = 0; j < n_cells2_; j++)
        {
          explored_(i, j) = 0;
        }
      }

      std::cout << "MATRICIES initialized" << std::endl;
      sub_ = nodeHandle_.subscribe (pcl_topic_, 1, &ElevationMapper::cloud_cb, this);

      // Create a ROS publisher for the output point cloud
      pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2> (out_pcl_topic_, 1);
      pub_pcl_gl_ = nodeHandle_.advertise<sensor_msgs::PointCloud2> (transformed_pcl_topic_, 1);

      std::cout << "Node initialized" << std::endl;

      return true;
    }

    bool transformPointCloud(const PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr& pointCloudTransformed) {
  
      // ros::Time timeStamp = ros::Time::now();
      ros::Time timeStamp;
      timeStamp.fromNSec(1000 * pointCloud->header.stamp);

      ros::Time timeStamp2(timeStamp);
      if (rosbag_)
        timeStamp2.fromSec(0.0);
      const std::string inputFrameId(pointCloud->header.frame_id);

      geometry_msgs::TransformStamped transformTf;
      try {
        // transformListener_->waitForTransform(map_frame_, inputFrameId, timeStamp, ros::Duration(0.2), ros::Duration(0.001));
        transformTf = tf_buffer_->lookupTransform(map_frame_, inputFrameId, timeStamp2, ros::Duration(5.0));
      } catch (tf::TransformException& ex) {
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

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
    {
      // std::cout << "callback" << std::endl;
      // Create a container for the data.
      // Container for original & filtered data
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

      PointCloudType::Ptr pointCloud(new PointCloudType);
      pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
      PointCloudType::Ptr pointCloudTransformed(new PointCloudType);

      uint32_t cur_time = (pointCloudMsg->header.stamp.toNSec() / 1000000 - initTimeMs); // TODO: shift instead of division

      if (!transformPointCloud(pointCloud, pointCloudTransformed)) {
        ROS_ERROR_THROTTLE(10, "Point cloud could not be processed (Throttled 10s)");
        return;
      }
      sensor_msgs::PointCloud2 output;
      pcl::toPCLPointCloud2(*pointCloudTransformed, pcl_pc);
      pcl_conversions::fromPCL (pcl_pc, output);

      // // Pub_lish the data
      pub_pcl_gl_.publish (output);

      PointCloudType::Ptr pointCloudGrid(new PointCloudType);
    // pcl::fromPCLPointCloud2(pcl_pc, *pointCloudGrid);

      pointCloudGrid->header.frame_id = map_frame_;
      pointCloudGrid->header = pointCloudTransformed->header;

      // std::vector<int> exploredI;
      // std::vector<int> exploredJ;
      std::set<pairs> measured;
      // std::cout << "finding out heights" << std::endl;
      // std::vector<>
      for (size_t i = 0; i < pointCloudTransformed->size(); i++)
      {
        auto& point = pointCloudTransformed->points[i];
        float coordinateX = point.x - origin1_;
        float coordinateY = point.y - origin2_;
        int indexX = int(coordinateX/resolution_ + n_cells1_/2);
        int indexY = int(coordinateY/resolution_ + n_cells2_/2);
        if ((indexX < 0) || (indexX >= n_cells1_)){
          ROS_ERROR_THROTTLE(10, "X index out of range");
          // return;
          continue;
        }
        if ((indexY < 0) || (indexY >= n_cells2_)){
          ROS_ERROR_THROTTLE(10, "Y index out of range");
          // return;
          continue;
        }
        // if (1 == (explored_)(indexX, indexY)){
          if (cur_time == elevation_time_estimated_(indexX, indexY)){
            current_measurement_(indexX, indexY) = std::max(point.z, current_measurement_(indexX, indexY));
          } else {
            current_measurement_(indexX, indexY) = point.z;
            elevation_time_estimated_(indexX, indexY) = cur_time;
          }
          // exploredI.push_back(indexX);
          // exploredJ.push_back(indexY);
          measured.insert(pairs(indexX, indexY));
          // (elevation_)(indexX, indexY) = (elevation_)(indexX, indexY)*0.9 + point.z*0.1;
        // } else {
        //   (elevation_)(indexX, indexY) = point.z;
        //   (explored_)(indexX, indexY) = 1;
        //   elevation_time_estimated_(indexX, indexY) = cur_time;
        // }
      }

      for (auto& pair: measured){
        int i = pair.first;
        int j = pair.second;
        if (cur_time == elevation_time_estimated_(i, j)){
      //       // continue;
            if (1 == (explored_)(i, j)){
              (elevation_)(i, j) = (elevation_)(i, j)*0.9 + current_measurement_(i, j)*0.1;
            } else {
              (elevation_)(i, j) = current_measurement_(i, j);
              (explored_)(i, j) = 1;
            }
          }
      }

      // for (int i = 0; i < n_cells1_; i++)
      // {
      //   for (int j = 0; j < n_cells2_; j++)
      //   {
      //     if (cur_time == elevation_time_estimated_(i, j)){
      //       // continue;
      //       if (1 == (explored_)(i, j)){
      //         (elevation_)(i, j) = (elevation_)(i, j)*0.9 + current_measurement_(i, j)*0.1;
      //       } else {
      //         (elevation_)(i, j) = current_measurement_(i, j);
      //         (explored_)(i, j) = 1;
      //       }
      //     }
      //     // float coordinateX = (i - n_cells1_/2) * resolution_; 
      //     // float coordinateY = (j - n_cells2_/2) * resolution_; 
      //     // pcl::PointXYZI p(1.0);
      //     // p.x = coordinateX + origin1_;
      //     // p.y = coordinateY + origin2_;
      //     // p.z = elevation_.coeff(i, j);
      //     // pointCloudGrid->insert(pointCloudGrid->end(), p);
      //   }
      // }


      // std::cout << "Publishing heights" << std::endl;
      for (auto& pair: measured){
        int i = pair.first;
        int j = pair.second;
        if (!(explored_)(i, j)){
            continue;
          }
          float coordinateX = (i - n_cells1_/2) * resolution_; 
          float coordinateY = (j - n_cells2_/2) * resolution_; 
          pcl::PointXYZI p(1.0);
          p.x = coordinateX + origin1_;
          p.y = coordinateY + origin2_;
          p.z = elevation_.coeff(i, j);
          pointCloudGrid->insert(pointCloudGrid->end(), p);
      }
      // for (int i = 0; i < n_cells1_; i++)
      // {
      //   for (int j = 0; j < n_cells2_; j++)
      //   {
      //     if (!(explored_)(i, j)){
      //       continue;
      //     }
      //     float coordinateX = (i - n_cells1_/2) * resolution_; 
      //     float coordinateY = (j - n_cells2_/2) * resolution_; 
      //     pcl::PointXYZI p(1.0);
      //     p.x = coordinateX + origin1_;
      //     p.y = coordinateY + origin2_;
      //     p.z = elevation_.coeff(i, j);
      //     pointCloudGrid->insert(pointCloudGrid->end(), p);
      //   }
      // }

      
      pcl::toPCLPointCloud2(*pointCloudGrid, pcl_pc);
      pcl_conversions::fromPCL (pcl_pc, output);

      // // Pub_lish the data
      pub_.publish (output);

      

    }

      

};


int
main (int argc, char** argv)
{
  
  ros::init (argc, argv, "my_elevation_map");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ElevationMapper mapper(nh, pnh);
  // nh.param("pcl_topic", pcl_topic);
  // nh.param("origin1", origin1);
  // nh.param("origin2", origin2);
  // nh.param("size1", size1);
  // nh.param("size2", size2);
  // nh.param("resolution", resolution);
  // nh.param("out_pcl_topic", out_pcl_topic);
  // nh.param("map_frame", map_frame_);
  // nh.param("rosbag", rosbag);

  // std::cout << "pcl_topic: " << pcl_topic << std::endl;
  // std::cout << "origin1: " << origin1 << std::endl;
  // std::cout << "origin2: " << origin2 << std::endl;
  // std::cout << "size1: " << size1 << std::endl;
  // std::cout << "size2: " << size2 << std::endl;
  // std::cout << "resolution: " << resolution << std::endl;
  // std::cout << "out_pcl_topic: " << out_pcl_topic << std::endl;
  // std::cout << "map_frame: " << map_frame_ << std::endl;
  // std::cout << "rosbag: " << rosbag << std::endl;

  // n_cells1 = int(size1/resolution);
  // n_cells2 = int(size2/resolution);
  // explored_ = Eigen::MatrixXf(n_cells1, n_cells2);
  // Initialize ROS
  // for (size_t i = 0; i < n_cells1; i++)
  // {
  //   for (size_t j = 0; j < n_cells2; j++)
  //   {
  //     explored_(i, j) = 0.0;
  //   }
  // }
  

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe (pcl_topic_, 1, cloud_cb);

  // // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> (out_pcl_topic_, 1);

  // Spin
  ros::spin ();
}