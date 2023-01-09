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
typedef std::pair<float, float> pairf;

using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;

class ElevationMapper{
  public:
    ElevationMapper(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle): nodeHandle_(nodeHandle), pnh_(privateNodeHandle){
      readPearameters();
      tf_buffer_ = new tf2_ros::Buffer();
	    transformListener_ = new tf2_ros::TransformListener(*tf_buffer_);
      initTimeMs = ros::Time::now().toNSec()/1000000;
      if (init_submap_){
        bool inited = init_submap();
        std::cout << "Submap initialized successfully: " << inited << std::endl;
      }
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
    uint32_t initTimeMs;

    float vis_radius_ = 6.0;
    int vis_radius_cells_ = 60;

    bool init_submap_ = false;
    float init_submap_height_offset_ = 0.0;
    float init_submap_radius_ = 0.0;

    bool publish_traversability_ = false;

    std::string out_pcl_topic_ = "elevation";
    std::string transformed_pcl_topic_ = "pcl_glob";
    std::string pcl_topic_ = "COSTAR_HUSKY/points";
    std::string map_frame_ = "COSTAR_HUSKY/odom";
    std::string init_submap_frame_ = "COSTAR_HUSKY";
    std::string travers_topic_ = "traversability";
    float slope_th_ = 0.1;

    Eigen::MatrixXf elevation_;
    Eigen::MatrixXi elevation_time_estimated_; // To get the highest point from the cell. TODO: maybe substitude with kd-tree ?
    Eigen::MatrixXf current_measurement_; // To get the highest point from the cell. TODO: maybe substitude with kd-tree ?
    Eigen::MatrixXi explored_;
    Eigen::MatrixXi traversability_;

    // tf::TransformListener transformListener_;
    tf2_ros::Buffer* tf_buffer_;
	  tf2_ros::TransformListener* transformListener_;
    ros::Publisher pub_;
    ros::Publisher pub_trav_;
    ros::Publisher pub_pcl_gl_;
    ros::Subscriber sub_;

    //methods

    bool readPearameters(){
      pnh_.getParam("pcl_topic", pcl_topic_);
      pnh_.getParam("origin1", origin1_);
      pnh_.getParam("origin2", origin2_);
      pnh_.getParam("size1", size1_);
      pnh_.getParam("size2", size2_);
      pnh_.getParam("resolution", resolution_);
      pnh_.getParam("out_pcl_topic", out_pcl_topic_);
      pnh_.getParam("map_frame", map_frame_);
      pnh_.getParam("vis_radius", vis_radius_);
      pnh_.getParam("init_submap", init_submap_);
      pnh_.getParam("init_submap_radius", init_submap_radius_);
      pnh_.getParam("init_submap_height_offset", init_submap_height_offset_);
      pnh_.getParam("init_submap_frame", init_submap_frame_);
      pnh_.getParam("traversability_topic", travers_topic_);
      pnh_.getParam("slope_th", slope_th_);

      std::cout << "pcl_topic: " << pcl_topic_ << std::endl;
      std::cout << "origin1: " << origin1_ << std::endl;
      std::cout << "origin2: " << origin2_ << std::endl;
      std::cout << "size1: " << size1_ << std::endl;
      std::cout << "size2: " << size2_ << std::endl;
      std::cout << "resolution: " << resolution_ << std::endl;
      std::cout << "out_pcl_topic: " << out_pcl_topic_ << std::endl;
      std::cout << "map_frame: " << map_frame_ << std::endl;
      std::cout << "vis_radius: " << vis_radius_ << std::endl;
      std::cout << "init_submap: " << init_submap_ << std::endl;
      std::cout << "init_submap_radius: " << init_submap_radius_ << std::endl;
      std::cout << "init_submap_height_offset: " << init_submap_height_offset_ << std::endl;
      std::cout << "init_submap_frame: " << init_submap_frame_ << std::endl;
      std::cout << "traversability topic: " << travers_topic_ << std::endl;
      std::cout << "Slope threshhold: " << slope_th_ << std::endl;


      n_cells1_ = int(size1_/resolution_);
      n_cells2_ = int(size2_/resolution_);
      vis_radius_cells_ = int(vis_radius_/resolution_);

      std::cout << "INITIALIZING MATRICIES" << std::endl;
      elevation_.resize(n_cells1_, n_cells2_);
      explored_.resize(n_cells1_, n_cells2_);
      elevation_time_estimated_.resize(n_cells1_, n_cells2_);
      current_measurement_.resize(n_cells1_, n_cells2_);
      traversability_.resize(n_cells1_, n_cells2_);
      

      std::cout << "n_cells1: " << n_cells1_ << std::endl;
      std::cout << "n_cells2: " << n_cells2_ << std::endl;
      std::cout << "vis_radius_cells: " << vis_radius_cells_ << std::endl;

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
      pub_trav_ = nodeHandle_.advertise<sensor_msgs::PointCloud2> (travers_topic_, 1);
      pub_pcl_gl_ = nodeHandle_.advertise<sensor_msgs::PointCloud2> (transformed_pcl_topic_, 1);

      std::cout << "Node initialized" << std::endl;

      return true;
    }

    bool transformPointCloud(const PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr& pointCloudTransformed) {
  
      // ros::Time timeStamp = ros::Time::now();
      ros::Time timeStamp;
      timeStamp.fromNSec(1000 * pointCloud->header.stamp);

      const std::string inputFrameId(pointCloud->header.frame_id);

      geometry_msgs::TransformStamped transformTf;
      try {
        // transformListener_->waitForTransform(map_frame_, inputFrameId, timeStamp, ros::Duration(0.2), ros::Duration(0.001));
        transformTf = tf_buffer_->lookupTransform(map_frame_, inputFrameId, timeStamp, ros::Duration(5.0));
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

      

      std::set<pairs> measured;
      for (size_t i = 0; i < pointCloudTransformed->size(); i++)
      {
        auto& point = pointCloudTransformed->points[i];
        // float coordinateX = point.x - origin1_;
        // float coordinateY = point.y - origin2_;
        pairs xy_index = positionToIndex(pairf(point.x, point.y));
        int indexX = xy_index.first;
        int indexY = xy_index.second;

        // std::cout << "iterating" << std::endl;
        if (!isIndexValid(indexX, indexY)){
          // std::cout << "invalid " << indexX << " " << indexY << std::endl;
          ROS_ERROR_THROTTLE(10, "X, Y index out of range" );
          continue;
        }

        if (cur_time == elevation_time_estimated_(indexX, indexY)){
          current_measurement_(indexX, indexY) = std::max(point.z, current_measurement_(indexX, indexY));
        } else {
          current_measurement_(indexX, indexY) = point.z;
          elevation_time_estimated_(indexX, indexY) = cur_time;
        }
        // std::cout << "inserting" << std::endl;
        measured.insert(pairs(indexX, indexY));
      }

      float avg_i = 0;
      int count = 1;
      float avg_j = 0;

      for (auto& pair: measured){
        int i = pair.first;
        int j = pair.second;
        avg_i = (count - 1.0)/count*avg_i + 1.0/count*i;
        avg_j = (count - 1.0)/count*avg_j + 1.0/count*j;
        count++;

        if (1 == (explored_)(i, j)){
          (elevation_)(i, j) = (elevation_)(i, j)*0.9 + current_measurement_(i, j)*0.1;
        } else {
          (elevation_)(i, j) = current_measurement_(i, j);
          (explored_)(i, j) = 1;
        }
        update_traversability(i, j);
      }

      PointCloudType::Ptr pointCloudGrid(new PointCloudType);

      pointCloudGrid->header.frame_id = map_frame_;
      pointCloudGrid->header = pointCloudTransformed->header;

      for (int i = int(avg_i)-vis_radius_cells_; i <= int(avg_i) + vis_radius_cells_; i++)
      {
        for (int j = int(avg_j)-vis_radius_cells_; j <= int(avg_j) + vis_radius_cells_; j++)
        {
          if (!isIndexValid(i, j)){
            continue;
          }
          if (!(explored_)(i, j)){
            continue;
          }

          pairf xy_coordinate = indexToPosition(pairs(i, j));
          float coordinateX = xy_coordinate.first; 
          float coordinateY = xy_coordinate.second; 
          pcl::PointXYZI p(1.0);
          p.x = coordinateX;
          p.y = coordinateY;
          p.z = elevation_.coeff(i, j);
          pointCloudGrid->insert(pointCloudGrid->end(), p);
        }
      }
      
      
      pcl::toPCLPointCloud2(*pointCloudGrid, pcl_pc);
      pcl_conversions::fromPCL (pcl_pc, output);

      // Publish the data
      pub_.publish (output);

      PointCloudType::Ptr pcl_trav(new PointCloudType);
      pcl_trav->header.frame_id = map_frame_;
      pcl_trav->header = pointCloudTransformed->header;

      for (int i = int(avg_i)-vis_radius_cells_; i <= int(avg_i) + vis_radius_cells_; i++)
      {
        for (int j = int(avg_j)-vis_radius_cells_; j <= int(avg_j) + vis_radius_cells_; j++)
        {
          if (!isIndexValid(i, j)){
              continue;
            }
          if ( !(explored_)(i, j) || (traversability_(i, j) == -1) ){
            continue;
          }
          pairf xy_coordinate = indexToPosition(pairs(i, j));
          float coordinateX = xy_coordinate.first; 
          float coordinateY = xy_coordinate.second;
          pcl::PointXYZI p(traversability_(i, j));
          p.x = coordinateX;
          p.y = coordinateY;
          p.z = 0;
          pcl_trav->insert(pcl_trav->end(), p);
        }
      }

      pcl::toPCLPointCloud2(*pcl_trav, pcl_pc);
      pcl_conversions::fromPCL (pcl_pc, output);

      // Publish the data
      pub_trav_.publish(output);

    
    }

    bool isIndexValid(int i, int j){
      return (i > 0) && (j>0) && (i<n_cells1_) && (j<n_cells2_);
    }

    bool update_traversability(int i, int j){
      if (!isIndexValid(i-1, j) || !isIndexValid(i, j-1) || !isIndexValid(i, j))
        return false;
      if (explored_(i, j) && explored_(i-1, j) && explored_(i, j-1)){
        float v = elevation_(i, j);
        float v_up = elevation_(i-1, j);
        float v_right = elevation_(i, j-1);
        float diff1 = std::fabs(v - v_up);
        float diff2 = std::fabs(v - v_right);
        float slope = std::sqrt(diff1*diff1 + diff2*diff2);
        int value = 0; // traversable
        if (slope > slope_th_){
          value = 2; // untraversable
        }
        traversability_(i, j) = value;
      }
      else{
        traversability_(i, j) = -1; // unknown
      }
      return true;
    }

    bool init_submap(){
      geometry_msgs::TransformStamped transformTf;
      ros::Time timeStamp;
      timeStamp.fromSec(0.0);
      try {
        transformTf = tf_buffer_->lookupTransform(map_frame_, init_submap_frame_, timeStamp, ros::Duration(5.0));
      } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());

        std::cout << "Init submap: failed to get a transform" << std::endl;
        return false;
      }
      float x_coord = transformTf.transform.translation.x ;
      float y_coord = transformTf.transform.translation.y ;
      float z_coord = transformTf.transform.translation.z - init_submap_height_offset_;
      pairs xy_ind = positionToIndex(pairf(x_coord, y_coord));
      int x_ind = xy_ind.first;
      int y_ind = xy_ind.second;
      std::cout << x_coord<< " " << y_coord << " " << z_coord << std::endl;

      int radius_inds = int(init_submap_radius_/resolution_);
      if (!radius_inds){
        std::cout << "Init submap radius is 0" << std::endl;
        return false;
      }
      
      for (int i = -radius_inds; i <= radius_inds; i++)
      {
        for (int j = -radius_inds; j <= radius_inds; j++)
        {
          int cellX = x_ind + i;
          int cellY = y_ind + j;
          if(!isIndexValid(cellX, cellY))
            continue;
          if (i*i + j*j >= radius_inds*radius_inds){ // only inside a circle
            continue;
          }
          elevation_(cellX, cellY) = z_coord;
          explored_(cellX, cellY) = 1;
          elevation_time_estimated_(cellX, cellY) = ros::Time::now().toNSec()/1000000 - 1;
          update_traversability(cellX, cellY);
        }
      }
      return true;
    }

    pairs positionToIndex(pairf position){
      int i = int((position.first - origin1_)/resolution_) + n_cells1_/2;
      int j = int((position.second - origin2_)/resolution_) + n_cells2_/2;
      return pairs(i, j);
    }

    pairf indexToPosition(pairs position){
      float x = (position.first - n_cells1_/2)*resolution_ + origin1_;
      float y = (position.second - n_cells2_/2)*resolution_ + origin2_;
      return pairf(x, y);
    }

};


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "my_elevation_map");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ElevationMapper mapper(nh, pnh);

  // Spin
  ros::spin ();
}