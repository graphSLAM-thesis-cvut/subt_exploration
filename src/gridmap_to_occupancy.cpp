#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

// Eigen
#include <Eigen/Core>

static std::string topic = "/elevation_mapping/elevation_map";

ros::Publisher m_mapPub;
// using namespace grid_map;

void gridmap_callback(const grid_map_msgs::GridMap& msg){
  std::cout << "Got a Gridmap!" << std::endl;
  grid_map::GridMap gridMap;
  nav_msgs::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::fromMessage(msg, gridMap);
  grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "elevation", 0.0, 1.0, occupancyGrid);
  m_mapPub.publish(occupancyGrid);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "gridmapToCostmap");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("gridmap_topic", topic);

  std::cout << "Gridmap Topic: " << topic << std::endl;

  m_mapPub = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5);
  ros::Subscriber grid_map_sub = nh.subscribe(topic, 10, gridmap_callback);
  ros::spin();

  return 0;
}