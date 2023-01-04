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
ros::Publisher travPub;
double slope_th = 0.1;
bool start = true;



void gridmap_callback(const grid_map_msgs::GridMap& msg){
  if (start){
    std::cout << "Got a first Gridmap! Publishing Traversability" << std::endl;
    start = false;
  }
  grid_map::GridMap gridMap;
  nav_msgs::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::fromMessage(msg, gridMap);
  grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "elevation", 0.0, 1.0, occupancyGrid);
  m_mapPub.publish(occupancyGrid);
  Eigen::MatrixXf m = gridMap.get("elevation");
  Eigen::MatrixXf m1(m);
  int a = 0;

  int nCells = m.cols()*m.rows();
  for (int i = 1; i < m.cols() - 1 ; i++)
  {
    for (int j = 1; j < m.rows() - 1; j++)
    {
      a++;
      float v = m.coeff(i, j);
      size_t index = m.cols()*m.rows() - (j*m.rows() + i) - 1;
      if (!isnan(v)){
        float v_up = m.coeff(i+1, j);
        float v_right = m.coeff(i, j+1);
        float diff1 = std::fabs(v - v_up);
        float diff2 = std::fabs(v - v_right);
        float slope = std::sqrt(diff1*diff1 + diff2*diff2);
        int value = 0;
        if (slope > slope_th){
          value = 100;
        }
        occupancyGrid.data[index] = value;
      }
      else{
        occupancyGrid.data[index] = -1;
      }
    }
  }
  travPub.publish(occupancyGrid);
}



int main(int argc, char **argv){

  ros::init(argc, argv, "gridmapToCostmap");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("gridmap_topic", topic);
  private_nh.getParam("slope_th", slope_th);

  std::cout << "Gridmap Topic: " << topic << std::endl;
  std::cout << "Slope threshhold: " << slope_th << std::endl;


  m_mapPub = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5);
  travPub = nh.advertise<nav_msgs::OccupancyGrid>("trav_map", 5);
  ros::Subscriber grid_map_sub = nh.subscribe(topic, 10, gridmap_callback);
  ros::spin();

  return 0;
}