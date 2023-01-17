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

#include "frontier.hpp"
#include "planner.hpp"
#include <std_srvs/Empty.h>
#include "nav_msgs/Path.h"
// #include <std_srvs/SetBoolResponse.h>
// #include <utility>

const int MAP_OPEN_LIST = 1, MAP_CLOSE_LIST = 2;

const int N_S4 = 4;

const int offsetx4[N_S4] = {0, 0, 1, -1};
const int offsety4[N_S4] = {1, -1, 0, 0};

const int N_S = 8;

const int offsetx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
const int offsety[8] = {1, -1, 0, 0, 1, -1, 1, -1};

typedef std::pair<int, int> pairs;
typedef std::pair<float, float> pairf;

using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;

class ElevationMapper
{
public:
  ElevationMapper(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle, float startPosition[3]): nodeHandle_(nodeHandle), pnh_(privateNodeHandle)
  {
    readPearameters();
    // tf_buffer_ = new tf2_ros::Buffer();
    // transformListener_ = new tf2_ros::TransformListener(*tf_buffer_);
    initTimeMs = ros::Time::now().toNSec() / 1000000;
    if (init_submap_)
    {
      bool inited = init_submap(startPosition);
      std::cout << "Submap initialized successfully: " << inited << std::endl;
    }
  }
  ~ElevationMapper()
  {
    nodeHandle_.shutdown();
  }

  // private:
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle pnh_;
  float origin1_ = 0;
  float origin2_ = 0;

  pairs robotIndex = pairs(-1, -1);

  int size1_ = 20;
  int size2_ = 20;
  float resolution_ = 0.1;
  int n_cells1_ = -1;
  int n_cells2_ = -1;
  uint32_t initTimeMs;

  ros::Time last_update;

  float vis_radius_ = 6.0;
  int vis_radius_cells_ = 60;

  bool init_submap_ = false;
  float init_submap_height_offset_ = 0.0;
  float init_submap_radius_ = 0.0;

  bool map_used_ = false;

  float slope_th_ = 0.1;

  Eigen::MatrixXf elevation_;
  Eigen::MatrixXi elevation_time_estimated_; // To get the highest point from the cell. TODO: maybe substitude with kd-tree ?
  Eigen::MatrixXf current_measurement_;      // To get the highest point from the cell. TODO: maybe substitude with kd-tree ?
  Eigen::MatrixXi explored_;
  Eigen::MatrixXf traversability_;
  Eigen::MatrixXf traversability_expanded_;

  std::vector<pairf> interest_points_;
  std::vector<pairf> explored_interest_points_;

  float robot_size_ = resolution_;
  int robot_size_cells_;

  float max_frontier_length_ = 3.0;
  int max_frontier_lenght_cells_;

  float min_frontier_length_ = 1.0;
  int min_frontier_size_;

  float explored_point_radius_ = 0.5;
  int explored_point_radius_cells_;

  int explored_point_radius_ency_;


  std::vector<pairs> current_path_;
  std::vector<pairs> current_interest_points_;

  // ros::ServiceServer service;

  int offsetx_[4] = {0, 1, 0, -1};
  int offsety_[4] = {1, 0, -1, 0};

  // methods

  bool readPearameters()
  {
    pnh_.getParam("origin1", origin1_);
    pnh_.getParam("origin2", origin2_);
    pnh_.getParam("size1", size1_);
    pnh_.getParam("size2", size2_);
    pnh_.getParam("resolution", resolution_);
    pnh_.getParam("vis_radius", vis_radius_);
    pnh_.getParam("init_submap", init_submap_);
    pnh_.getParam("init_submap_radius", init_submap_radius_);
    pnh_.getParam("init_submap_height_offset", init_submap_height_offset_);
    pnh_.getParam("slope_th", slope_th_);
    pnh_.getParam("robot_size", robot_size_);
    pnh_.getParam("max_frontier_length", max_frontier_length_);
    pnh_.getParam("explored_point_radius", explored_point_radius_);

    std::cout << "origin1: " << origin1_ << std::endl;
    std::cout << "origin2: " << origin2_ << std::endl;
    std::cout << "size1: " << size1_ << std::endl;
    std::cout << "size2: " << size2_ << std::endl;
    std::cout << "resolution: " << resolution_ << std::endl;
    std::cout << "vis_radius: " << vis_radius_ << std::endl;
    std::cout << "init_submap: " << init_submap_ << std::endl;
    std::cout << "init_submap_radius: " << init_submap_radius_ << std::endl;
    std::cout << "init_submap_height_offset: " << init_submap_height_offset_ << std::endl;
    std::cout << "Slope threshhold: " << slope_th_ << std::endl;
    std::cout << "Robot size: " << robot_size_ << std::endl;
    std::cout << "Max frontier lenth: " << max_frontier_length_ << std::endl;
    std::cout << "Min frontier lenth: " << min_frontier_length_ << std::endl;
    std::cout << "explored_point_radius " << explored_point_radius_ << std::endl;

    n_cells1_ = int(size1_ / resolution_);
    n_cells2_ = int(size2_ / resolution_);
    vis_radius_cells_ = int(vis_radius_ / resolution_);
    robot_size_cells_ = int(robot_size_ / resolution_);
    max_frontier_lenght_cells_ = int(max_frontier_length_ / resolution_);
    min_frontier_size_ = int(min_frontier_length_ / resolution_);
    explored_point_radius_cells_ = int(explored_point_radius_ / resolution_);

    std::cout << "INITIALIZING MATRICIES" << std::endl;
    elevation_.resize(n_cells1_, n_cells2_);
    explored_.resize(n_cells1_, n_cells2_);
    elevation_time_estimated_.resize(n_cells1_, n_cells2_);
    current_measurement_.resize(n_cells1_, n_cells2_);
    traversability_.resize(n_cells1_, n_cells2_);
    traversability_expanded_.resize(n_cells1_, n_cells2_);

    std::cout << "n_cells1: " << n_cells1_ << std::endl;
    std::cout << "n_cells2: " << n_cells2_ << std::endl;
    std::cout << "vis_radius_cells: " << vis_radius_cells_ << std::endl;
    std::cout << "robot_size_cells_: " << robot_size_cells_ << std::endl;
    std::cout << "max_frontier_lenght_cells_: " << max_frontier_lenght_cells_ << std::endl;
    std::cout << "min_frontier_size_: " << min_frontier_size_ << std::endl;
    std::cout << "explored_point_radius_cells_: " << explored_point_radius_cells_ << std::endl;

    for (size_t i = 0; i < n_cells1_; i++)
    {
      for (size_t j = 0; j < n_cells2_; j++)
      {
        explored_(i, j) = 0;
      }
    }

    std::cout << "MATRICIES initialized" << std::endl;

    return true;
  }

  void insert_cloud(PointCloudType::Ptr pointCloudTransformed, uint32_t cur_time){

    // aggregate measurements: maximum height for each updated cell
    std::set<pairs> measured;

    for (size_t i = 0; i < pointCloudTransformed->size(); i++)
    {
      // std::cout << "inserting new measurement ";
      auto &point = pointCloudTransformed->points[i];

      // get point index
      pairs xy_index = positionToIndex(pairf(point.x, point.y));
      int indexX = xy_index.first;
      int indexY = xy_index.second;

      // filter only valid indicies
      if (!isIndexValid(indexX, indexY))
      {
        ROS_ERROR_THROTTLE(10, "X, Y index out of range");
        continue;
      }

      // handle multiple measurements in the same lidar scan
      if (cur_time == elevation_time_estimated_(indexX, indexY)) // not the first point in the current scan
      {
        // std::cout << "old" << std::endl;
        current_measurement_(indexX, indexY) = std::max(point.z, current_measurement_(indexX, indexY));
      }
      else // first point in the current scan
      {
        // std::cout << "new" << std::endl;
        current_measurement_(indexX, indexY) = point.z;
        elevation_time_estimated_(indexX, indexY) = cur_time;
      }
      measured.insert(pairs(indexX, indexY));
    }

    // std::cout << "aggregating" << std::endl;
    for (auto &pair : measured)
    {
      
      // std::cout << "aggregating loop" << std::endl;
      int i = pair.first;
      int j = pair.second;

      // updating elevation map with the aggregated measurements
      if (1 == (explored_)(i, j)) // when explored, add new measurement
      {
        (elevation_)(i, j) = (elevation_)(i, j) * 0.9 + current_measurement_(i, j) * 0.1;
      }
      else // else set height to the measurement
      {
        (elevation_)(i, j) = current_measurement_(i, j);
        (explored_)(i, j) = 1;
      }

      update_traversability(i, j); // update traversability of the updated cell. TODO: update traversability after all the heights are updated
    }
  }

  bool isIndexValid(int i, int j)
  {
    return (i > 0) && (j > 0) && (i < n_cells1_) && (j < n_cells2_);
  }

  bool update_traversability(int i, int j)
  {
    float max_diff = 0.0;
    int di;
    int dj;
    float v = elevation_(i, j);
    float diff;
    float v_cur;
    for (size_t t = 0; t < 4; t++)
    {
      di = offsetx_[t];
      dj = offsety_[t];
      if (!isIndexValid(i + di, j + dj))
        return false;
      if (!explored_(i + di, j + dj))
      {
        traversability_(i, j) = -1.0;
        traversability_expanded_(i, j) = -1.0;
        return true;
      }
      v_cur = elevation_(i + di, j + dj);
      diff = std::fabs(v - v_cur);
      max_diff = std::max(max_diff, diff);
    }
    traversability_(i, j) = max_diff;
    if (!map_used_)
      traversability_expanded_(i, j) = max_diff;
    else
    {
      ROS_INFO("Not updating expanded traversability map as it is being used");
    }
    return true;
  }

  pairs getRobotCell()
  {
    return robotIndex;
  }

  pairs get_nearest_traversable_cell(pairs check_cell)
  {

    std::map<pairs, int> cell_states;
    std::queue<pairs> q_m;
    q_m.push(check_cell);
    cell_states[check_cell] = MAP_OPEN_LIST;
    int adj_vector[N_S4];
    int v_neighbours[N_S4];
    //
    // ROS_INFO("wfd 1");
    while (!q_m.empty())
    {
      auto &current_cell = q_m.front();
      q_m.pop();
      if (cell_states[current_cell] == MAP_CLOSE_LIST)
        continue;

      int current_i = current_cell.first;
      int current_j = current_cell.second;
      for (int t = 0; t < N_S4; t++)
      {
        int i = current_i + offsetx4[t];
        int j = current_j + offsety4[t];
        auto ij = pairs(i, j);
        if (!isIndexValid(i, j))
          continue;
        if (cell_states[ij] == MAP_OPEN_LIST || cell_states[ij] == MAP_CLOSE_LIST)
          continue;
        if (explored_(i, j) && traversability_(i, j) >= 0 && traversability_expanded_(i, j) < slope_th_ && !is_frontier_point(ij))
          return pairs(i, j);
        q_m.push(ij);
        cell_states[ij] = MAP_OPEN_LIST;
      }
      cell_states[current_cell] = MAP_CLOSE_LIST;
    }
    return pairs(-1, -1);
  }

  bool is_frontier_point(pairs point)
  {
    // The point under consideration must be known
    if (!is_explored(point.first, point.second, traversability_expanded_, explored_) || traversability_expanded_(point.first, point.second) > slope_th_)
    {
      return false;
    }

    int found = 0;
    int col;
    int row;
    for (int i = 0; i < N_S; i++)
    {
      row = offsetx[i] + point.first;
      col = offsety[i] + point.second;
      if (is_index_valid(row, col, traversability_expanded_))
      {
        // None of the neighbours should be occupied space.
        if (traversability_expanded_(row, col) > slope_th_ && is_explored(row, col, traversability_expanded_, explored_))
        {
          return false;
        }
        // At least one of the neighbours is open and known space, hence frontier point
        if (!is_explored(row, col, traversability_expanded_, explored_))
        {
          found++;
          //
          if (found == 1)
            return true;
        }
      }
    }
    return false;
  }

  bool init_submap(float coords[3])
  {
    float x_coord = coords[0];
    float y_coord = coords[1];
    float z_coord = coords[2];
    pairs xy_ind = positionToIndex(pairf(x_coord, y_coord));
    int x_ind = xy_ind.first;
    int y_ind = xy_ind.second;
    std::cout << x_coord << " " << y_coord << " " << z_coord << std::endl;

    int radius_inds = int(init_submap_radius_ / resolution_);
    if (!radius_inds)
    {
      std::cout << "Init submap radius is 0" << std::endl;
      return false;
    }

    for (int di = -radius_inds; di <= radius_inds; di++)
    {
      for (int dj = -radius_inds; dj <= radius_inds; dj++)
      {
        int cellX = x_ind + di;
        int cellY = y_ind + dj;
        if (!isIndexValid(cellX, cellY))
          continue;
        if (di * di + dj * dj >= radius_inds * radius_inds)
        { // only inside a circle
          continue;
        }
        elevation_(cellX, cellY) = z_coord;
        explored_(cellX, cellY) = 1;
        elevation_time_estimated_(cellX, cellY) = ros::Time::now().toNSec() / 1000000 - 1;
        traversability_(cellX, cellY) = 0.0;
        traversability_expanded_(cellX, cellY) = 0.0;
      }
    }
    return true;
  }
 
  bool detect_frontiers()
  {
    std::cout << "Finding frontiers!" << std::endl;
    pairs robotPoint = getRobotCell();
    pairs startPoint = get_nearest_traversable_cell(robotPoint);
    std::cout << "Robot coordinates: " << startPoint.first << " " << startPoint.second << std::endl;
    if (startPoint.first < 0)
    {
      ROS_ERROR("Could find a traversable cell around robot");
      std::cout << "Could find a traversable cell around robot" << std::endl;
      return false;
    }
    std::vector<std::vector<pairs>> frontiers = wfd(traversability_, traversability_expanded_, explored_, startPoint.first, startPoint.second, robot_size_cells_, max_frontier_lenght_cells_, min_frontier_size_, slope_th_);
    std::cout << "Found " << frontiers.size() << " frontiers!" << std::endl;

    std::vector<pairs> interest_points;
    for (auto &f : frontiers)
    {
      pairs interest_point = get_interest_point(f);
      auto interest_pt_coord = indexToPosition(interest_point);
      // TODO: make as a parameter!!!
      if (interest_pt_coord.first < 12)
        continue;

      interest_points.push_back(interest_point);
    }

    current_interest_points_ = interest_points;

    if (interest_points.size() > 0)
    {
      pairs chosen_point = choose_point(interest_points);
      if (chosen_point.first < 0)
      {
        std::cout << "No exploration points left!" << std::endl;
      }
      else
      {
        current_path_ = plan(startPoint, chosen_point, 10, optimalPlanner::PLANNER_RRTSTAR, planningObjective::OBJECTIVE_WEIGHTEDCOMBO, traversability_expanded_, explored_, slope_th_);
      }
    }

    return true;
  }

  pairs positionToIndex(pairf position)
  {
    int i = int((position.first - origin1_) / resolution_) + n_cells1_ / 2;
    int j = int((position.second - origin2_) / resolution_) + n_cells2_ / 2;
    return pairs(i, j);
  }

  pairf indexToPosition(pairs position)
  {
    float x = (position.first - n_cells1_ / 2) * resolution_ + origin1_;
    float y = (position.second - n_cells2_ / 2) * resolution_ + origin2_;
    return pairf(x, y);
  }

  pairs get_interest_point(std::vector<pairs> &frontier)
  {
    pairs result;
    int c = 1;
    float avg_i = 0;
    float avg_j = 0;
    for (auto &p : frontier)
    {
      avg_i = (c - 1.0) / c * avg_i + float(p.first) / c;
      avg_j = (c - 1.0) / c * avg_j + float(p.second) / c;
      c++;
    }
    pairs avg_point = pairs(int(avg_i), int(avg_j));
    float min_d = INFINITY;
    for (auto &p : frontier)
    {
      float cur_d = std::pow(avg_point.first - p.first, 2) + std::pow(avg_point.second - p.second, 2);
      if (cur_d < min_d)
      {
        min_d = cur_d;
        result = p;
      }
    }
    return result;
  }

  pairs choose_point(std::vector<pairs> points)
  {
    for (auto &p : points)
    {
      bool suits = true;
      for (auto &ex_p : explored_interest_points_)
      {
        if (std::sqrt(std::pow(p.first - ex_p.first, 2) + std::pow(p.second - ex_p.second, 2)) < explored_point_radius_cells_)
        {
          suits = false;
          break;
        }
      }
      if (suits)
      {
        explored_interest_points_.push_back(p);
        return p;
      }
    }
    return pairs(-1, -1);
  }

  void updateRobotPosition(pairf position){
    robotIndex = positionToIndex(position);
  }
};

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "my_elevation_map");
//   ros::NodeHandle nh;
//   ros::NodeHandle pnh("~");

//   ElevationMapper mapper(nh, pnh);

//   // Spin
//   ros::spin();
// }