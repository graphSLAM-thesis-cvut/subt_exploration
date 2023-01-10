#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Core>

using pairs = std::pair<int, int>;

// void get_neighbours(int n_array[], int position, int map_width);
// nav_msgs::OccupancyGrid downSizeMap(const nav_msgs::OccupancyGrid& map, int width, int height);
bool is_frontier_point(const Eigen::MatrixXf& mat, pairs point);
// int get_row_from_offset(int offset, int width);
// int get_column_from_offset(int offset, int width);
std::vector<std::vector<pairs>> wfd(const Eigen::MatrixXf& mat, int posex, int posey);
bool is_index_valid(int i, int j, const Eigen::MatrixXf& mat);