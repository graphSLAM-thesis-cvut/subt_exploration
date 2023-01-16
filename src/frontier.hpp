#ifndef MY_FRONTIER
#define MY_FRONTIER

#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Core>

using pairs = std::pair<int, int>;

bool is_frontier_point(const Eigen::MatrixXf& mat, const Eigen::MatrixXi& explored, pairs point, float trav_th);
bool is_explored(int i, int j,  const Eigen::MatrixXf& travers, const Eigen::MatrixXi& explored);
std::vector<std::vector<pairs>> wfd(const Eigen::MatrixXf& h_diffs, Eigen::MatrixXf& h_diffs_expanded, const Eigen::MatrixXi& explored, int posex, int posey, int robot_size_cells, int max_frontier_length_cells, int min_frontier_size, float trav_th);
void expand(const Eigen::MatrixXf& h_diffs, Eigen::MatrixXf& h_diffs_expanded, const Eigen::MatrixXi& explored, int posex, int posey, int robot_size_cells, float trav_th, bool unknownIsObstacle);
void increase_boudary(const Eigen::MatrixXf& h_diffs, Eigen::MatrixXf& h_diffs_expanded, const Eigen::MatrixXi& explored, int row, int col, int robot_size_cells, std::map<pairs, int>& cell_states);
bool is_index_valid(int i, int j, const Eigen::MatrixXf& mat);

#endif 