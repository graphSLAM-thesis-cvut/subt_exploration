#ifndef WF_CLUSTERING
#define WF_CLUSTERING

#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Core>

using pairs = std::pair<int, int>;
namespace wf_clustering
{

    bool is_index_valid(int i, int j, const Eigen::MatrixXf &mat);

    bool is_explored(int i, int j, const Eigen::MatrixXf &travers, const Eigen::MatrixXi &explored);

    bool cluster_map(Eigen::MatrixXf &trav, float travTh, pairs pose_start, Eigen::MatrixXi &explored, std::vector<std::vector<pairs>>& result);
}

#endif