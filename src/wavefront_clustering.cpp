#include "wavefront_clustering.hpp"

namespace wf_clustering
{
    const int MAP_OPEN_LIST = 1, MAP_CLOSE_LIST = 2, FRONTIER_OPEN_LIST = 3, FRONTIER_CLOSE_LIST = 4;
    // const float travTh = 0.1;

    using pairs = std::pair<int, int>;

    const int N_S = 8;

    const int offsetx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    const int offsety[8] = {1, -1, 0, 0, 1, -1, 1, -1};

    const int N_S4 = 4;

    const int offsetx4[N_S] = {0, 0, 1, -1};
    const int offsety4[N_S] = {1, -1, 0, 0};

    const int MIN_FOUND = 1;

    bool is_index_valid(int i, int j, const Eigen::MatrixXf &mat)
    {
        return (i >= 0 && j >= 0 && i < mat.rows() && j < mat.cols());
    }

    bool is_explored(int i, int j, const Eigen::MatrixXf &travers, const Eigen::MatrixXi &explored)
    {
        if (travers(i, j) < 0)
            return false;
        return (explored(i, j) == 1);
    }

    bool cluster_map(Eigen::MatrixXf &trav, float travTh, pairs pose_start, Eigen::MatrixXi &explored, std::vector<std::vector<pairs>>& result)
    {
        // std::vector<std::vector<pairs>>;
        std::vector<pairs> clear_cells;
        std::vector<pairs> obstacles;
        // std::vector<pairs> frontier_points;

        // Eigen::MatrixXf& h_diffs
        // Cell state list for map/frontier open/closed
        int map_size = trav.rows() * trav.cols();
        std::map<pairs, int> cell_states;
        //
        std::queue<pairs> q_m;
        q_m.push(pose_start);
        cell_states[pose_start] = MAP_OPEN_LIST;
        int adj_vector[N_S4];
        int v_neighbours[N_S4];
        //
        ROS_INFO("clustering some traversability map");
        while (!q_m.empty())
        {
            // ROS_INFO("wfd 2");
            pairs cur_pos = q_m.front();
            q_m.pop();
            // ROS_INFO("cur_pos: %d %d, cell_state: %d", cur_pos.first, cur_pos.second, cell_states[cur_pos]);
            // Skip if map_close_list
            if (cell_states[cur_pos] == MAP_CLOSE_LIST)
                continue;

            int i1, j1, i2, j2;
            for (int t1 = 0; t1 < N_S4; ++t1)
            {
                i1 = cur_pos.first + offsetx4[t1];
                j1 = cur_pos.second + offsety4[t1];
                pairs ij1(i1, j1);
                // ROS_INFO("wfd 6");
                if (is_index_valid(i1, j1, trav) && cell_states[ij1] != MAP_CLOSE_LIST && cell_states[ij1] != MAP_OPEN_LIST) // any valid unvisited cell
                {
                    // trav(i1, j1) = trav(i1, j1); // filling the values on the way
                    if (explored(i1, j1) == 1 && trav(i1, j1) < travTh && trav(i1, j1) >= 0)
                    { // propagate through the traversable cells
                        q_m.push(ij1);
                        cell_states[ij1] = MAP_OPEN_LIST;
                        clear_cells.push_back(ij1);
                    } 
                    else if (explored(i1, j1) == 1 && trav(i1, j1) > travTh){
                        obstacles.push_back(ij1);
                    }
                    // else if ((explored(i1, j1) == 0 || trav(i1, j1) == -1.0) && unknownIsObstacle) // increase boundaries if unknown and planning
                    // 	increase_boudary(trav, trav, explored, i1, j1, robot_size_cells, cell_states);
                }
            }
            // ROS_INFO("wfd 7");
            cell_states[cur_pos] = MAP_CLOSE_LIST;
            // ROS_INFO("wfd 7.1");
        }
        result.push_back(clear_cells);
        result.push_back(obstacles);

        ROS_INFO("clusteres traversability map");
        return true;
    }
}
