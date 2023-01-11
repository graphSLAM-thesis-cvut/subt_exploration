#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include "frontier.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Core>

// TODO: 
// - make obstacles wider - DONE
// - specify maximum frontier length - DONE 
// - extract one point interest from frontier
// - specify maximum frontier expantion length ?
// - put all the parameters to the yaml file or take them as an input
// - maybe memorize previous frontiers


using namespace std;

const int MAP_OPEN_LIST = 1, MAP_CLOSE_LIST = 2, FRONTIER_OPEN_LIST = 3, FRONTIER_CLOSE_LIST = 4;
const float OCC_THRESHOLD = 0.2;

using pairs = std::pair<int, int>;


const int N_S = 8;

const int offsetx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
const int offsety[8] = {1, -1, 0, 0, 1, -1, 1, -1};

const int N_S4 = 4;

const int offsetx4[N_S] = {0, 0, 1, -1};
const int offsety4[N_S] = {1, -1, 0, 0};

const int MIN_FOUND = 1;

vector<vector<pairs>> wfd(const Eigen::MatrixXf& h_diffs, Eigen::MatrixXf& h_diffs_expanded, const Eigen::MatrixXi& explored, int posex, int posey, int robot_size_cells, int max_frontier_length_cells) {
    expand(h_diffs, h_diffs_expanded, explored, posex, posey, robot_size_cells);

	pairs pose_start(posex, posey);
	vector<vector<pairs> > frontiers;
	// Cell state list for map/frontier open/closed
	int map_size = h_diffs.rows() * h_diffs.cols();
	std::map<pairs, int> cell_states;
	//
	queue<pairs> q_m;	
	q_m.push(pose_start);
	cell_states[pose_start] = MAP_OPEN_LIST;
	int adj_vector[N_S4];
	int v_neighbours[N_S4];
	//
	// ROS_INFO("wfd 1");
	while(!q_m.empty()) {
		// ROS_INFO("wfd 2");
		pairs cur_pos = q_m.front();
		q_m.pop();
		// ROS_INFO("cur_pos: %d %d, cell_state: %d", cur_pos.first, cur_pos.second, cell_states[cur_pos]);
		// Skip if map_close_list
		if(cell_states[cur_pos] == MAP_CLOSE_LIST)
			continue;
		if(is_frontier_point(h_diffs_expanded, explored, cur_pos)) {
			queue<pairs> q_f;
			vector<pairs> new_frontier;

            int frontier_size = 0;

			q_f.push(cur_pos);
			cell_states[cur_pos] = FRONTIER_OPEN_LIST;
			// Second BFS
			while(!q_f.empty()) {
				// ROS_INFO("wfd 3");
				// ROS_INFO("Size: %d", int(q_f.size()));
				pairs n_cell = q_f.front();
				q_f.pop();
				//
				if(cell_states[n_cell] == MAP_CLOSE_LIST || cell_states[n_cell] == FRONTIER_CLOSE_LIST)
					continue;
				//
				if(is_frontier_point(h_diffs_expanded, explored, n_cell)) {
					// ROS_INFO("adding %d %d to frontiers", n_cell.first, n_cell.second);
					new_frontier.push_back(n_cell);
                    frontier_size ++;
                    if(frontier_size > max_frontier_length_cells){

				        cell_states[n_cell] = FRONTIER_CLOSE_LIST;
                        break;
                    }
					// get_neighbours(adj_vector, cur_pos);			
					//
					// ROS_INFO("wfd 3.5");
                    int i;
                    int j;
					for(int t = 0; t < N_S; t++) {
                        i = n_cell.first  + offsetx[t];
                        j = n_cell.second + offsety[t];
                        pairs ij(i, j);
						if(is_index_valid(i, j, h_diffs_expanded)) {
							if(cell_states[ij] != FRONTIER_OPEN_LIST && 
								cell_states[ij] != FRONTIER_CLOSE_LIST && 
								cell_states[ij] != MAP_CLOSE_LIST) {
								// ROS_INFO("wfd 4");
								if(h_diffs_expanded(i, j) < OCC_THRESHOLD) { //if(h_diffs_expanded(i, j) != 100) {
								    // ROS_INFO("wfd 5");
									q_f.push(ij);
									cell_states[ij] = FRONTIER_OPEN_LIST;
								}
							}
						}
					}
				}
				cell_states[n_cell] = FRONTIER_CLOSE_LIST;
			}
			if(new_frontier.size() > 2)
				frontiers.push_back(new_frontier);
			
			// ROS_INFO("WFD 4.5");
			for(unsigned int i = 0; i < new_frontier.size(); i++) {
				cell_states[new_frontier[i]] = MAP_CLOSE_LIST;
				// ROS_INFO("WFD 5");
			}
		}
		//
		// get_neighbours(adj_vector, cur_pos, map_width);
        int i1, j1, i2, j2;
		for (int t1 = 0; t1 < N_S4; ++t1) {
            i1 = cur_pos.first + offsetx4[t1];
            j1 = cur_pos.second + offsety4[t1];
            pairs ij1(i1, j1);
			// ROS_INFO("wfd 6");
			if(is_index_valid(i1, j1, h_diffs_expanded)) {
				if(cell_states[ij1] != MAP_OPEN_LIST &&  cell_states[ij1] != MAP_CLOSE_LIST && h_diffs_expanded(i1, j1) < OCC_THRESHOLD && h_diffs_expanded(i1, j1) >=0 ) {

			        // ROS_INFO("wfd 6.1");
					// get_neighbours(v_neighbours, adj_vector[i], map_width);
					bool map_open_neighbor = false;
					for(int t2 = 0; t2 < N_S4; t2++) {
                        i2 = i1 + offsetx4[t2];
                        j2 = j1 + offsety4[t2];
                        pairs ij2(i2, j2);

						if(is_index_valid(i2, j2, h_diffs_expanded)) {

			                // ROS_INFO("wfd 6.3");
							if(h_diffs_expanded(i2, j2) < OCC_THRESHOLD && h_diffs_expanded(i2, j2) >= 0 && explored(i2, j2) ) { //>= 0 AANPASSING
                                // ROS_INFO("wfd 6.4");
								map_open_neighbor = true;
								break;
							}
						}
					}
					if(map_open_neighbor) {
						q_m.push(ij1);
						cell_states[ij1] = MAP_OPEN_LIST;
					}
				}
			}
		}
		// ROS_INFO("wfd 7");
		cell_states[cur_pos] = MAP_CLOSE_LIST;
		// ROS_INFO("wfd 7.1");
	}
	// ROS_INFO("wfd 8");
	return frontiers;
}

void expand(const Eigen::MatrixXf& h_diffs, Eigen::MatrixXf& h_diffs_expanded, const Eigen::MatrixXi& explored, int posex, int posey, int robot_size_cells){
    pairs pose_start(posex, posey);
	// Eigen::MatrixXf& h_diffs
	// Cell state list for map/frontier open/closed
	int map_size = h_diffs.rows() * h_diffs.cols();
	std::map<pairs, int> cell_states;
	//
	queue<pairs> q_m;	
	q_m.push(pose_start);
	cell_states[pose_start] = MAP_OPEN_LIST;
	int adj_vector[N_S4];
	int v_neighbours[N_S4];
	//
	ROS_INFO("expanding the map's obstacles 1");
	while(!q_m.empty()) {
		// ROS_INFO("wfd 2");
		pairs cur_pos = q_m.front();
		q_m.pop();
		// ROS_INFO("cur_pos: %d %d, cell_state: %d", cur_pos.first, cur_pos.second, cell_states[cur_pos]);
		// Skip if map_close_list
		if(cell_states[cur_pos] == MAP_CLOSE_LIST)
			continue;

        int i1, j1, i2, j2;
		for (int t1 = 0; t1 < N_S4; ++t1) {
            i1 = cur_pos.first + offsetx4[t1];
            j1 = cur_pos.second + offsety4[t1];
            pairs ij1(i1, j1);
			// ROS_INFO("wfd 6");
			if(is_index_valid(i1, j1, h_diffs)) {
				if(cell_states[ij1] != MAP_OPEN_LIST &&  cell_states[ij1] != MAP_CLOSE_LIST && h_diffs_expanded(i1, j1) < OCC_THRESHOLD && h_diffs_expanded(i1, j1) >=0 ) {

			        // ROS_INFO("wfd 6.1");
					// get_neighbours(v_neighbours, adj_vector[i], map_width);
					bool map_open_neighbor = false;
					for(int t2 = 0; t2 < N_S4; t2++) {
                        i2 = i1 + offsetx4[t2];
                        j2 = j1 + offsety4[t2];
                        pairs ij2(i2, j2);

						if(is_index_valid(i2, j2, h_diffs)) {

			                // ROS_INFO("wfd 6.3");
							if(h_diffs_expanded(i2, j2) < OCC_THRESHOLD && h_diffs_expanded(i2, j2) >= 0 && explored(i2, j2) ) { //>= 0 AANPASSING
                                // ROS_INFO("wfd 6.4");
								map_open_neighbor = true;
								break;
							} //else if (h_diffs(i2, j2) > OCC_THRESHOLD && explored(i2, j2))
                                //increase_boudary(h_diffs, h_diffs_expanded, explored, )
						}
					}
					if(map_open_neighbor) {
						q_m.push(ij1);
						cell_states[ij1] = MAP_OPEN_LIST;
					}
				} else if (explored(i1, j1) && h_diffs(i1, j1) > OCC_THRESHOLD )
                    increase_boudary(h_diffs, h_diffs_expanded, explored, i1, j1, robot_size_cells);
			}
		}
		// ROS_INFO("wfd 7");
		cell_states[cur_pos] = MAP_CLOSE_LIST;
		// ROS_INFO("wfd 7.1");
	}
}

void increase_boudary(const Eigen::MatrixXf& h_diffs, Eigen::MatrixXf& h_diffs_expanded, const Eigen::MatrixXi& explored, int row, int col, int robot_size_cells){
    float value = h_diffs(row, col);
    for(int di = -robot_size_cells; di <= robot_size_cells; di++){
        for(int dj = -robot_size_cells; dj <= robot_size_cells; dj++){
            int cellX = row + di;
            int cellY = col + dj;
            if(!is_index_valid(cellX, cellY, h_diffs))
                continue;
            if (di*di + dj*dj > robot_size_cells*robot_size_cells + 1){ // only inside a circle
                continue;
            }
            if(!explored(cellX, cellY))
                continue;
            if (h_diffs(row, col) > h_diffs_expanded(cellX, cellY)){
                h_diffs_expanded(cellX, cellY) = h_diffs(row, col);
            } 
        }
    }
}

bool is_frontier_point(const Eigen::MatrixXf& mat, const Eigen::MatrixXi& explored, pairs point) {
	// The point under consideration must be known
	if(!is_explored(point.first, point.second, mat, explored) || mat(point.first, point.second) > OCC_THRESHOLD ) {
		return false;
	}

	int found = 0;
    int col;
    int row;
	for(int i = 0; i < N_S; i++) {
        row = offsetx[i] + point.first;
        col = offsety[i] + point.second;
		if(is_index_valid(row, col, mat)) {
			// None of the neighbours should be occupied space.		
			if(mat(row, col) > OCC_THRESHOLD && is_explored(row, col, mat, explored)) {
				return false;
			}
			//At least one of the neighbours is open and known space, hence frontier point
			if(!is_explored(row, col,  mat, explored)) {
				found++;
				//
				if(found == MIN_FOUND) 
					return true;
			}
		}
	}
	return false;
}

bool is_index_valid(int i, int j, const Eigen::MatrixXf& mat){
    return ( i >= 0 && j>=0 && i<mat.rows() && j<mat.cols() );
}

bool is_explored(int i, int j,  const Eigen::MatrixXf& travers, const Eigen::MatrixXi& explored){
    if (travers(i, j) < 0)
        return false;
    return (explored(i, j) == 1);
}