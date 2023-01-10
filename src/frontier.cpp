#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include "frontier.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Core>

using namespace std;

const int MAP_OPEN_LIST = 1, MAP_CLOSE_LIST = 2, FRONTIER_OPEN_LIST = 3, FRONTIER_CLOSE_LIST = 4;
const int OCC_THRESHOLD = 10;
const int N_S = 4;

using pairs = std::pair<int, int>;

const int offsetx[4] = {0, 0, 1, -1};
const int offsety[4] = {1, -1, 0, 0};
const int MIN_FOUND = 1;

vector<vector<pairs>> wfd(const Eigen::MatrixXf& mat, int posex, int posey) {	
	pairs pose_start(posex, posey);
	vector<vector<pairs> > frontiers;
	// Cell state list for map/frontier open/closed
	int map_size = mat.rows() * mat.cols();
	std::map<pairs, int> cell_states;
	//
	queue<pairs> q_m;	
	q_m.push(pose_start);
	cell_states[pose_start] = MAP_OPEN_LIST;
	int adj_vector[N_S];
	int v_neighbours[N_S];
	//
	//ROS_INFO("wfd 1");
	while(!q_m.empty()) {
		//ROS_INFO("wfd 2");
		pairs cur_pos = q_m.front();
		q_m.pop();
		//ROS_INFO("cur_pos: %d, cell_state: %d",cur_pos, cell_states[cur_pos]);
		// Skip if map_close_list
		if(cell_states[cur_pos] == MAP_CLOSE_LIST)
			continue;
		if(is_frontier_point(mat, cur_pos)) {
			queue<pairs> q_f;
			vector<pairs> new_frontier;
			q_f.push(cur_pos);
			cell_states[cur_pos] = FRONTIER_OPEN_LIST;
			// Second BFS
			while(!q_f.empty()) {
				//ROS_INFO("wfd 3");
				//ROS_INFO("Size: %d", q_f.size());
				pairs n_cell = q_f.front();
				q_f.pop();
				//
				if(cell_states[n_cell] == MAP_CLOSE_LIST || cell_states[n_cell] == FRONTIER_CLOSE_LIST)
					continue;
				//
				if(is_frontier_point(mat, n_cell)) {
					//ROS_INFO("adding %d to frontiers", n_cell);
					new_frontier.push_back(n_cell);
					// get_neighbours(adj_vector, cur_pos);			
					//
					//ROS_INFO("wfd 3.5");
                    int i;
                    int j;
					for(int t = 0; t < N_S; t++) {
                        i = cur_pos.first  + offsetx[t];
                        j = cur_pos.second + offsety[t];
                        pairs ij(i, j);
						if(is_index_valid(i, j, mat)) {
							if(cell_states[ij] != FRONTIER_OPEN_LIST && 
								cell_states[ij] != FRONTIER_CLOSE_LIST && 
								cell_states[ij] != MAP_CLOSE_LIST) {
								//ROS_INFO("wfd 4");
								if(mat(i, j) != 100) {
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
			
			//ROS_INFO("WFD 4.5");
			for(unsigned int i = 0; i < new_frontier.size(); i++) {
				cell_states[new_frontier[i]] = MAP_CLOSE_LIST;
				//ROS_INFO("WFD 5");
			}
		}
		//
		// get_neighbours(adj_vector, cur_pos, map_width);
        int i1, j1, i2, j2;
		for (int t1 = 0; t1 < N_S; ++t1) {
            i1 = offsetx[t1];
            j1 = offsety[t1];
            pairs ij1(i1, j1);
			//ROS_INFO("wfd 6");
			if(is_index_valid(i1, j1, mat)) {
				if(cell_states[cur_pos] != MAP_OPEN_LIST &&  cell_states[cur_pos] != MAP_CLOSE_LIST) {
					// get_neighbours(v_neighbours, adj_vector[i], map_width);
					bool map_open_neighbor = false;
					for(int t2 = 0; t2 < N_S; t2++) {
                        i2 = offsetx[t2];
                        j2 = offsety[t2];
                        pairs ij2(i2, j2);

						if(is_index_valid(i2, j2, mat)) {
							if(mat(i2, j2) < OCC_THRESHOLD && mat(i2, j2) >= 0) { //>= 0 AANPASSING
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
		//ROS_INFO("wfd 7");
		cell_states[cur_pos] = MAP_CLOSE_LIST;
		//ROS_INFO("wfd 7.1");
	}
	// ROS_INFO("wfd 8");
	return frontiers;
}



bool is_frontier_point(const Eigen::MatrixXf& mat, pairs point) {
	// The point under consideration must be known
	if(mat(point.first, point.second) != -1.0) {
		return false;
	}
	//
	// int locations[N_S]; 
	// get_neighbours(locations, point, map_width);
	int found = 0;
    int col;
    int row;
	for(int i = 0; i < N_S; i++) {
        row = offsetx[i] + point.first;
        col = offsety[i] + point.second;
		if(is_index_valid(row, col, mat)) {
			// None of the neighbours should be occupied space.		
			if(mat(row, col) > OCC_THRESHOLD) {
				return false;
			}
			//At least one of the neighbours is open and known space, hence frontier point
			if(mat(row, col) == 0) {
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