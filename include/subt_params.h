#ifndef SUBT_PARAMS_H
#define SUBT_PARAMS_H

#include <string>
#include "ros/ros.h"

namespace subt_params
{
    struct Params
    {

        std::string out_pcl_topic_ = "elevation";
        std::string transformed_pcl_topic_ = "pcl_glob";
        std::string pcl_topic_ = "COSTAR_HUSKY/points";
        std::string odom_topic_ = "/odom_gt";
        std::string map_frame_ = "COSTAR_HUSKY/odom";
        std::string init_submap_frame_ = "COSTAR_HUSKY";
        std::string travers_topic_ = "traversability";
        std::string frontiers_topic_ = "frontiers";
        std::string travers_expanded_topic_ = "travers_expanded";
        std::string travers_expanded_plan_topic_ = "travers_expanded_plan";
        std::string clearity_topic_ = "clearity";
        std::string path_topic_ = "rrt_path";

        float robot_size_ = resolution_;
        int robot_size_cells_;

        float max_frontier_length_ = 3.0;
        int max_frontier_lenght_cells_;

        float min_frontier_length_ = 1.0;
        int min_frontier_size_;

        float explored_point_radius_ = 0.5;
        int explored_point_radius_cells_;

        float origin1_ = 0;
        float origin2_ = 0;

        int size1_ = 20;
        int size2_ = 20;
        float resolution_ = 0.1;
        int n_cells1_ = -1;
        int n_cells2_ = -1;

        float vis_radius_ = 6.0;
        int vis_radius_cells_ = 60;

        bool init_submap_ = false;
        float init_submap_height_offset_ = 0.0;
        float init_submap_radius_ = 0.0;

        float slope_th_ = 0.1;

        int map_update_frequency_ = 50;
    };

    // ROS loader for the params
    inline Params load(ros::NodeHandle &pnh_)
    {
        Params ret;

        pnh_.getParam("origin1", ret.origin1_);
        pnh_.getParam("origin2", ret.origin2_);
        pnh_.getParam("size1", ret.size1_);
        pnh_.getParam("size2", ret.size2_);
        pnh_.getParam("resolution", ret.resolution_);
        pnh_.getParam("vis_radius", ret.vis_radius_);
        pnh_.getParam("init_submap", ret.init_submap_);
        pnh_.getParam("init_submap_radius", ret.init_submap_radius_);
        pnh_.getParam("init_submap_height_offset", ret.init_submap_height_offset_);
        pnh_.getParam("slope_th", ret.slope_th_);
        pnh_.getParam("robot_size", ret.robot_size_);
        pnh_.getParam("max_frontier_length", ret.max_frontier_length_);
        pnh_.getParam("explored_point_radius", ret.explored_point_radius_);
        pnh_.getParam("map_update_frequency", ret.map_update_frequency_);
        pnh_.getParam("out_pcl_topic", ret.out_pcl_topic_);
        pnh_.getParam("transformed_pcl_topic", ret.transformed_pcl_topic_);
        pnh_.getParam("pcl_topic", ret.pcl_topic_);
        pnh_.getParam("odom_topic", ret.odom_topic_);
        pnh_.getParam("map_frame", ret.map_frame_);
        pnh_.getParam("init_submap_frame", ret.init_submap_frame_);
        pnh_.getParam("traversability_topic", ret.travers_topic_);
        pnh_.getParam("frontiers_topic", ret.frontiers_topic_);
        pnh_.getParam("travers_expanded_topic", ret.travers_expanded_topic_);
        pnh_.getParam("travers_expanded_plan_topic", ret.travers_expanded_plan_topic_);
        pnh_.getParam("clearity_topic", ret.clearity_topic_);
        pnh_.getParam("path_topic", ret.path_topic_);

        std::cout << "origin1: " << ret.origin1_ << std::endl;
        std::cout << "origin2: " << ret.origin2_ << std::endl;
        std::cout << "size1: " << ret.size1_ << std::endl;
        std::cout << "size2: " << ret.size2_ << std::endl;
        std::cout << "resolution: " << ret.resolution_ << std::endl;
        std::cout << "vis_radius: " << ret.vis_radius_ << std::endl;
        std::cout << "init_submap: " << ret.init_submap_ << std::endl;
        std::cout << "init_submap_radius: " << ret.init_submap_radius_ << std::endl;
        std::cout << "init_submap_height_offset: " << ret.init_submap_height_offset_ << std::endl;
        std::cout << "Slope threshhold: " << ret.slope_th_ << std::endl;
        std::cout << "Robot size: " << ret.robot_size_ << std::endl;
        std::cout << "Max frontier lenth: " << ret.max_frontier_length_ << std::endl;
        std::cout << "Min frontier lenth: " << ret.min_frontier_length_ << std::endl;
        std::cout << "explored_point_radius " << ret.explored_point_radius_ << std::endl;
        std::cout << "Update frequency: " << ret.map_update_frequency_ << std::endl;

        // std::cout << "" << ret. << std::e
        std::cout << "out_pcl_topic: " << ret.out_pcl_topic_ << std::endl;
        std::cout << "transformed_pcl_topic: " << ret.transformed_pcl_topic_ << std::endl;
        std::cout << "pcl_topic: " << ret.pcl_topic_ << std::endl;
        std::cout << "odom_topic: " << ret.odom_topic_ << std::endl;
        std::cout << "map_frame: " << ret.map_frame_ << std::endl;
        std::cout << "init_submap_frame: " << ret.init_submap_frame_ << std::endl;
        std::cout << "travers_topic: " << ret.travers_topic_ << std::endl;
        std::cout << "frontiers_topic: " << ret.frontiers_topic_ << std::endl;
        std::cout << "travers_expanded_topic: " << ret.travers_expanded_topic_ << std::endl;
        std::cout << "clearity_topic: " << ret.clearity_topic_ << std::endl;
        std::cout << "path_topic: " << ret.path_topic_ << std::endl;

        ret.n_cells1_ = int(ret.size1_ / ret.resolution_);
        ret.n_cells2_ = int(ret.size2_ / ret.resolution_);
        ret.vis_radius_cells_ = int(ret.vis_radius_ / ret.resolution_);
        ret.robot_size_cells_ = int(ret.robot_size_ / ret.resolution_);
        ret.max_frontier_lenght_cells_ = int(ret.max_frontier_length_ / ret.resolution_);
        ret.min_frontier_size_ = int(ret.min_frontier_length_ / ret.resolution_);
        ret.explored_point_radius_cells_ = int(ret.explored_point_radius_ / ret.resolution_);

        std::cout << "n_cells1: " << ret.n_cells1_ << std::endl;
        std::cout << "n_cells2: " << ret.n_cells2_ << std::endl;
        std::cout << "vis_radius_cells: " << ret.vis_radius_cells_ << std::endl;
        std::cout << "robot_size_cells: " << ret.robot_size_cells_ << std::endl;
        std::cout << "max_frontier_lenght_cells: " << ret.max_frontier_lenght_cells_ << std::endl;
        std::cout << "min_frontier_size: " << ret.min_frontier_size_ << std::endl;
        std::cout << "explored_point_radius_cells: " << ret.explored_point_radius_cells_ << std::endl;

        // change degrees to rads
        // ret.ang_max = M_PI*ret.ang_max/180.0;
        // ret.ang_turnThreshold = M_PI*ret.ang_turnThreshold/180.0;
        return ret;
    };

}

#endif