/**
 * @file hybrid_a_star_planner.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this planner is the interface for hybrid a star planner in move it plugin
 * @version 0.1
 * @date 2022-04-13
 *
 * @copyright Copyright (c) 2022
 *
 */
/** include the libraries you need in your planner here */
/** for global path planner interface */
#pragma once
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include "planner.h"
using std::string;

namespace HybridAStar
{

    class HybridAStarPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        HybridAStarPlanner();
        HybridAStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

    private:
        // Planner hybrid_a_star_;
    };
};
