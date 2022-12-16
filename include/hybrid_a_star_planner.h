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
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include "node2d.h"
#include "node3d.h"
using std::string;

namespace HybridAStar
{

    class HybridAStarPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        HybridAStarPlanner();
        HybridAStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        ~HybridAStarPlanner();

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        /**
         * @brief Publish the plan to RVIZ
         * @param path the vector contain the path
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
        /**
         * @brief Publish the path node to RVIZ
         * @param path the vector contain the path
         */
        void publishPathNodes(const std::vector<geometry_msgs::PoseStamped> &path);
        /**
         * @brief The call back function of makeplan service
         * @param req
         * @param resp the plan the planner made
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

    protected:
        bool initialized_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        ros::Publisher path_vehicles_pub_; // 用于在路径上发布车子位置
        costmap_2d::Costmap2D *costmap;

    private:
        /**
         * @brief Clarn the visualization_msgs::Marker 清理可视化信息的标记点
         */
        void clearPathNodes(void);

        /**
         * @brief Check whethe the start pose is available
         * @param start A reference to start pose
         * @return True if the start pose is available
         */
        bool checkStartPose(const geometry_msgs::PoseStamped &start);

        /**
         * @brief Check whethe the goal pose is available
         * @param goal A reference to goal pose
         * @return True if the goal pose is available
         */
        bool checkgoalPose(const geometry_msgs::PoseStamped &goal);
        visualization_msgs::MarkerArray pathNodes; // 节点数据结构，用于可视化
        double resolution;
        ros::ServiceServer make_plan_srv_;
        bool use_hybrid_astar;
        std::shared_ptr<Planner> hybrid_a_star_ptr_;
        ParameterManager params_;
    };
};
