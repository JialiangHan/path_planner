/**
 * @file rrt.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this file implement a rrt class to conduct a path planner using rrt algorithm
 * @version 0.1
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <vector>
#include "node3d.h"
#include "collisiondetection.h"
#include "utility.h"
#include "visualize.h"
#include "piecewise_cubic_bezier.h"

using namespace HybridAStar;

namespace RRTPlanner
{
    typedef std::vector<Node3D> Path3D;
    // this is a tree structure
    typedef std::vector<Node3D> RRT;
    class RRTPlanner
    {
    private:
        ParameterRRTPlanner params_;
        Path3D path_;
        Node3D start_;
        Node3D goal_;
        RRT rrt_;
        std::shared_ptr<CollisionDetection> configuration_space_ptr_;

        std::shared_ptr<Visualize> visualization_ptr_;
        // std::shared_ptr<AStar> a_star_ptr_;
        uint map_width_;
        uint map_height_;
        /**
         * @brief select a collision-free node in current map
         *
         * @return Node3D
         */
        Node3D SelectRandomNode();
        /**
         * @brief find step size and steering angle for current node
         *
         * @param current
         * @return std::pair<float, float> first is step size, second is steering angle
         */
        std::pair<float, float> FindStepSizeAndSteeringAngle(const Node3D &closest_node, const Node3D &random_node);
        /**
         * @brief generate a collision-free successor
         *
         * @return Node3D
         */
        Node3D GenerateSuccessor();
        /**
         * @brief generate a collision-free successor for closest node using step size and steering angle
         *
         * @param closest_node
         * @param stepsize_steering_angle
         * @return Node3D
         */
        Node3D GenerateSuccessor(const Node3D &closest_node, const std::pair<float, float> stepsize_steering_angle);
        /**
         * @brief trace path using a rrt
         *
         */
        void TracePath();
        /**
         * @brief check if current rrt reach goal?
         *
         * @return int: index of goal node
         */
        int GoalCheck(bool consider_orientation);
        /**
         * @brief add node3d to a rrt
         *
         * @param current
         */
        void AddNodeToRRT(const Node3D &current);
        /**
         * @brief find closet node to this random node on rrt tree
         *
         * @param random_node
         * @return Node3D
         */
        Node3D FindClosestNode(const Node3D &random_node);

        void Planning();

    public:
        RRTPlanner();
        RRTPlanner(const ParameterRRTPlanner &params,
                   const std::shared_ptr<Visualize> &visualization_ptr);

        /**
         * @brief set map and calculate lookup table
         *
         * @param map
         */
        void Initialize(nav_msgs::OccupancyGrid::Ptr map);

        Path3D GetPath(const Node3D &start, const Node3D &goal);
        /**
         * @brief short cut the path since rrt path will have some unnecessary path point
         *
         * @param consider_steering_angle_limit, if true, consider vehicle steering angle limit.
         * @return Path3D
         */
        Path3D ShortCut(bool consider_steering_angle_limit);

        Path3D PiecewiseCubicBezier(const Path3D &path);
    };

}