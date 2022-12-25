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
#include "cubic_bezier.h"
#include <cmath>

using namespace HybridAStar;

namespace RRTPlanner
{

    // this is a tree structure
    typedef std::vector<Node3D> RRT;
    // status for extend,connect
    enum Status : int
    {
        // collision
        Trapped = 0,
        // reach the target
        Reached = 1,
        // this status is not collision and not reached,
        Advanced = 2
    };
    enum RRTFlag : int
    {
        // rrt root is start
        Start = 0,
        // rrt root is goal
        Goal = 1,
    };
    class RRTPlanner
    {
    private:
        ParameterRRTPlanner params_;
        Utility::Path3D path_;
        Node3D start_;
        Node3D goal_;
        std::shared_ptr<CollisionDetection> configuration_space_ptr_;

        std::shared_ptr<Visualize> visualization_ptr_;
        // std::shared_ptr<AStar> a_star_ptr_;
        uint map_width_;
        uint map_height_;
        float resolution_;

        /**
         * @brief select a collision-free node in current map
         *
         * @return Node3D
         */
        Node3D
        SelectRandomNode();
        /**
         * @brief
         *
         * @param max_steering_angle in rad
         * @param current
         * @return float
         */
        float SelectRandomSteeringAngle(const float &max_steering_angle, const Node3D &current);
        /**
         * @brief find step size and steering angle for current node
         *
         * @param current
         * @return std::pair<float, float> first is step size, second is steering angle
         */
        std::pair<float, float> FindStepSizeAndSteeringAngle(const Node3D &closest_node, const Node3D &direction_node);

        float FindSteeringAngle(const Node3D &closest_node, const Node3D &direction_node);

        float FindStepSize(const Node3D &closest_node, const float &steering_angle, const Node3D &target);
        /**
         * @brief generate a collision-free successor
         *
         * @return Node3D
         */
        Node3D GenerateSuccessor(RRT &rrt_start);
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
        Utility::Path3D TracePath(const RRT &rrt, const int &goal_index);

        Utility::Path3D TracePath(const RRT &rrt, const Node3D &current);
        /**
         * @brief check if current rrt reach goal?
         *
         * @return int: index of goal node
         */
        int GoalCheck(const RRT &rrt, bool consider_orientation);
        /**
         * @brief add node3d to a rrt
         *
         * @param current
         */
        void AddNodeToRRT(RRT &rrt, Node3D &current);

        void AddNodeToRRT(RRT &rrt, Utility::Path3D &current);
        /**
         * @brief find closet node to this random node on rrt tree
         *
         * @param random_node
         * @return Node3D
         */
        Node3D FindClosestNode(const RRT &rrt, const Node3D &random_node, const bool &flag = false);

        Node3D FindDirectionNode(const int &failure_counts, const RRT &rrt, const RRTFlag &flag = RRTFlag::Start);

        void Planning();
        /**
         * @brief this function is similar to the one in hybrid a star. 1. find node on rrt closest to goal. 2. if there is a no-collision curve from the node in first step to goal, then put this curve into rrt and return true
         *
         * @return true
         * @return false
         */
        bool AnalyticExpansion(RRT &rrt, const Node3D &start, Node3D &goal);

        float GetPossibilityToGoal(const int &failure_counts);
        /**
         * @brief find step size by obstacle density
         *
         * @param closest_node
         * @return float
         */
        float FindOriginalStepSize(const Node3D &closest_node, const float &distance_to_goal, const float &available_step_size_obstacle);
        /**
         * @brief rewiring process in RRT*, current node is not on rrt
         *
         * @param current
         */
        void Rewire(RRT &rrt, Node3D &current, const float &radius);
        /**
         * @brief check if current node is on rrt
         *
         * @param current
         * @return true
         * @return false
         */
        bool CheckNodeOnRRT(const RRT &rrt, const Node3D &current);

        float FindNodeCostSoFar(const Node3D &current);

        std::vector<Node3D> FindNeighbors(const RRT &rrt, const Node3D &current, const float &radius);

        std::vector<std::pair<Node3D, float>> FindNeighborsAndCostSoFar(const RRT &rrt, const Node3D &current, const float &radius);

        /**
         * @brief extend a rrt towards target only one step
         *
         * @param rrt
         * @param target
         * @return Status
         *       // collision
            Trapped = 0,
            // reach the target
            Reached = 1,
            // this status is not collision and not reached,
            Advanced = 2
         */
        std::pair<Status, Node3D> Extend(RRT &rrt, const Node3D &target, const bool &flag = false);
        /**
         * @brief connect rrt to target node in multiple step
         *
         * @param rrt
         * @param target
         * @return Status
         */
        std::pair<Status, Node3D> Connect(RRT &rrt, const Node3D &target);
        /**
         * @brief swap twp rrts
         *
         * @param rrt_1
         * @param rrt_2
         */
        void Swap(std::pair<RRT, RRTFlag> &rrt_1, std::pair<RRT, RRTFlag> &rrt_2);

        void RRTConnectPlanner();

        RRT InitializeRRT(const Node3D &root);
        /**
         * @brief Set the Path using two connected rrt
         *
         * @param rrt_1
         * @param rrt_2
         * @param connect_point
         */
        void SetPath(const std::pair<RRT, RRTFlag> &rrt_1, const std::pair<RRT, RRTFlag> &rrt_2, const Node3D &connect_point);
        /**
         * @brief this treatment is for rrt based on goal, which is just to plus 180deg to node orientation
         *
         */
        Node3D TreatNode(const Node3D &goal);

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

        Utility::Path3D GetPath(const Node3D &start, const Node3D &goal);
        /**
         * @brief short cut the path since rrt path will have some unnecessary path point
         *
         * @param consider_steering_angle_limit, if true, consider vehicle steering angle limit.
         * @return Utility::Path3D
         */
        Utility::Path3D ShortCut(const Utility::Path3D &path, bool consider_steering_angle_limit);

        Utility::Path3D PiecewiseCubicBezier(const Utility::Path3D &path);
    };

}