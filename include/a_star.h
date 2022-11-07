/**
 * @file a_star.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this is a class only for a star algorithm
 * @version 0.1
 * @date 2022-01-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <boost/heap/binomial_heap.hpp>
#include "visualize.h"
#include "collisiondetection.h"
#include "utility.h"
namespace HybridAStar
{

    /*!
 * \brief A class that encompasses the functions central to the search.
 */
    class AStar
    {
    public:
        /// The default constructor
        AStar(){};
        // AStar(const std::shared_ptr<CollisionDetection> &configuration_space_ptr,
        //       const std::shared_ptr<Visualize> &visualization_ptr, const uint &possible_direction, const bool &visualization2D);
        AStar(const ParameterAStar &params,
              const std::shared_ptr<Visualize> &visualization_ptr);

        /**
         * @brief set map and calculate lookup table
         *
         * @param map
         */
        void Initialize(nav_msgs::OccupancyGrid::Ptr map);
        // void Initialize(const Node2D &start, const Node2D &goal);

        /**
         * @brief this is traditional A-star algorithm
         * 
         * @param nodes2D 
         * @return float 
         */
        float GetAStarCost(Node2D *nodes2D, const Node2D &start, const Node2D &goal, const bool &in_hybrid_a = false);

        Utility::Path3D GetPath(Node3D &start, Node3D &goal,
                                Node2D *nodes2D);

    private:
        /**
       * @brief Create possible successors of current node according its possible direction
       * 
       * @param pred 
       * @param possible_dir 
       * @return std::vector<std::shared_ptr<Node2D>> 
       */
        std::vector<std::shared_ptr<Node2D>> CreateSuccessor(const Node2D &pred, const uint &possible_dir);

        std::vector<std::shared_ptr<Node2D>> CreateSuccessor(const Node2D &pred, const uint &possible_dir, const uint &step_size);
        /**
         * @brief update heuristic for a star
         * 
         * @param goal 
         */
        void UpdateHeuristic(Node2D &current);
        /**
         * @brief update cost so far for a star
         * 
         * @param node 
         */
        void UpdateCostSoFar(Node2D &node);

        void TracePath(std::shared_ptr<Node2D> node2d_ptr);

        int FindStepSize(const Node2D &current_node);

    private:
        ParameterAStar params_;
        /// A flag for the visualization of 2D nodes (true = on; false = off)
        bool visualization2D_ = true;
        Utility::Path2D path_;
        Node2D start_;
        Node2D goal_;
        std::shared_ptr<CollisionDetection> configuration_space_ptr_;
        std::shared_ptr<Visualize> visualization_ptr_;
    };
}
