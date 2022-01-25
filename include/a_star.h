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
    //path from start to goal
    typedef std::vector<Node2D> Path2D;
    /*!
 * \brief A class that encompasses the functions central to the search.
 */
    class AStar
    {
    public:
        /// The deault constructor
        AStar(){};
        AStar(const std::shared_ptr<CollisionDetection> &configuration_space_ptr,
              const std::shared_ptr<Visualize> &visualization_ptr, const uint &possible_direction, const bool &visualization2D);

        void Initialize(const Node2D &start, const Node2D &goal);

        /**
         * @brief this is traditional A-star algorithm
         * 
         * @param nodes2D 
         * @return float 
         */
        float GetAStarCost(Node2D *nodes2D);

    private:
        /**
       * @brief Create possible successors of current node according its possible direction
       * 
       * @param pred 
       * @param possible_dir 
       * @return std::vector<std::shared_ptr<Node2D>> 
       */
        std::vector<std::shared_ptr<Node2D>> CreateSuccessor(const Node2D &pred, const uint &possible_dir);
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

        // void TracePath(std::shared_ptr<Node2D> node2d_ptr);

    private:
        // nubmer of direction to create successor for A-star algorithm
        uint possible_direction_ = 8;
        /// A flag for the visualization of 2D nodes (true = on; false = off)
        bool visualization2D_ = true;
        Path2D path_;
        Node2D start_;
        Node2D goal_;
        std::shared_ptr<CollisionDetection> configuration_space_ptr_;
        std::shared_ptr<Visualize> visualization_ptr_;
    };
}
