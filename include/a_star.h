/**
 * @file a_star.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this is a class only for a star algorithm, Inheritance from expander
 * @version 0.2
 * @date 2022-12-13
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "expander.h"
#include "visualize.h"
#include "collisiondetection.h"
#include "utility.h"
namespace HybridAStar
{

    /*!
 * \brief A class that encompasses the functions central to the search.
 */
    class AStar : public Expander
    {
    public:
        AStar(std::string frame_id, costmap_2d::Costmap2D *_costmap, const ParameterAStar &params,
              const std::shared_ptr<Visualize> &visualization_ptr)
            : Expander(frame_id, _costmap)
        {
            params_ = params;
            configuration_space_ptr_.reset(new CollisionDetection(params_.collision_detection_params, _costmap));
            visualization_ptr_ = visualization_ptr;
            resolution_ = _costmap->getResolution();
            origin_y_ = _costmap->getOriginY();
            origin_x_ = _costmap->getOriginX();
            nav_msgs::OccupancyGrid::Ptr map;
            map.reset(new nav_msgs::OccupancyGrid());
            Utility::TypeConversion(_costmap, frame_id, map);
            configuration_space_ptr_->UpdateGrid(map);
        }
        AStar(const ParameterAStar &params,
              const std::shared_ptr<Visualize> &visualization_ptr) : Expander()
        {
            params_ = params;
            configuration_space_ptr_.reset(new CollisionDetection(params_.collision_detection_params));
            visualization_ptr_ = visualization_ptr;
        };

        ~AStar(){

        };
        /**
         * @brief Find the path between the start pose and goal pose
         * @param start the reference of start pose
         * @param goal the reference of goal pose
         * @param cells_x the number of the cells of the costmap in x axis
         * @param cells_y the number of the cells of the costmap in y axis
         * @param plan the refrence of plan;
         * @return true if a valid plan was found.
         */
        bool calculatePath(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                           int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped> &plan, ros::Publisher &pub, visualization_msgs::MarkerArray &pathNodes);
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

        Utility::Path3D GetPath(Node2D &start, Node2D &goal,
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

        std::vector<std::shared_ptr<Node2D>> CreateSuccessor(const Node2D &pred, const uint &possible_dir, const float &step_size);
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

        float FindStepSize(const Node2D &current_node);

    private:
        ParameterAStar params_;
        /// A flag for the visualization of 2D nodes (true = on; false = off)
        bool visualization2D_ = true;
        Utility::Path2D path_;
        Node2D start_;
        Node2D goal_;
        std::shared_ptr<CollisionDetection> configuration_space_ptr_;
        std::shared_ptr<Visualize> visualization_ptr_;

        float resolution_;
        float origin_y_;
        float origin_x_;
    };
}
