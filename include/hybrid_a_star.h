#pragma once
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <boost/heap/binomial_heap.hpp>
typedef ompl::base::SE2StateSpace::StateType State;

#include "visualize.h"
#include "collisiondetection.h"
#include "a_star.h"
#include "cubic_bezier.h"
#include "lookup_table.h"
#include "piecewise_cubic_bezier.h"
namespace HybridAStar
{
   //path from start to goal
   typedef std::vector<Node3D> Path3D;
   /*!
 * \brief A class that encompasses the functions central to the search.
 */
   class HybridAStar
   {
   public:
      /// The deault constructor
      HybridAStar(){};
      HybridAStar(const ParameterHybridAStar &params,
                  const std::shared_ptr<Visualize> &visualization_ptr);
      /**
       * @brief set map and calculate lookup table
       * 
       * @param map 
       */
      void Initialize(nav_msgs::OccupancyGrid::Ptr map);

      // HYBRID A* ALGORITHM
      /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
            \return the pointer to the node satisfying the goal condition
  */
      Path3D GetPath(Node3D &start, Node3D &goal,
                     Node3D *nodes3D, Node2D *nodes2D);

   private:
      /**
   * @brief select possible steering angle(relative angle) from available_angle_range_vec according vehicle setup
   * 
   * @param available_angle_range_vec 
   * @param pred 
   * @return std::vector<float> 
   */
      std::vector<float> SelectAvailableSteeringAngle(const Utility::AngleRangeVec &available_angle_range_vec, const Node3D &pred);
      std::vector<std::pair<float, float>> SelectAvailableSteeringAngle(const std::vector<std::pair<float, Utility::AngleRange>> &available_angle_range_vec, const Node3D &pred);
      /**
       * @brief update heuristic for hybrid a star
       * 
       * @param start 
       * @param goal 
       * @param nodes2D 
       */
      void UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D);
      /**
       * @brief analytical expansion in paper, here use dubins curve.
       * 
       * @param start 
       * @param goal 
       * @param configurationSpace 
       * @return Node3D* 
       */
      Path3D AnalyticExpansions(const Node3D &start, Node3D &goal);

      std::vector<std::shared_ptr<Node3D>> CreateSuccessor(const Node3D &pred);

      void UpdateCostSoFar(Node3D &node, const float &weight_turning, const float &weight_change_of_direction, const float &weight_reverse);

      void TracePath(std::shared_ptr<Node3D> node3d_ptr);

      void ConvertToPiecewiseCubicBezierPath();

   private:
      ParameterHybridAStar params_;
      Path3D piecewise_cubic_bezier_path_;
      Path3D path_;
      Node3D start_;
      Node3D goal_;
      std::shared_ptr<CollisionDetection> configuration_space_ptr_;
      std::shared_ptr<LookupTable> lookup_table_ptr_;
      std::shared_ptr<Visualize> visualization_ptr_;
      std::shared_ptr<AStar> a_star_ptr_;
      uint map_width_;
      uint map_height_;
      /**
       * @brief index of path_ which analytic expansion start.
       * 
       */
      uint analytical_expansion_index_;
   };
}
