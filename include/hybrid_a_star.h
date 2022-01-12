#ifndef ALGORITHM_H
#define ALGORITHM_H

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
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
         \return the pointer to the node satisfying the goal condition
  */
      Path3D GetPath(Node3D &start, Node3D &goal,
                     Node3D *nodes3D, Node2D *nodes2D, int width, int height);

   private:
      /**
       * @brief update heuristic for hybrid a star
       * 
       * @param start 
       * @param goal 
       * @param nodes2D 
       * @param width 
       * @param height 
       */
      void UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D, int width, int height);
      /**
       * @brief analytical expansion in paper, here use dubins curve.
       * 
       * @param start 
       * @param goal 
       * @param configurationSpace 
       * @return Node3D* 
       */
      Path3D AnalyticExpansions(const Node3D &start, Node3D &goal, std::shared_ptr<CollisionDetection> &configurationSpace);

      std::vector<std::shared_ptr<Node3D>> CreateSuccessor(const Node3D &pred, const int &possible_dir);

      void UpdateCostSoFar(Node3D &node, const float &weight_turning, const float &weight_change_of_direction, const float &weight_reverse);

      void TracePath(std::shared_ptr<Node3D> node3d_ptr);

   private:
      ParameterHybridAStar params_;
      Path3D path_;
      Node3D start_;
      Node3D goal_;
      std::shared_ptr<CollisionDetection> configuration_space_ptr_;
      std::shared_ptr<LookupTable> lookup_table_ptr_;
      std::shared_ptr<Visualize> visualization_ptr_;
      std::shared_ptr<AStar> a_star_ptr_;
   };
}
#endif // ALGORITHM_H
