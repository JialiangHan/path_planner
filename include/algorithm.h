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

#include "cubic_bezier.h"
#include "lookup_table.h"
namespace HybridAStar
{
   //path from start to goal
   typedef std::vector<Node3D> Path;
   /*!
 * \brief A class that encompasses the functions central to the search.
 */
   class Algorithm
   {
   public:
      /// The deault constructor
      Algorithm(){};
      Algorithm(const ParameterAlgorithm &params)
      {
         params_ = params;
      };

      // HYBRID A* ALGORITHM
      /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration
     \param lookup_table_ptr the lookup of analytical solutions (Dubin's paths)
     \param visualization the visualization object publishing the search to RViz
     \return the pointer to the node satisfying the goal condition
  */
      Path HybridAStar(Node3D &start, Node3D &goal,
                       Node3D *nodes3D, Node2D *nodes2D, int width, int height, std::shared_ptr<CollisionDetection> &configurationSpace, const std::shared_ptr<LookupTable> &lookup_table_ptr, std::shared_ptr<Visualize> &visualization);

      Path GetPath() const { return path_; };

   private:
      /**
    * 
    * @brief this is traditional A-star algorithm
    * 
    * @param start 
    * @param goal 
    * @param nodes2D 
    * @param width 
    * @param height 
    * @param configurationSpace 
    * @param visualization 
    * @return float 
    */
      float AStar(Node2D &start, Node2D &goal, Node2D *nodes2D, int width, int height, std::shared_ptr<CollisionDetection> &configurationSpace, std::shared_ptr<Visualize> &visualization);
      /**
       * @brief update heuristic for hybrid a star
       * 
       * @param start 
       * @param goal 
       * @param nodes2D 
       * @param lookup_table_ptr 
       * @param width 
       * @param height 
       * @param configurationSpace 
       * @param visualization 
       */
      void UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D, const std::shared_ptr<LookupTable> &lookup_table_ptr, int width, int height, std::shared_ptr<CollisionDetection> &configurationSpace, std::shared_ptr<Visualize> &visualization);
      /**
       * @brief analytical expansion in paper, here use dubins curve.
       * 
       * @param start 
       * @param goal 
       * @param configurationSpace 
       * @return Node3D* 
       */
      Path AnalyticExpansions(const Node3D &start, Node3D &goal, std::shared_ptr<CollisionDetection> &configurationSpace);

      std::vector<std::shared_ptr<Node3D>> CreateSuccessor(const Node3D &pred, const int &possible_dir);

      /**
       * @brief Create possible successors of current node according its possible direction
       * 
       * @param pred 
       * @param possible_dir 
       * @return std::vector<std::shared_ptr<Node2D>> 
       */
      std::vector<std::shared_ptr<Node2D>> CreateSuccessor(const Node2D &pred, const int &possible_dir);

      // void TracePath(const Node3D *node);

      void TracePath(std::shared_ptr<Node3D> node3d_ptr);

   private:
      ParameterAlgorithm params_;
      Path path_;
      Node3D start_;
      Node3D goal_;
   };
}
#endif // ALGORITHM_H
