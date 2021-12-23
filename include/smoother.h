#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
#include "utility.h"
#include "parameter_manager.h"
namespace HybridAStar
{
   /*!
   \brief This class takes a path object and smooths the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
   class Smoother
   {
   public:
      Smoother(const ParameterSmoother &smoother_params)
      {
         params_ = smoother_params;
      }

      /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.

     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */
      void SmoothPath(DynamicVoronoi &voronoi);

      /*!
     \brief Given a node pointer the path to the root node will be traced recursively, output path is from goal to start.
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */
      void TracePath(const Node3D *node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());

      /// returns the path of the smoother object
      const std::vector<Node3D> &GetPath() { return path_; }

      /// obstacleCost - pushes the path away from obstacles
      Vector2D ObstacleTerm(Vector2D xi);

      /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
      Vector2D CurvatureTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

      /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
      Vector2D SmoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

      /// voronoiCost - trade off between path length and closeness to obstacles
      Vector2D VoronoiTerm(Vector2D xi);

      // cost for path length, in order to minimize path length
      Vector2D PathLengthTerm(Vector2D xim1, Vector2D xi, Vector2D xip1);

      /// a boolean test, whether vector is on the grid or not
      bool isOnGrid(Vector2D vec)
      {
         if (vec.getX() >= 0 && vec.getX() < map_width_ &&
             vec.getY() >= 0 && vec.getY() < map_height_)
         {
            return true;
         }
         return false;
      };
      /**
       * @brief main purpose is to remove some duplicates
       * 
       * @return int 
       */
      int PreprocessPath();
      /**
       * @brief Get the Path Difference between two path, these two path must have the same size
       * 
       * @param path_before_smooth 
       * @param path_after_smooth 
       * @return float 
       */
      float GetPathDiff(const std::vector<Node3D> &path_before_smooth, const std::vector<Node3D> &path_after_smooth);

   private:
      void SetSmootherParams(const ParameterSmoother &smoother_params);
      ParameterSmoother params_;

      /// voronoi diagram describing the topology of the map
      DynamicVoronoi voronoi_;
      /// width of the map
      int map_width_;
      /// map_height_ of the map
      int map_height_;
      /// path to be smoothed
      std::vector<Node3D> path_;
      ///path after preprocessed.
      std::vector<Node3D> preprocessed_path_;

      std::shared_ptr<ParameterManager> param_manager_;
   };
}
#endif // SMOOTHER_H
