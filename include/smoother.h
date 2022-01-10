#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <vector>
#include "dynamicvoronoi.h"
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

      void Clear();

      /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.

     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */
      void SmoothPath(DynamicVoronoi &voronoi);

      //       /*!
      //      \brief Given a node pointer the path to the root node will be traced recursively, output path is from goal to start.
      //      \param node a 3D node, usually the goal node
      //   */
      //       void TracePath(const Node3D *node);
      void SetPath(const std::vector<Node3D> &path) { path_ = path; };
      /// returns the path of the smoother object
      const std::vector<Node3D> &GetPath() { return path_; }

      /// obstacleCost - pushes the path away from obstacles
      Eigen::Vector2d ObstacleTerm(Eigen::Vector2d xi);

      /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
      Eigen::Vector2d CurvatureTerm(Eigen::Vector2d xim2, Eigen::Vector2d xim1, Eigen::Vector2d xi, Eigen::Vector2d xip1, Eigen::Vector2d xip2);

      /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
      Eigen::Vector2d SmoothnessTerm(Eigen::Vector2d xim2, Eigen::Vector2d xim1, Eigen::Vector2d xi, Eigen::Vector2d xip1, Eigen::Vector2d xip2);

      /// voronoiCost - trade off between path length and closeness to obstacles
      Eigen::Vector2d VoronoiTerm(Eigen::Vector2d xi);

      // cost for path length, in order to minimize path length
      Eigen::Vector2d PathLengthTerm(Eigen::Vector2d xim1, Eigen::Vector2d xi, Eigen::Vector2d xip1);

      /// a boolean test, whether vector is on the grid or not
      bool isOnGrid(Eigen::Vector2d vec)
      {
         if (vec(0, 0) >= 0 && vec(0, 0) < map_width_ &&
             vec(1, 0) >= 0 && vec(1, 0) < map_height_)
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

      Eigen::Vector2d OrthogonalComplements(const Eigen::Vector2d &a, const Eigen::Vector2d &b);

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
