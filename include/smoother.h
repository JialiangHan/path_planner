#pragma once

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

     During the different iterations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */
      void SmoothPath(const DynamicVoronoi &voronoi);

      //       /*!
      //      \brief Given a node pointer the path to the root node will be traced recursively, output path is from goal to start.
      //      \param node a 3D node, usually the goal node
      //   */
      //       void TracePath(const Node3D *node);
      void SetPath(const std::vector<Node3D> &path) { path_ = path; };
      /// returns the path of the smoother object
      const std::vector<Node3D> &GetPath() { return path_; }

      /// obstacleCost - pushes the path away from obstacles
      Eigen::Vector2f ObstacleTerm(const Eigen::Vector2f &xi);

      /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
      Eigen::Vector2f CurvatureTerm(const Eigen::Vector2f &xim2, const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1, const Eigen::Vector2f &xip2);

      Eigen::Vector2f CurvatureTerm(const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1);

      /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
      Eigen::Vector2f SmoothnessTerm(const Eigen::Vector2f &xim2, const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1, const Eigen::Vector2f &xip2);

      /// voronoiCost - trade off between path length and closeness to obstacles
      Eigen::Vector2f VoronoiTerm(const Eigen::Vector2f &xi);

      // cost for path length, in order to minimize path length
      Eigen::Vector2f PathLengthTerm(const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1);

      /// a boolean test, whether vector is on the grid or not
      bool isOnGrid(const Eigen::Vector2f &vec)
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

      Eigen::Vector2f OrthogonalComplements(const Eigen::Vector2f &a, const Eigen::Vector2f &b);

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
