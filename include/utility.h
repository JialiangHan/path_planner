/*!
   \file
   \brief This is a collection of utility functions that are used throughout the project.
    include some type conversion function
*/
#ifndef UTILITY
#define UTILITY

#include <cmath>
#include <nav_msgs/Path.h>
#include "node2d.h"
#include "node3d.h"
namespace HybridAStar
{
    /*!
    \brief The namespace that wraps helper.h
    \namespace Helper
*/
    namespace Utility
    {
        /**
         * @brief convert a ros path msg to vector of node_3d
         * 
         * @param path a ros path
         * @param node_3d_vec 
         */
        void ConvertRosPathToVectorNode3D(const nav_msgs::Path::ConstPtr &path, std::vector<Node3D> &node_3d_vec);

        Node2D ConvertNode3DToNode2D(const Node3D &node_3d);

        Node2D ConvertIndexToNode2D(const int &index, const int &map_width);

        float GetDistanceFromNode2DToNode3D(const Node2D &obstacle_2d, const Node3D &node_3d);
    }
}

#endif // UTILITY
