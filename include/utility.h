/**
 * @file utility.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief include some type conversion function
 * @version 0.1
 * @date 2021-12-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef UTILITY
#define UTILITY
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <nav_msgs/Path.h>
#include "node2d.h"
#include "node3d.h"
#include "point.h"
#include <Eigen/Dense>
// #include "vector2d.h"

namespace Utility
{

    void ConvertRosPathToVectorVector3D(const nav_msgs::Path::ConstPtr &path, std::vector<Eigen::Vector3d> &vector_3d_vec);

    Eigen::Vector2d ConvertIndexToEigenVector2d(const int &index, const int &map_width);

    float GetDistanceFromVector2dToVector3d(const Eigen::Vector3d &vector_3d, const Eigen::Vector2d &vector_2d);

}

#endif // UTILITY
