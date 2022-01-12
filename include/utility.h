/**
 * @file utility.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief include some type conversion function, some function can`t be in a class
 * @version 0.2
 * @date 2022-01-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef UTILITY
#define UTILITY
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include "node2d.h"
#include "node3d.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
namespace Utility
{
    //*******************type conversion*******************
    void ConvertRosPathToVectorVector3D(const nav_msgs::Path::ConstPtr &path, std::vector<Eigen::Vector3f> &vector_3d_vec);

    Eigen::Vector2f ConvertIndexToEigenVector2f(const int &index, const int &map_width);

    nav_msgs::Path ConvertVectorVector3DToRosPath(const std::vector<Eigen::Vector3f> &vector_3d_vec);

    Eigen::Vector2f ConvertVector3fToVector2f(const Eigen::Vector3f &vector_3d);

    Eigen::Vector3f ConvertVector2fToVector3f(const Eigen::Vector2f &vector_2d);

    Eigen::Vector3f ConvertNode3DToVector3f(const HybridAStar::Node3D &node3d);

    HybridAStar::Node3D ConvertVector3fToNode3D(const Eigen::Vector3f &vector3d);
    //**********************computational geometry****************

    /**
     * @brief check if p3 lines on p1-p2
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @return true 
     * @return false 
     */
    bool OnSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3);
    /**
     * @brief determine p3 is above or below segment p1-p2
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @return int 1 above and on segment, 0 below, 
     */
    int IsAboveSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3);
    /**
     * @brief determine if two segment(p1-p2, p3-p4) are intersected? 
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @param p4 
     * @return int 
     */
    int IsIntersect(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, const Eigen::Vector2f &p4);
    /**
     * @brief find intersection point between segment p1-p2 and segment p3-p4
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @param p4 
     * @return Eigen::Vector2f 
     */
    Eigen::Vector2f FindIntersectionPoint(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, const Eigen::Vector2f &p4);
    /**
     * @brief this is only work for vector2d, for 3d please cross in eigen
     * 
     * @param p1 
     * @param p2 
     * @return float 
     */
    float CrossProduct(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);
    /**
     * @brief determine if a point is inside a polygon
     * 
     * @param polygon p0->p1->p2->p3->p0
     * @param point 
     * @return int 1 inside, 0 outside
     */
    int IsInsidePolygon(const std::vector<Eigen::Vector2f> &polygon, const Eigen::Vector2f &point);
    int IsInsidePolygon(const std::vector<Eigen::Vector2f> &polygon, const Eigen::Vector3f &point);
    /**
     * @brief Create a Polygon object, current is only for rectangle
     * 
     * @param width 
     * @param height 
     * @return std::vector<Eigen::Vector2f> 
     */
    std::vector<Eigen::Vector2f> CreatePolygon(const float &width, const float &height);

    float GetDistanceFromVector2fToVector3f(const Eigen::Vector3f &vector_3d, const Eigen::Vector2f &vector_2d);

    //*************************other ***********************

    float Clamp(const float &number, const float &upper_bound, const float &lower_bound);

    /**
     * @brief convert angle in deg into [-PI,Pi)
     * 
     * @param deg 
     * @return float 
     */
    float DegNormalization(const float &deg);

    float RadNormalization(const float &rad);

    float DegToZeroTo2P(const float &deg);

    float RadToZeroTo2P(const float &rad);

    float ConvertDegToRad(const float &deg);

    float ConvertRadToDeg(const float &rad);

    bool IsCloseEnough(const HybridAStar::Node3D &start, const HybridAStar::Node3D &goal, const float &distance_range, const float &angle_range);

    float GetDistance(const HybridAStar::Node3D &start, const HybridAStar::Node3D &goal);
    float GetDistance(const HybridAStar::Node2D &start, const HybridAStar::Node2D &goal);
}

#endif // UTILITY
