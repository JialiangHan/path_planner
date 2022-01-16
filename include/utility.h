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
    //******************typedef****************
    // first is start angle, second is range length, start angle in [0,2Pi]
    typedef std::pair<float, float> AngleRange;
    typedef std::vector<AngleRange> AngleRangeVec;
    typedef std::vector<Eigen::Vector2f> Polygon;
    //*******************type conversion*******************
    void ConvertRosPathToVectorVector3D(
        const nav_msgs::Path::ConstPtr &path,
        std::vector<Eigen::Vector3f> &vector_3d_vec);

    Eigen::Vector2f ConvertIndexToEigenVector2f(const int &index,
                                                const int &map_width);

    nav_msgs::Path ConvertVectorVector3DToRosPath(
        const std::vector<Eigen::Vector3f> &vector_3d_vec);

    Eigen::Vector2f ConvertVector3fToVector2f(const Eigen::Vector3f &vector_3d);

    Eigen::Vector3f ConvertVector2fToVector3f(const Eigen::Vector2f &vector_2d);

    Eigen::Vector3f ConvertNode3DToVector3f(const HybridAStar::Node3D &node3d);
    Eigen::Vector2f ConvertNod2DToVector2f(const HybridAStar::Node2D &node2d);
    Eigen::Vector2f ConvertNod3DToVector2f(const HybridAStar::Node3D &node3d);

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
    bool OnSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                   const Eigen::Vector2f &p3);
    /**
 * @brief determine p3 is above or below segment p1-p2
 *
 * @param p1
 * @param p2
 * @param p3
 * @return int 1 above and on segment, 0 below,
 */
    int IsAboveSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                       const Eigen::Vector2f &p3);
    /**
 * @brief determine if two segment(p1-p2, p3-p4) are intersected?
 *
 * @param p1
 * @param p2
 * @param p3
 * @param p4
 * @return int 1:intersect, 0: not intersect
 */
    int IsIntersect(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                    const Eigen::Vector2f &p3, const Eigen::Vector2f &p4);
    /**
 * @brief find intersection point between segment p1-p2 and segment p3-p4
 *
 * @param p1
 * @param p2
 * @param p3
 * @param p4
 * @return Eigen::Vector2f
 */
    Eigen::Vector2f FindIntersectionPoint(const Eigen::Vector2f &p1,
                                          const Eigen::Vector2f &p2,
                                          const Eigen::Vector2f &p3,
                                          const Eigen::Vector2f &p4);
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
    int IsInsidePolygon(const Polygon &polygon, const Eigen::Vector2f &point);
    int IsInsidePolygon(const Polygon &polygon, const Eigen::Vector3f &point);
    /**
 * @brief Create a Polygon object, current is only for rectangle
 *
 * @param width
 * @param height
 * @return Polygon
 */
    Polygon CreatePolygon(const float &width, const float &height);
    /**
 * @brief Create a Polygon object, current is only for rectangle
 *
 * @param origin left bottom point of rectangle
 * @param width
 * @param height
 * @return Polygon
 */
    Polygon CreatePolygon(const Eigen::Vector2f &origin, const float &width = 1,
                          const float &height = 1);

    float GetDistanceFromPointToPoint(const Eigen::Vector3f &vector_3d,
                                      const Eigen::Vector2f &vector_2d);
    /**
 * @brief determine if a segment starting from start to end is intersect with a
 * polygon,
 *
 * @param polygon
 * @param start
 * @param end
 * @return int 1: intersect, 0: not intersect
 */
    int IsSegmentIntersectWithPolygon(const Polygon &polygon,
                                      const Eigen::Vector2f &start,
                                      const Eigen::Vector2f &end);
    /**
 * @brief Get the Distance From Segment To a point
 *
 * @param start
 * @param end
 * @param point
 * @return float
 */
    float GetDistanceFromSegmentToPoint(const Eigen::Vector2f &start,
                                        const Eigen::Vector2f &end,
                                        const Eigen::Vector2f &point);
    /**
 * @brief Get the min Distance From Polygon To Point
 *
 * @param polygon
 * @param point
 * @return float
 */
    float GetDistanceFromPolygonToPoint(const Polygon &polygon,
                                        const Eigen::Vector2f &point);
    float GetDistanceFromPointToPoint(const Eigen::Vector2f &p1,
                                      const Eigen::Vector2f &p2);
    /**
 * @brief Get the Angle Between Two Vector , sign is determined by crossproduct,
 * CCW is positive
 *
 * @param p1_start
 * @param p1_end
 * @param p2_start
 * @param p2_end
 * @return float angle  all the angle are in range[0,Pi]
 */
    float GetAngleBetweenTwoVector(const Eigen::Vector2f &p1_start,
                                   const Eigen::Vector2f &p1_end,
                                   const Eigen::Vector2f &p2_start,
                                   const Eigen::Vector2f &p2_end);
    /**
 * @brief Get the Angle Range From Point To Segment object, start at point to
 * start vector.
 *
 * @param start
 * @param end
 * @param point
 * @return ::AngleRange all the angle are in range[0,2Pi]
 */
    AngleRange GetAngleRangeFromPointToSegment(const Eigen::Vector2f &start,
                                               const Eigen::Vector2f &end,
                                               const Eigen::Vector2f &point);
    /**
 * @brief Get the Angle Range From Point To Polygon object
 *
 * @param polygon
 * @param point
 * @return AngleRange all the angle are in range[0,2Pi]
 */
    AngleRange GetAngleRangeFromPointToPolygon(const Polygon &polygon,
                                               const Eigen::Vector2f &point);
    /**
 * @brief find visible vertex of a polygon from a point view, current this
 * function only work for rectangle
 *
 * @param polygon
 * @param point
 * @return std::vector<Eigen::Vector2f>
 */
    std::vector<Eigen::Vector2f>
    FindVisibleVertexFromNode(const Polygon &polygon, const Eigen::Vector2f &point);
    /**
 * @brief find if these two angle range are overlapped, fully included is not considered in this function
 * 
 * @param angle_range_1 
 * @param angle_range_2 
 * @return true 
 * @return false 
 */
    bool IsOverlap(const AngleRange &angle_range_1,
                   const AngleRange &angle_range_2);
    /**
     * @brief determine is angle range 1 is fully inside angle range 2
     * 
     * @param angle_range_1 
     * @param angle_range_2 
     * @return true 
     * @return false 
     */
    bool IsAngleRangeInclude(const AngleRange &angle_range_1,
                             const AngleRange &angle_range_2);

    // /**
    // * @brief if these two angle range are overlapped, combine them into one angle range
    // *
    // * @param angle_range_1
    // * @param angle_range_2
    // * @return AngleRange
    // */
    // AngleRange CombineAngleRange(const AngleRange &angle_range_1,
    //                              const AngleRange &angle_range_2);
    /**
 * @brief angle range 1 minus angle range 2,only for overlap condition, fully included is not considered
 * 
 * @param angle_range_1 
 * @param angle_range_2 
 * @return AngleRange 
 */
    AngleRange MinusAngleRange(const AngleRange &angle_range_1,
                               const AngleRange &angle_range_2);
    //*************************other ***********************

    float Clamp(const float &number, const float &upper_bound,
                const float &lower_bound);

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

    bool IsCloseEnough(const HybridAStar::Node3D &start,
                       const HybridAStar::Node3D &goal, const float &distance_range,
                       const float &angle_range);

    float GetDistance(const HybridAStar::Node3D &start,
                      const HybridAStar::Node3D &goal);
    float GetDistance(const HybridAStar::Node2D &start,
                      const HybridAStar::Node2D &goal);
}

#endif // UTILITY
