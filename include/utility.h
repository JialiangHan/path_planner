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
#pragma once
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <set>
#include <algorithm>
#include "node2d.h"
#include "node3d.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "computational_geometry.h"
#include <bits/stdc++.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
namespace Utility
{
    //******************typedef****************
    // first is start angle, second is range length, start angle in [0,2Pi]
    typedef std::pair<float, float> AngleRange;
    typedef std::vector<AngleRange> AngleRangeVec;
    typedef std::vector<Eigen::Vector2f> Polygon;
    typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> Edge;
    // path from start to goal
    typedef std::vector<HybridAStar::Node3D> Path3D;
    // path from start to goal
    typedef std::vector<HybridAStar::Node2D> Path2D;
    //*******************type conversion*******************
    /**
     * @brief this function convert type a to type b
     *
     */
    void TypeConversion(const geometry_msgs::PoseStamped &start, HybridAStar::Node3D &node3d);

    void TypeConversion(const geometry_msgs::PoseStamped &start, HybridAStar::Node2D &node2d);

    void TypeConversion(const HybridAStar::Node3D &node3d, geometry_msgs::PoseStamped &pose);

    void TypeConversion(const Path3D &path3d, std::vector<geometry_msgs::PoseStamped> &plan);

    void TypeConversion(
        const nav_msgs::Path::ConstPtr &path,
        std::vector<Eigen::Vector3f> &vector_3d_vec);

    Eigen::Vector2f ConvertIndexToEigenVector2f(const int &index,
                                                const int &map_width);

    void TypeConversion(
        const std::vector<Eigen::Vector3f> &vector_3d_vec, nav_msgs::Path &path);

    void TypeConversion(const Eigen::Vector3f &vector_3d, Eigen::Vector2f &vector_2d);

    void TypeConversion(const HybridAStar::Node3D &node3d, Eigen::Vector3f &vector);

    void TypeConversion(const HybridAStar::Node2D &node2d, Eigen::Vector2f &vector2f);

    void TypeConversion(const HybridAStar::Node3D &node3d, Eigen::Vector2f &vector2f);

    void TypeConversion(const Eigen::Vector3f &vector3d, HybridAStar::Node3D &node_3d);

    void TypeConversion(const Eigen::Vector2f &vector2d, HybridAStar::Node3D &node3d);

    void TypeConversion(const HybridAStar::Node3D &node3d, HybridAStar::Node2D &node_2d);

    void TypeConversion(const HybridAStar::Node2D &node2d, HybridAStar::Node3D &node_3d);

    void TypeConversion(const Path2D &path_2d, Path3D &path_3d);

    void TypeConversion(const std::vector<std::shared_ptr<HybridAStar::Node3D>> &smart_ptr_vec, std::vector<HybridAStar::Node3D *> &ptr_vec);

    void TypeConversion(costmap_2d::Costmap2D *_costmap, std::string frame_id, nav_msgs::OccupancyGrid::Ptr &map);
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
     * @brief determine if a point is on a line
     *
     * @param point
     * @param start
     * @param unit_vector
     * @return true
     * @return false
     */
    bool IsPointOnLine(const Eigen::Vector2f &point, const Eigen::Vector2f &start,
                       const Eigen::Vector2f &unit_vector);
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
     * @brief this is only work for vector2d, for 3d please use cross in eigen
     *
     * @param p1
     * @param p2
     * @return float
     */
    float CrossProduct(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);
    /**
     * @brief determine if a point is inside a polygon, on edge is not considered
     *
     * @param polygon p0->p1->p2->p3->p0
     * @param point
     * @return int 1 inside, 0 outside
     */
    int IsInsidePolygon(const Polygon &polygon, const Eigen::Vector2f &point);
    int IsInsidePolygon(const Polygon &polygon, const Eigen::Vector3f &point);
    /**
     * @brief determine if a point is on polygon edge
     *
     * @param polygon
     * @param point
     * @return true
     * @return false
     */
    bool IsOnPolygon(const Polygon &polygon, const Eigen::Vector2f &point);
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
    /**
 * @brief Create a Polygon object according to its center cooridnate x,y, and rotate it according to its headings
 * 
 * @param center : polygon centter
 * @param width : polygon width in x direction
 * @param height :polygon height in y direction
 * @param heading :vehicle heading
 * @return Polygon 
 */
    Polygon CreatePolygon(const Eigen::Vector2f &center, const float &width,
                          const float &height, const float &heading);

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
     * @brief Get the Distance From Point To Line, line is represented by a starting point and unit vector
     *
     * @param start
     * @param unit_vector
     * @param point
     * @return float
     */
    float GetDistanceFromPointToLine(const Eigen::Vector2f &point,
                                     const Eigen::Vector2f &start,
                                     const Eigen::Vector2f &unit_vector);

    /**
 * @brief Get the min Distance From Polygon To Point
 *
 * @param polygon
 * @param point
 * @return float
 */
    float GetDistanceFromPolygonToPoint(const Polygon &polygon,
                                        const Eigen::Vector2f &point);
    /**
    * @brief Get the Distance From Polygon To Segment 
    * 
    * @param polygon 
    * @param start segment start
    * @param end segment end
    * @return float distance
    */
    float GetDistanceFromPolygonToSegment(const Polygon &polygon,
                                          const Eigen::Vector2f &start,
                                          const Eigen::Vector2f &end);
    float GetDistanceFromPointToPoint(const Eigen::Vector2f &p1,
                                      const Eigen::Vector2f &p2);

    float GetDistanceFromPolygonToPolygon(const Polygon &polygon1,
                                          const Polygon &polygon2);
    /**
     * @brief Get the Distance From Polygon To point along direction at angle.
     *
     * @param polygon
     * @param point
     * @param angle
     * @return float -1: no intersection
     */
    float GetDistanceFromPolygonToPointAtAngle(const Polygon &polygon, const Eigen::Vector2f &point, const float &angle);
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
    float
    GetAngleBetweenTwoVector(const Eigen::Vector2f &p1_start,
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
     * @brief Get the Angle Range From Point To Polygon object consider in a circle whose radius is range
     *
     * @param polygon
     * @param point
     * @param radius obstacle detection radius
     * @return AngleRange
     */
    AngleRange GetAngleRangeFromPointToPolygon(const Polygon &polygon,
                                               const Eigen::Vector2f &point, const float &radius);
    /**
     * @brief find visible vertex of a polygon from a point view, current this function only work for rectangle, return empty vector if point is on polygon
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
     * @brief determine is angle range 1 include angle range 2, ar1 is larger and include ar2.
     *
     * @param angle_range_1
     * @param angle_range_2
     * @return true
     * @return false
     */
    bool IsAngleRangeInclude(const AngleRange &angle_range_1,
                             const AngleRange &angle_range_2);
    /**
     * @brief determine is angle is inside this angle range, if angle is on boundary ,still return true
     *
     * @param angle_range
     * @param angle rad
     * @return true
     * @return false
     */
    bool IsAngleRangeInclude(const AngleRange &angle_range, const float &angle);

    /**
     * @brief angle range 1 minus angle range 2,only for overlap condition, fully included is not considered, remove ar2 from ar1, return ar1 except parts in ar2
     *
     * @param angle_range_1 this one should always be the free angle range
     * @param angle_range_2 should be obstacle angle range
     * @return AngleRange
     */
    AngleRange MinusAngleRangeOverlap(const AngleRange &angle_range_1,
                                      const AngleRange &angle_range_2);
    /**
     * @brief ar1 minus ar2, return should be a vector, size should be one only for ar1 and ar2 are overlap and size is two for ar1 include ar2
     *
     * @param ar1
     * @param ar2
     * @return std::vector<AngleRange>
     */
    std::vector<AngleRange> MinusAngleRange(const AngleRange &ar1, const AngleRange &ar2);
    /**
     * @brief find common angle range in ar1 and ar2, if nothing in common, return (-1,-1);
     *
     * @param angle_range_1
     * @param angle_range_2
     * @return AngleRange
     */
    AngleRange FindCommonAngleRange(const AngleRange &ar1,
                                    const AngleRange &ar2);
    /**
     * @brief combine angle range only if they have some range in common.
     *
     * @param angle_range_1
     * @param angle_range_2
     * @return AngleRange start and range are only negative when ar1 and ar2 not in common
     */
    AngleRange CombineAngleRange(const AngleRange &angle_range_1,
                                 const AngleRange &angle_range_2);
    /**
     * @brief calculate curvature from three points
     *
     * @param pre
     * @param current
     * @param succ
     * @return float
     */
    float CalculateCurvature(const Eigen::Vector2f &pre, const Eigen::Vector2f &current, const Eigen::Vector2f &succ);
    /**
     * @brief determine if polygon1 is intersect with polygon2,
     *
     * @param polygon1
     * @param polygon2
     * @return true: intersect
     * @return false: not intersect
     */
    bool IsPolygonIntersectWithPolygon(const Polygon &polygon1, const Polygon &polygon2);
    /**
     * @brief determine if these two polygon are share edges, in another words, they are in neighbor
     *
     * @param polygon1
     * @param polygon2
     * @return true in neighbor
     * @return false
     */
    bool IsPolygonInNeighbor(const Polygon &polygon1, const Polygon &polygon2);

    //  FindCommonEdge(const Polygon &polygon1, const Polygon &polygon2);
    /**
     * @brief combine these two polygon which are in neighbor
     *
     * @param polygon1
     * @param polygon2
     * @return Polygon
     */
    Polygon CombinePolygon(const Polygon &polygon1, const Polygon &polygon2);
    /**
     * @brief Get the Angle Range From Point To Edge At some Radius
     *
     * @param point
     * @param start edge start
     * @param end edge end
     * @param radius
     * @return AngleRange
     */
    AngleRange GetAngleRangeFromPointToEdgeAtRadius(const HybridAStar::Node3D &point, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end);

    AngleRange GetAngleRangeFromPointToEdgeAtRadius(const Eigen::Vector2f &point, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    /**
     * @brief Get the Angle Range From Point To Polygon At some radius, center should always outside polygon
     *
     * @param point circle center
     * @param radius
     * @param polygon
     * @return AngleRange
     */
    AngleRange GetAngleRangeFromPointToPolygonAtRadius(const Eigen::Vector2f &point, const float &radius, const Polygon &polygon);

    /**
     * @brief Get the Intersection Points Between Circle And Segment
     *
     * @param center circle center
     * @param radius
     * @param start
     * @param end
     * @return std::vector<Eigen::Vector2f>
     */
    std::vector<Eigen::Vector2f> GetIntersectionPointsBetweenCircleAndSegment(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    /**
     * @brief determine relationship between circle and polygon
     *
     * @param center
     * @param radius
     * @param polygon
     * @return int 1: polygon is fully inside circle, 0: polygon partially inside circle, -1: polygon is fully outside circle
     */
    int IsPolygonInsideCircle(const Eigen::Vector2f &center, const float &radius, const Polygon &polygon);
    /**
     * @brief determine if a point is inside circle, on circle is considered as outside.
     *
     * @param center
     * @param radius
     * @param point
     * @return true point inside circle
     * @return false point outside circle
     */
    bool IsPointInsideCircle(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &point);
    /**
     * @brief determine if edge is inside circle, on circle is not considered.
     *
     * @param center
     * @param radius
     * @param start
     * @param end
     * @return int 1: edge is fully inside circle, 0: edge partially inside circle, -1: edge is fully outside circle
     */
    int IsEdgeInsideCircle(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    /**
     * @brief find angle range using two angle,a1 is start,a2 is end, if angle range is greater than Pi, make it two angle rangle
     *
     * @param a1 in rad, could be negative, start angle
     * @param a2 in rad,could be negative, end angle
     * @return  std::vector<AngleRange>
     */
    std::vector<AngleRange> FindAngleRange(const float &a1, const float &a2);
    /**
     * @brief Get the Intersection Points Between Circle And Line, first assumption is there must be at least one intersection points
     *
     * @param center
     * @param radius
     * @param start start point of line
     * @param unit_vector unit vector of line
     * @return std::vector<Eigen::Vector2f>
     */
    std::vector<Eigen::Vector2f> GetIntersectionPointsBetweenCircleAndLine(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &unit_vector);
    /**
     * @brief Get the Unit Vector of two points,
     *
     * @param start
     * @param end
     * @return Eigen::Vector2f
     */
    Eigen::Vector2f GetUnitVector(const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    /**
     * @brief find projection point on a line
     *
     * @param center
     * @param start
     * @param unit_vector
     * @return Eigen::Vector2f
     */
    Eigen::Vector2f FindProjectionPoint(const Eigen::Vector2f &center, const Eigen::Vector2f &start, const Eigen::Vector2f &unit_vector);

    float GetAngleRangeStart(const AngleRange &ar);

    float GetAngleRangeEnd(const AngleRange &ar);
    /**
     * @brief check if distance from a1 to a2 is positive or negative
     * positive: CCW, negative: CW
     * @param a1 in rad
     * @param a2 in rad
     * @return true distance is positive,
     * @return false distance is negative
     */
    bool Angle1RightAngle2(const float &a1, const float &a2);
    /**
     * @brief check if distance from a1 to a2 is positive or negative
     * positive: CCW, negative: CW
     * @param a1 in rad
     * @param a2 in rad
     * @return true distance is positive,
     * @return false distance is negative
     */
    bool AngleRange1RightAngleRange2(const AngleRange &ar1, const AngleRange &ar2);
    /**
     * @brief check if a1 is equal to a2.
     *
     * @param a1
     * @param a2
     * @return true
     * @return false
     */
    bool IsEqual(const float &a1, const float &a2);
    /**
     * @brief Get the Polygon Edges which Facing Point, since polygon are rectangle, there must be one or two edges facing point.
     *
     * @param polygon
     * @param point
     * @return std::vector<Edge>
     */
    std::vector<Edge> GetPolygonEdgesFacingPoint(const Polygon &polygon, const Eigen::Vector2f &point);
    /**
     * @brief check if two angle range are equal
     *
     * @param ar1
     * @param ar2
     * @return true
     * @return false
     */
    bool IsEqual(const AngleRange &ar1, const AngleRange &ar2);
    /**
     * @brief check if two angle range have the same boundary, same start or same end, include are not counted as share boundary
     *
     * @param ar1
     * @param ar2
     * @return true
     * @return false
     */
    bool ShareBoundary(const AngleRange &ar1, const AngleRange &ar2);
    /**
     * @brief sort angle range vec by its start angle
     *
     * @param ar_vec
     * @return std::vector<AngleRange>
     */
    std::vector<AngleRange> SortAngleRange(const std::vector<AngleRange> &ar_vec);
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
                       const float &angle_range, bool consider_orientation);

    float GetDistance(const HybridAStar::Node3D &start,
                      const HybridAStar::Node3D &goal);
    float GetDistance(const HybridAStar::Node2D &start,
                      const HybridAStar::Node2D &goal);
    /**
     * @brief Get the Angle object
     *
     * @param start
     * @param goal
     * @return float in [-pi,pi]
     */
    float GetAngle(const HybridAStar::Node3D &start,
                   const HybridAStar::Node3D &goal);
    float GetAngle(const HybridAStar::Node2D &start,
                   const HybridAStar::Node2D &goal);
    float GetAngle(const Eigen::Vector2f &start, const Eigen::Vector2f &goal);
    /**
     * @brief Get the  Distance from angle1 to angle 2, positive is CCW, negative is CW
     *
     * @param angle1 in rad
     * @param angle2 in rad
     * @return float
     */
    float GetAngleDistance(const float &angle1, const float &angle2);
    /**
     * @brief Get the Angle Distance  from an angle to an angle range, return min distance from angle to ar start and ar end.
     *
     * @param angle
     * @param ar
     * @return float always be positive
     */
    float GetAngleDistance(const float &angle, const AngleRange &ar);
    /**
     * @brief check if element is already inside vector
     *
     * @param vector
     * @param element
     * @return true yes, element is inside vector
     * @return false no, element is not inside vector
     */
    bool DuplicateCheck(const std::vector<std::pair<float, float>> &vector, const std::pair<float, float> &element);
    /**
     * @brief check if element is already inside vector, if flag is true, check both element, else just check element.second;
     *
     * @param vector
     * @param element
     * @param flag
     * @return true
     * @return false
     */
    bool DuplicateCheck(const std::vector<std::pair<float, float>> &vector, const std::pair<float, float> &element, bool flag);
    /**
     * @brief compare first angle and second angle, return smaller and greater angle in a pair
     *
     * @param first_angle
     * @param second_angle
     * @return std::pair<float,float> pair first is smaller angle, second is greater angle
     */
    std::pair<float, float> CompareAngle(const float &first_angle, const float &second_angle);
    /**
     * @brief check which quadrant is this angle lie in.
     *
     * @param angle in rad
     * @return int
     */
    int CheckAngleQuadrant(const float &angle);
    /**
     * @brief normalization two angles use the same way: normalize to 0-2Pi or -pi to pi;
     *
     * @param first_angle
     * @param second_angle
     * @return std::pair<float, float>
     */
    std::pair<float, float> AngleNormalization(const float &first_angle, const float &second_angle);

    int FindIndex(const std::vector<std::pair<HybridAStar::Node3D, float>> &node3d_vec, const std::pair<HybridAStar::Node3D, float> &element);

    int FindIndex(const std::vector<HybridAStar::Node3D> &node3d_vec, const HybridAStar::Node3D &element);
    // T here should be path3d or path2d
    template <typename T>
    float GetLength(const T &t)
    {
       float total_length = 0;
       if (t.size() <= 0)
       {
          return total_length;
       }

       for (size_t i = 0; i < t.size() - 1; i++)
       {
          total_length += GetDistance(t[i], t[i + 1]);
       }
       return total_length;
    };

    std::vector<float> FormSteeringAngleVec(const float &steering_angle, const int &number_of_successors);
}
