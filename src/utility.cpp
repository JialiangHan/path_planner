#include "utility.h"

namespace Utility
{
    //*******************type conversion*******************

    HybridAStar::Node2D ConvertNode3DToNode2D(const HybridAStar::Node3D &node3d)
    {
        HybridAStar::Node2D out;
        out.setX(node3d.GetX());
        out.setY(node3d.GetY());
        return out;
    }
    void ConvertRosPathToVectorVector3D(
        const nav_msgs::Path::ConstPtr &path,
        std::vector<Eigen::Vector3f> &vector_3d_vec)
    {
        vector_3d_vec.clear();
        for (uint i = 0; i < path->poses.size(); ++i)
        {
            Eigen::Vector3f point;
            point.x() = (path->poses[i].pose.position.x);
            point.y() = (path->poses[i].pose.position.y);
            // not sure this is correct;
            point.z() = (path->poses[i].pose.position.z);
            vector_3d_vec.emplace_back(point);
        }
    }
    Eigen::Vector2f ConvertIndexToEigenVector2f(const int &index,
                                                const int &map_width)
    {
        Eigen::Vector2f out;
        out.x() = (index % map_width);
        out.y() = (index / map_width);
        return out;
    }
    nav_msgs::Path ConvertVectorVector3DToRosPath(
        const std::vector<Eigen::Vector3f> &vector_3d_vec)
    {
        nav_msgs::Path out;
        out.header.stamp = ros::Time::now();
        for (const auto &vector3d : vector_3d_vec)
        {
            geometry_msgs::PoseStamped vertex;
            vertex.pose.position.x = vector3d.x();
            vertex.pose.position.y = vector3d.y();
            vertex.pose.position.z = vector3d.z();
            vertex.pose.orientation.x = 0;
            vertex.pose.orientation.y = 0;
            vertex.pose.orientation.z = 0;
            vertex.pose.orientation.w = 0;
            out.poses.push_back(vertex);
        }
        return out;
    }
    Eigen::Vector2f ConvertVector3fToVector2f(const Eigen::Vector3f &vector_3d)
    {
        Eigen::Vector2f out;
        out.x() = vector_3d.x();
        out.y() = vector_3d.y();
        return out;
    }
    Eigen::Vector3f ConvertVector2fToVector3f(const Eigen::Vector2f &vector_2d)
    {
        Eigen::Vector3f out;
        out.x() = vector_2d.x();
        out.y() = vector_2d.y();
        out.z() = 0;
        return out;
    }
    Eigen::Vector3f ConvertNode3DToVector3f(const HybridAStar::Node3D &node3d)
    {
        Eigen::Vector3f out;
        out.x() = node3d.GetX();
        out.y() = node3d.GetY();
        out.z() = node3d.GetT();
        return out;
    }
    Eigen::Vector2f ConvertNod3DToVector2f(const HybridAStar::Node3D &node3d)
    {
        Eigen::Vector3f vector3f = ConvertNode3DToVector3f(node3d);
        return ConvertVector3fToVector2f(vector3f);
    }
    Eigen::Vector2f ConvertNod2DToVector2f(const HybridAStar::Node2D &node2d)
    {
        Eigen::Vector2f out(node2d.GetX(), node2d.GetY());
        return out;
    }
    HybridAStar::Node3D ConvertVector3fToNode3D(const Eigen::Vector3f &vector3d)
    {
        float x = vector3d.x();
        float y = vector3d.y();
        float t = vector3d.z();
        HybridAStar::Node3D node3d(x, y, t, 0, 0, nullptr);
        return node3d;
    }
    //**********************computational geometry****************

    bool OnSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p3,
                   const Eigen::Vector2f &p2)
    {
        if (p2.x() <= std::max(p1.x(), p3.x()) &&
            p2.x() >= std::min(p1.x(), p3.x()) &&
            p2.y() <= std::max(p1.y(), p3.y()) &&
            p2.y() >= std::min(p1.y(), p3.y()))
        {
            Eigen::Vector2f p1p2 = p2 - p1;
            Eigen::Vector2f p1p3 = p3 - p1;
            if (abs(p1p2.dot(p1p3) / p1p2.norm() / p1p3.norm() - 1) < 1e-6)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    float CrossProduct(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
    {
        float out;
        out = p1.x() * p2.y() - p2.x() * p1.y();
        // DLOG(INFO) << "result is " << out;
        return out;
    }

    int IsIntersect(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                    const Eigen::Vector2f &p3, const Eigen::Vector2f &p4)
    {
        if ((p1.x() == p2.x() && p2.x() == p3.x() && p3.x() == p4.x()) || (p1.y() == p2.y() && p2.y() == p3.y() && p3.y() == p4.y()))
        {
            //these segments are have same coordinate. maybe they are overlapped.
            return 0;
        }
        if (std::max(p3.x(), p4.x()) < std::min(p1.x(), p2.x()) ||
            std::max(p3.y(), p4.y()) < std::min(p1.y(), p2.y()) ||
            std::max(p1.x(), p2.x()) < std::min(p3.x(), p4.x()) ||
            std::max(p1.y(), p2.y()) < std::min(p3.y(), p4.y()))
        {
            // DLOG(INFO) << "these two segments are far away!";
            return 0;
        }
        else
        {
            if (CrossProduct(p1 - p3, p4 - p3) * CrossProduct(p2 - p3, p4 - p3) <= 0 &&
                CrossProduct(p3 - p2, p1 - p2) * CrossProduct(p4 - p2, p1 - p2) <= 0)
            {
                // DLOG(INFO) << "Intersection!!";
                return 1;
            }
            return 0;
        }
    }

    Eigen::Vector2f FindIntersectionPoint(const Eigen::Vector2f &p1,
                                          const Eigen::Vector2f &p2,
                                          const Eigen::Vector2f &p3,
                                          const Eigen::Vector2f &p4)
    {
        Eigen::Vector2f out;
        if (0 == IsIntersect(p1, p2, p3, p4))
        {
            out.x() = 10000;
            out.y() = 10000;
        }
        else
        {
            Eigen::Vector2f dir_1 = p2 - p1, dir_2 = p4 - p3;
            float t;
            t = CrossProduct(p3 - p1, dir_2) / CrossProduct(dir_1, dir_2);
            out = p1 + t * dir_1;
        }
        return out;
    }
    int IsAboveSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                       const Eigen::Vector2f &p3)
    {
        int above = 1, on_segment = 1, below = 0;
        if (OnSegment(p1, p2, p3))
        {
            return on_segment;
        }
        else
        {
            if ((p2 - p1).x() == 0)
            {
                if (p3.x() > p1.x())
                {
                    return below;
                }
                else
                {
                    return above;
                }
            }
            else if ((p2 - p1).y() == 0)
            {
                if (p3.y() > p1.y())
                {
                    return above;
                }
                else
                {
                    return below;
                }
            }
            else
            {
                float k = (p2 - p1).y() / (p2 - p1).x();
                float b = p1.y() - k * p1.x();
                if (k * p3.x() + b - p3.y() < 0)
                {
                    return above;
                }
                else
                {
                    return below;
                }
            }
        }
    }
    int IsInsidePolygon(const Polygon &polygon, const Eigen::Vector2f &point)
    {
        Eigen::Vector2f far_away_point(10000, point.y());
        int number_intersection = 0;
        for (uint i = 0; i < polygon.size() - 1; ++i)
        {
            // check if point is on polygon edge
            if (OnSegment(polygon[i], polygon[i + 1], point))
            {
                // DLOG(INFO) << "Point is on polygon edge.";
                return 1;
            }
            // if edge intersect with vector from point to far away point
            if (IsIntersect(point, far_away_point, polygon[i], polygon[i + 1]) == 1)
            {
                Eigen::Vector2f intersection_point = FindIntersectionPoint(
                    point, far_away_point, polygon[i], polygon[i + 1]);
                // DLOG(INFO) << "point horizontal vector is intersecting polygon!";
                if ((polygon[i] - intersection_point).norm() < 1e-3)
                {
                    if (IsAboveSegment(point, far_away_point, polygon[i + 1]) == 0)
                    {
                        number_intersection += 1;
                        // DLOG(INFO) << "true intersection!! +1";
                    }
                }
                else if ((polygon[i + 1] - intersection_point).norm() < 1e-3)
                {
                    if (IsAboveSegment(point, far_away_point, polygon[i]) == 0)
                    {
                        number_intersection += 1;
                        // DLOG(INFO) << "true intersection!! +1";
                    }
                }
                else
                {
                    number_intersection += 1;
                    // DLOG(INFO) << "true intersection!! +1";
                }
            }
        }
        // DLOG(INFO) << "number of intersection is " << number_intersection;
        if ((number_intersection % 2) == 0)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    int IsInsidePolygon(const Polygon &polygon, const Eigen::Vector3f &point)
    {
        Eigen::Vector2f point_2d = ConvertVector3fToVector2f(point);
        return IsInsidePolygon(polygon, point_2d);
    }
    Polygon CreatePolygon(const float &width, const float &height)
    {
        Polygon polygon;
        polygon.emplace_back(Eigen::Vector2f(0, 0));
        polygon.emplace_back(Eigen::Vector2f(width, 0));
        polygon.emplace_back(Eigen::Vector2f(width, height));
        polygon.emplace_back(Eigen::Vector2f(0, height));
        polygon.emplace_back(Eigen::Vector2f(0, 0));
        return polygon;
    }
    Polygon CreatePolygon(const Eigen::Vector2f &origin, const float &width,
                          const float &height)
    {
        Polygon polygon;
        polygon.emplace_back(origin);
        polygon.emplace_back(Eigen::Vector2f(width + origin.x(), origin.y()));
        polygon.emplace_back(
            Eigen::Vector2f(width + origin.x(), origin.y() + height));
        polygon.emplace_back(Eigen::Vector2f(origin.x(), origin.y() + height));
        polygon.emplace_back(origin);
        return polygon;
    }
    float GetDistanceFromPointToPoint(const Eigen::Vector3f &vector_3d,
                                      const Eigen::Vector2f &vector_2d)
    {
        return GetDistanceFromPointToPoint(ConvertVector3fToVector2f(vector_3d),
                                           vector_2d);
    }
    float GetDistanceFromPointToPoint(const Eigen::Vector2f &p1,
                                      const Eigen::Vector2f &p2)
    {
        return (p1 - p2).norm();
    }
    int IsSegmentIntersectWithPolygon(const Polygon &polygon,
                                      const Eigen::Vector2f &start,
                                      const Eigen::Vector2f &end)
    {
        for (uint i = 0; i < polygon.size() - 1; ++i)
        {
            if (IsIntersect(polygon[i], polygon[i + 1], start, end))
            {
                return 1;
            }
        }
        return 0;
    }
    float GetDistanceFromSegmentToPoint(const Eigen::Vector2f &start,
                                        const Eigen::Vector2f &end,
                                        const Eigen::Vector2f &point)
    {
        if (OnSegment(start, end, point))
        {
            return 0;
        }
        Eigen::Vector2f far_point;
        float distance_to_start, distance_to_goal;
        distance_to_start = GetDistanceFromPointToPoint(start, point);
        distance_to_goal = GetDistanceFromPointToPoint(end, point);
        if (distance_to_start > distance_to_goal)
        {
            far_point = start;
        }
        else
        {
            far_point = end;
        }
        float projection_length =
            abs((point - far_point).dot(end - start)) / (end - start).norm();
        if (projection_length > (end - start).norm())
        {
            // this means point is outside segment
            return std::min(distance_to_start, distance_to_goal);
        }
        return sqrt(std::pow((point - far_point).norm(), 2) -
                    std::pow(projection_length, 2));
    }
    float GetDistanceFromPolygonToPoint(const Polygon &polygon,
                                        const Eigen::Vector2f &point)
    {
        float min_distance = 100000;
        if (IsInsidePolygon(polygon, point))
        {
            return -1;
        }
        for (uint i = 0; i < polygon.size() - 1; ++i)
        {
            float current_distance =
                GetDistanceFromSegmentToPoint(polygon[i], polygon[i + 1], point);
            if (current_distance < min_distance)
            {
                min_distance = current_distance;
            }
        }
        return min_distance;
    }
    float GetAngleBetweenTwoVector(const Eigen::Vector2f &p1_start,
                                   const Eigen::Vector2f &p1_end,
                                   const Eigen::Vector2f &p2_start,
                                   const Eigen::Vector2f &p2_end)
    {
        Eigen::Vector2f v1 = p1_end - p1_start;
        Eigen::Vector2f v2 = p2_end - p1_start;
        // return of acos is in [0,pi]
        float angle = std::acos(v1.dot(v2) / v1.norm() / v2.norm());
        float sign = CrossProduct(v1, v2);
        if (sign > 0)
        {
            return angle;
        }
        else
        {
            return -angle;
        }
    }
    AngleRange GetAngleRangeFromPointToSegment(const Eigen::Vector2f &start,
                                               const Eigen::Vector2f &end,
                                               const Eigen::Vector2f &point)
    {
        Eigen::Vector2f horizontal(10000, point.y());
        float start_angle = GetAngleBetweenTwoVector(point, horizontal, point, start);
        float range_length = GetAngleBetweenTwoVector(point, start, point, end);
        return std::make_pair(RadToZeroTo2P(start_angle), range_length);
    }
    //checked
    AngleRange GetAngleRangeFromPointToPolygon(const Polygon &polygon,
                                               const Eigen::Vector2f &point)
    {
        float starting_angle = 100000, range = 0;
        std::vector<Eigen::Vector2f> visible_vertex = FindVisibleVertexFromNode(polygon, point);
        // DLOG(INFO) << "point is " << point.x() << " " << point.y();
        // DLOG(INFO) << "size of visible vertex is " << visible_vertex.size();
        for (uint index = 0; index < visible_vertex.size() - 1; ++index)
        {
            AngleRange angle_range1 = GetAngleRangeFromPointToSegment(visible_vertex[index], visible_vertex[index + 1], point);
            AngleRange angle_range2 = GetAngleRangeFromPointToSegment(visible_vertex[index + 1], visible_vertex[index], point);
            AngleRange angle_range;

            //use smallest starting angle as new start angle for polygon
            if (angle_range1.first > angle_range2.first)
            {

                angle_range.first = angle_range2.first;
                angle_range.second = angle_range2.second;
            }
            else
            {
                angle_range.first = angle_range1.first;
                angle_range.second = angle_range1.second;
            }
            // DLOG(INFO) << "vertex angle range start is " << Utility::ConvertRadToDeg(angle_range.first) << " range is " << Utility::ConvertRadToDeg(angle_range.second);
            starting_angle = starting_angle > angle_range.first ? angle_range.first : starting_angle;
            // DLOG(INFO) << "angle range start is " << Utility::ConvertRadToDeg(starting_angle);
            // DLOG(INFO) << "range is " << range;
            range += angle_range.second;
            // DLOG(INFO) << "range is " << Utility::ConvertRadToDeg(range);
        }

        return std::make_pair(starting_angle, abs(range));
    }
    std::vector<Eigen::Vector2f>
    FindVisibleVertexFromNode(const Polygon &polygon,
                              const Eigen::Vector2f &point)
    {
        std::vector<Eigen::Vector2f> out;
        bool intersect_flag = false;
        for (uint i = 0; i < polygon.size() - 1; ++i)
        {
            for (uint index = 0; index < polygon.size() - 1; ++index)
            {
                if (polygon[i] != polygon[index] && polygon[i] != polygon[index + 1])
                {
                    if (IsIntersect(point, polygon[i], polygon[index], polygon[index + 1]) == 1)
                    {
                        intersect_flag = true;
                    }
                }
            }
            if (!intersect_flag)
            {
                // DLOG(INFO) << "vertex is " << polygon[i].x() << " " << polygon[i].y();
                out.emplace_back(polygon[i]);
            }
            intersect_flag = false;
        }
        return out;
    }
    bool IsOverlap(const AngleRange &angle_range_1,
                   const AngleRange &angle_range_2)
    {
        float angle_range_1_start = angle_range_1.first;
        float angle_range_1_end = angle_range_1.first + angle_range_1.second;
        float angle_range_2_start = angle_range_2.first;
        float angle_range_2_end = angle_range_2.first + angle_range_2.second;
        // far away
        if (angle_range_2_start > angle_range_1_end ||
            angle_range_1_start > angle_range_2_end)
        {
            return false;
        }
        // include
        else if ((angle_range_1_start <= angle_range_2_start &&
                  angle_range_1_end >= angle_range_2_end) ||
                 (angle_range_2_start <= angle_range_1_start &&
                  angle_range_2_end >= angle_range_1_end))
        {
            return false;
        }
        // overlap
        else if ((angle_range_1_end >= angle_range_2_end &&
                  angle_range_1_start >= angle_range_2_start) ||
                 (angle_range_2_end >= angle_range_1_end &&
                  angle_range_2_start >= angle_range_1_start))
        {
            return true;
        }
        return false;
    }
    // checked
    bool IsAngleRangeInclude(const AngleRange &angle_range_1,
                             const AngleRange &angle_range_2)
    {
        //all input should be in rad.
        float angle_range_1_start = angle_range_1.first;
        float angle_range_1_end = angle_range_1.first + angle_range_1.second;
        float angle_range_2_start = angle_range_2.first;
        float angle_range_2_end = angle_range_2.first + angle_range_2.second;
        if (angle_range_1_start > M_PI && angle_range_1_end > M_PI)
        {
            angle_range_1_start = Utility::RadNormalization(angle_range_1_start);
            angle_range_1_end = Utility::RadNormalization(angle_range_1_end);
        }
        if (angle_range_2_start > M_PI && angle_range_2_end > M_PI)
        {
            angle_range_2_start = Utility::RadNormalization(angle_range_2_start);
            angle_range_2_end = Utility::RadNormalization(angle_range_2_end);
        }
        // DLOG(INFO) << "ar1 start is " << Utility::ConvertRadToDeg(angle_range_1_start) << " end is " << Utility::ConvertRadToDeg(angle_range_1_end) << " ar2 start is " << Utility::ConvertRadToDeg(angle_range_2_start) << " end is " << Utility::ConvertRadToDeg(angle_range_2_end);
        // DLOG(INFO) << "ar1 start smaller than ar2 start :" << (angle_range_1_start <= angle_range_2_start);
        // DLOG(INFO) << "ar1 end greater than ar2 end :" << (angle_range_1_end >= angle_range_2_end);
        if (((angle_range_1_start <= angle_range_2_start) || abs(angle_range_1_start - angle_range_2_start) < 1e-3) && angle_range_1_end >= angle_range_2_end)
        {
            return true;
        }

        return false;
    }
    // checked
    AngleRange MinusAngleRange(const AngleRange &angle_range_1,
                               const AngleRange &angle_range_2)
    {
        AngleRange out;
        if (!IsOverlap(angle_range_1, angle_range_2))
        {
            DLOG(INFO) << "angle range 1 and angle range 2 is not overlap!";
            return out;
        }
        float angle_range_1_start = angle_range_1.first;
        float angle_range_1_end = angle_range_1.first + angle_range_1.second;
        float angle_range_2_start = angle_range_2.first;
        float angle_range_2_end = angle_range_2.first + angle_range_2.second;
        DLOG(INFO) << "ar1 start is " << Utility::ConvertRadToDeg(angle_range_1_start) << " end is " << Utility::ConvertRadToDeg(angle_range_1_end) << " ar2 start is " << Utility::ConvertRadToDeg(angle_range_2_start) << " end is " << Utility::ConvertRadToDeg(angle_range_2_end);
        //case 1, start of first<start of second<end of first<end of second
        if (angle_range_1_start <= angle_range_2_start && angle_range_2_start <= angle_range_1_end && angle_range_1_end <= angle_range_2_end)
        {
            out.first = angle_range_1_start;
            out.second = angle_range_2_start - angle_range_1_start;
        }
        //case 2, start of second<start of first<end of second<end of first
        if (angle_range_2_start <= angle_range_1_start && angle_range_1_start <= angle_range_2_end && angle_range_2_end <= angle_range_1_end)
        {
            out.first = angle_range_2_end;
            out.second = angle_range_1_end - angle_range_2_end;
        }
        return out;
    }
    //*************************other ***********************
    float Clamp(const float &number, const float &upper_bound,
                const float &lower_bound)
    {
        return std::max(lower_bound, std::min(number, upper_bound));
    }
    float GetDistance(const HybridAStar::Node2D &start,
                      const HybridAStar::Node2D &goal)
    {
        return GetDistanceFromPointToPoint(ConvertNod2DToVector2f(start),
                                           ConvertNod2DToVector2f(goal));
    }
    float GetDistance(const HybridAStar::Node3D &start,
                      const HybridAStar::Node3D &goal)
    {
        return GetDistanceFromPointToPoint(ConvertNod3DToVector2f(start),
                                           ConvertNod3DToVector2f(goal));
    }
    bool IsCloseEnough(const HybridAStar::Node3D &start,
                       const HybridAStar::Node3D &goal, const float &distance_range,
                       const float &angle_range)
    {
        float distance = GetDistance(start, goal);
        if (distance < distance_range)
        {
            float angle_diff = RadNormalization(start.GetT() - goal.GetT());

            if (abs(angle_diff) <= angle_range)
            {
                DLOG(INFO)
                    << "two node distance and orientation are close enough, return true";
                // DLOG(INFO) << "current node is " << start.GetX() << " " << start.GetY() << " " << Utility::ConvertRadToDeg(start.GetT());
                // DLOG(INFO) << "goal is " << goal.GetX() << " " << goal.GetY() << " " << Utility::ConvertRadToDeg(goal.GetT());
                // DLOG(INFO) << "angle diff is " << Utility::ConvertRadToDeg(angle_diff) << " angle range is " << Utility::ConvertRadToDeg(angle_range);
                return true;
            }
            else
            {
                DLOG(INFO)
                    << "two node distance is close enough but orientation is too far is "
                    << Utility::ConvertRadToDeg(angle_diff);
                return false;
            }
        }
        else
        {
            // DLOG(INFO) << "too far, distance is " << distance;
            return false;
        }
    }
    float ConvertDegToRad(const float &deg)
    {
        float rad;
        rad = deg / 360 * 2 * M_PI;
        return (rad);
    }
    float ConvertRadToDeg(const float &rad)
    {
        float deg;
        deg = rad / 2 / M_PI * 360;
        return (deg);
    }
    float DegNormalization(const float &t)
    {
        float out;
        out = fmod(t, 360);
        if (out < 0)
        {
            out += 360;
        }
        if (out > 180)
        {
            out = out - 360;
        }
        return out;
    }
    float RadNormalization(const float &rad)
    {
        float out;
        out = fmod(rad, 2.f * M_PI);
        if (out < 0)
        {
            out += 2.f * M_PI;
        }
        if (out > M_PI)
        {
            out = out - 2.0f * M_PI;
        }
        if (abs(out) < 1e-3)
        {
            out = 0;
        }
        return out;
    }

    float DegToZeroTo2P(const float &deg)
    {
        float out;
        out = fmod(deg, 360);
        if (out < 0)
        {
            out += 360;
        }

        return out;
    }

    float RadToZeroTo2P(const float &rad)
    {
        float out;
        out = fmod(rad, 2.f * M_PI);
        if (out < 0)
        {
            out += 2.f * M_PI;
        }
        if (abs(out - 2 * M_PI) < 1e-3)
        {
            out = 0;
        }

        return out;
    }
    float GetAngle(const HybridAStar::Node3D &start,
                   const HybridAStar::Node3D &goal)
    {
        return GetAngle(ConvertNode3DToNode2D(start), ConvertNode3DToNode2D(goal));
    }
    float GetAngle(const HybridAStar::Node2D &start,
                   const HybridAStar::Node2D &goal)
    {
        Eigen::Vector2f diff;
        diff = ConvertNod2DToVector2f(start) - ConvertNod2DToVector2f(goal);
        return atan2(diff.y(), diff.x());
    }
} // namespace Utility
