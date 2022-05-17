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
            out.poses.emplace_back(vertex);
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
            // DLOG(INFO) << "answer is " << abs(p1p2.dot(p1p3) / p1p2.norm() / p1p3.norm() - 1);
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
    // checked, correct.
    bool IsPointOnLine(const Eigen::Vector2f &point, const Eigen::Vector2f &start, const Eigen::Vector2f &unit_vector)
    {
        Eigen::Vector2f diff = point - start;
        float angle = acos(diff.dot(unit_vector) / diff.norm() / unit_vector.norm());
        // DLOG(INFO) << "angle is " << angle;
        // DLOG(INFO) << "angle is " << (angle < 1e-3);
        // DLOG(INFO) << "angle -pI " << ((angle - M_PI) < 1e-3);
        if (abs(angle) < 1e-3 || (abs(angle - M_PI) < 1e-3))
        {
            // DLOG(INFO) << "point " << point.x() << " " << point.y() << " is on the line!";
            return true;
        }
        return false;
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
            // these segments are have same coordinate. maybe they are overlapped.
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
            // DLOG(INFO) << "Point " << point.x() << " " << point.y() << " is outside polygon  first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y();

            return 0;
        }
        else
        {
            // DLOG(INFO) << "Point " << point.x() << " " << point.y() << " is inside polygon  first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y();
            return 1;
        }
    }
    int IsInsidePolygon(const Polygon &polygon, const Eigen::Vector3f &point)
    {
        Eigen::Vector2f point_2d = ConvertVector3fToVector2f(point);
        return IsInsidePolygon(polygon, point_2d);
    }

    bool IsOnPolygon(const Polygon &polygon, const Eigen::Vector2f &point)
    {
        for (uint i = 0; i < polygon.size() - 1; ++i)
        {
            // check if point is on polygon edge
            if (OnSegment(polygon[i], polygon[i + 1], point))
            {
                // DLOG(INFO) << "Point " << point.x() << " " << point.y() << " is on polygon edge (start: " << polygon[i].x() << " " << polygon[i].y() << " end: " << polygon[i + 1].x() << " " << polygon[i + 1].y() << " )";
                return true;
            }
        }
        // DLOG(INFO) << "Point is not on polygon edge,";
        return false;
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
    // checked, correct
    Polygon CreatePolygon(const Eigen::Vector2f &origin, const float &width,
                          const float &height)
    {
        Polygon polygon;

        Eigen::Vector2f second_point(width + origin.x(), origin.y());
        Eigen::Vector2f third_point(width + origin.x(), origin.y() + height);
        Eigen::Vector2f fourth_point(Eigen::Vector2f(origin.x(), origin.y() + height));
        polygon.emplace_back(origin);
        polygon.emplace_back(second_point);
        polygon.emplace_back(third_point);
        polygon.emplace_back(fourth_point);
        polygon.emplace_back(origin);

        // DLOG(INFO) << "first point is " << origin.x() << " " << origin.y() << " second_point is " << second_point.x() << " " << second_point.y() << " third_point is " << third_point.x() << " " << third_point.y() << " fourth_point is " << fourth_point.x() << " " << fourth_point.y();

        return polygon;
    }

    Polygon CreatePolygon(const Eigen::Vector2f &center, const float &width,
                          const float &height, const float &heading)
    {
        Polygon polygon;
        Eigen::Vector2f first_point(-width / 2, -height / 2);
        Eigen::Vector2f second_point(+width / 2, -height / 2);
        Eigen::Vector2f third_point(+width / 2, +height / 2);
        Eigen::Vector2f fourth_point(-width / 2, +height / 2);
        // DLOG(INFO) << "first point is " << first_point.x() << " " << first_point.y();
        // DLOG(INFO) << "second_point is " << second_point.x() << " " << second_point.y();
        // DLOG(INFO) << "third_point is " << third_point.x() << " " << third_point.y();
        // DLOG(INFO) << "fourth_point is " << fourth_point.x() << " " << fourth_point.y();
        // rotation
        Eigen::Matrix2f rotation_matrix;
        rotation_matrix(0, 0) = cos(heading);
        rotation_matrix(0, 1) = -sin(heading);
        rotation_matrix(1, 0) = sin(heading);
        rotation_matrix(1, 1) = cos(heading);
        first_point = center + rotation_matrix * first_point;
        second_point = center + rotation_matrix * second_point;
        third_point = center + rotation_matrix * third_point;
        fourth_point = center + rotation_matrix * fourth_point;
        // DLOG(INFO) << "first point is " << first_point.x() << " " << first_point.y();
        // DLOG(INFO) << "second_point is " << second_point.x() << " " << second_point.y();
        // DLOG(INFO) << "third_point is " << third_point.x() << " " << third_point.y();
        // DLOG(INFO) << "fourth_point is " << fourth_point.x() << " " << fourth_point.y();
        // push back
        polygon.emplace_back(first_point);
        polygon.emplace_back(second_point);
        polygon.emplace_back(third_point);
        polygon.emplace_back(fourth_point);
        polygon.emplace_back(first_point);

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
        Eigen::Vector2f far_point, unit_vector = GetUnitVector(start, end);
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
        return GetDistanceFromPointToLine(point, start, unit_vector);
    }
    // checked, correct.
    float GetDistanceFromPointToLine(const Eigen::Vector2f &point,
                                     const Eigen::Vector2f &start,
                                     const Eigen::Vector2f &unit_vector)
    {
        if (IsPointOnLine(point, start, unit_vector))
        {
            return 0;
        }

        float projection_length = abs((point - start).dot(unit_vector));

        return sqrt((point - start).norm() * (point - start).norm() - projection_length * projection_length);
    }

    float GetDistanceFromPolygonToPoint(const Polygon &polygon,
                                        const Eigen::Vector2f &point)
    {
        float min_distance = 100000;
        // check point is on polygon edge
        if (IsOnPolygon(polygon, point))
        {
            return 0;
        }
        // check point is inside polygon
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
    // checked, correct.
    float GetAngleBetweenTwoVector(const Eigen::Vector2f &p1_start,
                                   const Eigen::Vector2f &p1_end,
                                   const Eigen::Vector2f &p2_start,
                                   const Eigen::Vector2f &p2_end)
    {
        // DLOG(INFO) << "GetAngleBetweenTwoVector in:";
        Eigen::Vector2f v1 = p1_end - p1_start;
        Eigen::Vector2f v2 = p2_end - p1_start;
        // return of acos is in [0,pi]
        float angle = std::acos(v1.dot(v2) / v1.norm() / v2.norm());
        float sign = CrossProduct(v1, v2);
        // DLOG(INFO) << "angle is " << ConvertRadToDeg(angle) << " sign is " << ConvertRadToDeg(sign);
        if (sign >= 0)
        {
            // DLOG(INFO) << "GetAngleBetweenTwoVector out.";
            return angle;
        }
        else
        {
            // DLOG(INFO) << "GetAngleBetweenTwoVector out.";
            return -angle;
        }
    }
    // checked,correct
    AngleRange GetAngleRangeFromPointToSegment(const Eigen::Vector2f &start,
                                               const Eigen::Vector2f &end,
                                               const Eigen::Vector2f &point)
    {
        // DLOG(INFO) << "GetAngleRangeFromPointToSegment in:";
        Eigen::Vector2f horizontal(10000, point.y());
        float start_angle = GetAngleBetweenTwoVector(point, horizontal, point, start);
        float range_length = GetAngleBetweenTwoVector(point, horizontal, point, end);
        AngleRange out = FindAngleRange(start_angle, range_length);
        // DLOG(INFO) << "start angle is " << ConvertRadToDeg(start_angle) << " "
        //            << " range_length is " << ConvertRadToDeg(range_length);
        // DLOG(INFO) << "range start is " << ConvertRadToDeg(out.first) << " "
        //            << " range is " << ConvertRadToDeg(out.second);
        // DLOG(INFO) << "GetAngleRangeFromPointToSegment out.";
        return out;
    }

    // checked
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

            // use smallest starting angle as new start angle for polygon
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

    AngleRange GetAngleRangeFromPointToPolygon(const Polygon &polygon,
                                               const Eigen::Vector2f &point, const float &radius)
    {
        DLOG(INFO) << "GetAngleRangeFromPointToPolygon in:";
        AngleRange out;
        // 1. check if polygon is inside this circle
        float distance = Utility::GetDistanceFromPolygonToPoint(polygon, point);
        if (distance > radius)
        {
            DLOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << "  and distance to point: " << point.x() << " " << point.y() << " is " << distance << " range is " << radius;
            return out;
        }

        // 2. find angle which has distance equal to radius from point to polygon
        out = GetAngleRangeFromPointToPolygonAtRadius(point, radius, polygon);
        DLOG(INFO) << "GetAngleRangeFromPointToPolygon out.";
        return out;
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
        // all input should be in rad.
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
        // DLOG(INFO) << "ar1 start is " << Utility::ConvertRadToDeg(angle_range_1_start) << " end is " << Utility::ConvertRadToDeg(angle_range_1_end) << " ar2 start is " << Utility::ConvertRadToDeg(angle_range_2_start) << " end is " << Utility::ConvertRadToDeg(angle_range_2_end);
        // case 1, start of first<start of second<end of first<end of second
        if (angle_range_1_start <= angle_range_2_start && angle_range_2_start <= angle_range_1_end && angle_range_1_end <= angle_range_2_end)
        {
            out.first = angle_range_1_start;
            out.second = angle_range_2_start - angle_range_1_start;
        }
        // case 2, start of second<start of first<end of second<end of first
        if (angle_range_2_start <= angle_range_1_start && angle_range_1_start <= angle_range_2_end && angle_range_2_end <= angle_range_1_end)
        {
            out.first = angle_range_2_end;
            out.second = angle_range_1_end - angle_range_2_end;
        }
        return out;
    }
    float CalculateCurvature(const Eigen::Vector2f &pre, const Eigen::Vector2f &current, const Eigen::Vector2f &succ)
    {
        float curvature = 0;
        // get three points from path

        if (pre == current || current == succ)
        {
            DLOG(WARNING) << "In CalculateCurvature: some points are equal, skip these points for curvature calculation!!";
            return curvature;
        }

        // get two vector between these three nodes
        Eigen::Vector2f pre_vector = current - pre;

        Eigen::Vector2f succ_vector = succ - current;

        // calculate delta distance and delta angle
        float delta_distance = succ_vector.norm();
        float pre_vector_length = pre_vector.norm();

        // there would some calculation error here causing number inside acos greater than 1 or smaller than -1.
        float delta_angle = std::acos(Clamp(pre_vector.dot(succ_vector) / (delta_distance * pre_vector_length), 1, -1));

        curvature = abs(delta_angle) / pre_vector_length;
        // DLOG(INFO) << "succ x is :" << succ(0,0) << "y is: " << succ.y();
        // DLOG(INFO) << "pre_vector x is :" << pre_vector(0,0) << "y is: " << pre_vector.y();
        // DLOG(INFO) << "succ_vector x is :" << succ_vector(0,0) << "y is: " << succ_vector.y();
        // DLOG(INFO) << "delta_distance is:" << delta_distance;
        // DLOG(INFO) << "pre_vector_length is: " << pre_vector_length;
        // DLOG(INFO) << "delta_angle is: " << delta_angle;
        return curvature;
    }

    bool IsPolygonIntersectWithPolygon(const Polygon &polygon1, const Polygon &polygon2)
    {
        for (uint index = 0; index < polygon1.size() - 1; ++index)
        {
            if (IsSegmentIntersectWithPolygon(polygon2, polygon1[index], polygon1[index + 1]))
            {
                return true;
            }
        }
        return false;
    }
    float GetDistanceFromPolygonToSegment(const Polygon &polygon,
                                          const Eigen::Vector2f &start,
                                          const Eigen::Vector2f &end)
    {
        float out = 100000;
        if (IsSegmentIntersectWithPolygon(polygon, start, end))
        {
            return -1;
        }
        // get distance from point to polygon
        float temp_distance = GetDistanceFromPolygonToPoint(polygon, start);
        if (out > temp_distance)
        {
            out = temp_distance;
        }
        temp_distance = GetDistanceFromPolygonToPoint(polygon, end);
        if (out > temp_distance)
        {
            out = temp_distance;
        }
        // get distance from segment to vertex of polygon
        for (const auto &vertex : polygon)
        {
            temp_distance = GetDistanceFromSegmentToPoint(start, end, vertex);
            if (out > temp_distance)
            {
                out = temp_distance;
            }
        }
        return out;
    }

    bool IsPolygonInNeighbor(const Polygon &polygon1, const Polygon &polygon2)
    {
        for (const auto &vertex1 : polygon1)
        {
            for (const auto &vertex2 : polygon2)
            {
                if (vertex1 == vertex2)
                {
                    return true;
                }
            }
        }
        return false;
    }

    Polygon CombinePolyon(const Polygon &polygon1, const Polygon &polygon2)
    {
        Polygon out;
        if (!IsPolygonInNeighbor(polygon1, polygon2))
        {
            return out;
        }
        // // put all vertex in out
        // out.insert(out.end(), polygon1.begin(), polygon1.end());
        // out.insert(out.end(), polygon2.begin(), polygon2.end());

        // // sort by x and then y
        // std::sort(out.begin(), out.end(), [](Eigen::Vector2f a, Eigen::Vector2f b) -> bool
        //           { return a.x() < b.x() ? a.x() < b.x() : a.y() < b.y(); });
        // // remove duplicates
        // out.erase(unique(out.begin(), out.end()), out.end());
        // out.insert(out.end(), out[0]);
        // if polygon2 is on top of polygon1
        if (polygon2[0].y() > polygon1[0].y())
        {
            out.emplace_back(polygon1[0]);
            out.emplace_back(polygon1[1]);
            out.emplace_back(polygon2[2]);
            out.emplace_back(polygon2[3]);
            out.emplace_back(polygon1[0]);
        }
        // if polygon2 is at left of polygon1
        else if (polygon2[0].x() < polygon1[0].x())
        {
            out.emplace_back(polygon2[0]);
            out.emplace_back(polygon1[1]);
            out.emplace_back(polygon1[2]);
            out.emplace_back(polygon2[3]);
            out.emplace_back(polygon2[0]);
        }
        // if polygon2 is at bottom of polygon1
        else if (polygon2[0].y() < polygon1[0].y())
        {
            out.emplace_back(polygon2[0]);
            out.emplace_back(polygon2[1]);
            out.emplace_back(polygon1[2]);
            out.emplace_back(polygon1[3]);
            out.emplace_back(polygon2[0]);
        }
        // if polygon2 is at right of polygon1
        else if (polygon2[0].x() > polygon1[0].x())
        {
            out.emplace_back(polygon1[0]);
            out.emplace_back(polygon2[1]);
            out.emplace_back(polygon2[2]);
            out.emplace_back(polygon1[3]);
            out.emplace_back(polygon1[0]);
        }
        return out;
    }

    float GetDistanceFromPolygonToPolygon(const Polygon &polygon1,
                                          const Polygon &polygon2)
    {
        float out = 10000;
        for (uint index = 0; index < polygon1.size() - 1; ++index)
        {
            float temp = GetDistanceFromPolygonToSegment(polygon2, polygon1[index], polygon1[index + 1]);
            out = out >= temp ? temp : out;
        }
        return out;
    }
    // checked in test.cpp
    float GetDistanceFromPolygonToPointAtAngle(const Polygon &polygon, const Eigen::Vector2f &point, const float &angle)
    {
        // DLOG(INFO) << "GetDistanceFromPolygonToPointAtAngle in:";
        float out = -1;
        ComputationalGeometry::Segment segment(point, 10000, angle);
        if (!Utility::IsSegmentIntersectWithPolygon(polygon, segment.GetStart(), segment.GetEnd()))
        {
            // DLOG(INFO) << "NO intersect with obstacles!!";
            // DLOG(INFO) << "GetDistanceFromPolygonToPointAtAngle out.";
            return out;
        }
        // 1. check if polygon and segment start at point intersect?
        for (uint index = 0; index < polygon.size() - 1; ++index)
        {
            ComputationalGeometry::Segment segment_on_polygon(polygon[index], polygon[index + 1]);
            if (segment.IsIntersect(segment_on_polygon))
            {
                float distance = (segment.FindIntersectionPoint(segment_on_polygon) - point).norm();
                // DLOG(INFO) << "distance is " << distance;
                if (out > distance || out < 0)
                {
                    out = distance;
                }
            }
        }
        // 2. if so, find segment in polygon which is intersect with segment start at point
        // 3. from intersect point, calculate distance
        // DLOG(INFO) << "GetDistanceFromPolygonToPointAtAngle out.";
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
        return GetAngle(ConvertNod2DToVector2f(start), ConvertNod2DToVector2f(goal));
    }
    // checked, correct
    float GetAngle(const Eigen::Vector2f &start, const Eigen::Vector2f &goal)
    {
        Eigen::Vector2f diff;
        diff = goal - start;
        float angle = RadToZeroTo2P(atan2(diff.y(), diff.x()));
        // make sure angle is in [0,2pi]

        // DLOG(INFO) << "angle is " << ConvertRadToDeg(angle);
        return angle;
    }

    bool IsAngleRangeInclude(const AngleRange &angle_range, const float &angle)
    {
        // if angle is in between angle range start and angle range end, then it`s include
        float angle_range_end = angle_range.first + angle_range.second;
        // DLOG(INFO) << "angle range start " << ConvertRadToDeg(angle_range.first) << " angle range end " << ConvertRadToDeg(angle_range_end) << " angle is " << ConvertRadToDeg(angle);
        // first check 0-360
        if (angle > angle_range.first && angle < angle_range_end)
        {
            return true;
        }

        // second check [-pi,pi]
        if (RadNormalization(angle) > RadNormalization(angle_range.first) && RadNormalization(angle) < RadNormalization(angle_range_end))
        {
            return true;
        }
        return false;
    }

    AngleRange GetAngleRangeFromPointToEdgeAtRadius(const HybridAStar::Node3D &point, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end)
    {
        return GetAngleRangeFromPointToEdgeAtRadius(ConvertNod3DToVector2f(point), radius, start, end);
    }
    // checked ,correct
    AngleRange GetAngleRangeFromPointToEdgeAtRadius(const Eigen::Vector2f &point, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end)
    {
        // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius in:";
        AngleRange out;

        // 1. check if distance from point to edge is smaller than radius?
        float distance = GetDistanceFromSegmentToPoint(start, end, point);
        if (distance >= radius)
        {
            DLOG(INFO) << "point is too far from edge!!!";
            out.second = 0;
            // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
            return out;
        }
        // 2. find intersection point for a circle and a segment
        std::vector<Eigen::Vector2f> intersection_points = GetIntersectionPointsBetweenCircleAndSegment(point, radius, start, end);
        // 3. find its angle from intersection point to point
        // 3.1 edge is fully inside polygon or outside polygon
        if (intersection_points.size() == 0)
        {
            // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
            return GetAngleRangeFromPointToSegment(start, end, point);
        }

        if (intersection_points.size() == 1)
        {
            bool is_start_inside_circle = IsPointInsideCircle(point, radius, start);
            bool is_end_inside_circle = IsPointInsideCircle(point, radius, end);
            // 3.2.1 edge is tangent to the circle, size is one and two end points are outside circle
            if (!is_start_inside_circle && !is_end_inside_circle)
            {
                out.second = 0;
                // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
                return out;
            }
            // 3.2.2 edge is partially inside circle
            if (is_start_inside_circle && !is_end_inside_circle)
            {
                float intersection_angle = GetAngle(point, intersection_points[0]);
                float start_angle = GetAngle(point, start);
                out = FindAngleRange(intersection_angle, start_angle);
            }
            if (!is_start_inside_circle && is_end_inside_circle)
            {
                float intersection_angle = GetAngle(point, intersection_points[0]);
                float start_angle = GetAngle(point, end);
                out = FindAngleRange(intersection_angle, start_angle);
            }
            // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
            return out;
        }

        // 3.3 edge intersect with circle two times
        // intersection_points size must be two.
        if (intersection_points.size() != 2)
        {
            DLOG(INFO) << "more than two intersection points found, impossible!!!";
            out.second = 0;
            // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
            return out;
        }
        // DLOG(INFO) << "intersection point is " << intersection_points[0].x() << " " << intersection_points[0].y() << " second point is " << intersection_points[1].x() << " " << intersection_points[1].y();
        out = FindAngleRange(GetAngle(point, intersection_points[0]), GetAngle(point, intersection_points[1]));
        // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
        return out;
    }
    // checked, correct
    AngleRange GetAngleRangeFromPointToPolygonAtRadius(const Eigen::Vector2f &point, const float &radius, const Polygon &polygon)
    {
        AngleRange out(-1, -1);
        // three states of polygon position:
        // 1. fully inside circle
        // 2. partially inside circle
        // 3. fully outside circle
        // for 1 and 3, radius has nothing to do with result, just use normal function
        if (IsPolygonInsideCircle(point, radius, polygon) != 0)
        {
            out = GetAngleRangeFromPointToPolygon(polygon, point);
            return out;
        };
        // loop all the edge for polygon
        AngleRangeVec angle_range_vec;
        for (uint index = 0; index < polygon.size() - 1; ++index)
        {
            if (IsEdgeInsideCircle(point, radius, polygon[index], polygon[index + 1]) != -1)
            {
                angle_range_vec.emplace_back(GetAngleRangeFromPointToEdgeAtRadius(point, radius, polygon[index], polygon[index + 1]));
            }
        }
        for (const auto &angle_range : angle_range_vec)
        {
            if (out.second == -1)
            {
                out = angle_range;
            }
            out = CombineAngleRange(out, angle_range);
        }
        return out;
    }
    // checked, correct.
    AngleRange FindAngleRange(const float &a1, const float &a2)
    {
        AngleRange out;
        float min_angle, max_angle, range_start, range;
        if (a1 > a2)
        {
            min_angle = a2;
            max_angle = a1;
        }
        else
        {
            min_angle = a1;
            max_angle = a2;
        }

        // if range greater than PI, than reverse start and end
        range = max_angle - min_angle;
        if (range > M_PI)
        {
            range_start = max_angle;
            range = min_angle + 2 * M_PI - min_angle;
        }
        else
        {
            range_start = min_angle;
            range = max_angle - min_angle;
        }
        out.first = range_start;
        out.second = range;

        return out;
    }
    // checked, correct.
    std::vector<Eigen::Vector2f> GetIntersectionPointsBetweenCircleAndSegment(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end)
    {
        // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndSegment in:";
        std::vector<Eigen::Vector2f> out;
        // three state: zero intersection, one intersection, two intersection
        // distance from center to segment larger than radius, then there must be zero intersection
        if (GetDistanceFromSegmentToPoint(start, end, center) > radius)
        {
            DLOG(INFO) << "segment is too far from circle!!!";
            // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndSegment out.";
            return out;
        }
        // zero intersection:
        //  two points are inside circle
        if ((IsPointInsideCircle(center, radius, start) && IsPointInsideCircle(center, radius, end)))
        {
            DLOG(INFO) << "No intersection for circle and segment since two ends of segment are all inside circle!!!";
            // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndSegment out.";
            return out;
        }
        // find intersection between circle and line,
        Eigen::Vector2f unit_vector = GetUnitVector(start, end);
        // DLOG(INFO) << "unit vector is " << unit_vector.x() << " " << unit_vector.y();
        std::vector<Eigen::Vector2f> intersection_vec = GetIntersectionPointsBetweenCircleAndLine(center, radius, start, unit_vector);

        // then check if intersection points are on segment
        for (const auto &points : intersection_vec)
        {
            // DLOG(INFO) << "all intersections from line is " << points.x() << " " << points.y();
            if (OnSegment(start, end, points))
            {
                out.emplace_back(points);
            }
        }
        // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndSegment out.";
        return out;
    }
    // checked, correct
    bool IsPointInsideCircle(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &point)
    {
        float distance = GetDistanceFromPointToPoint(center, point);
        if (distance > radius)
        {
            // point is outside circle
            return false;
        }
        return true;
    }
    // checked, correct
    int IsEdgeInsideCircle(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end)
    {
        bool start_inside = IsPointInsideCircle(center, radius, start);
        bool end_inside = IsPointInsideCircle(center, radius, end);
        // edge is inside circle
        if (start_inside && end_inside)
        {
            return 1;
        }
        if (!start_inside && !end_inside)
        {
            return -1;
        }
        return 0;
    }
    // checked, correct
    int IsPolygonInsideCircle(const Eigen::Vector2f &center, const float &radius, const Polygon &polygon)
    {
        // DLOG(INFO) << "IsPolygonInsideCircle in:";
        int out = 0;
        for (uint index = 0; index < polygon.size() - 1; ++index)
        {
            out += IsEdgeInsideCircle(center, radius, polygon[index], polygon[index + 1]);
            // DLOG(INFO) << "out is " << out;
        }
        if (out == 4)
        {
            // DLOG(INFO) << "IsPolygonInsideCircle out.";
            return 1;
        }
        if (out == -4)
        {
            // DLOG(INFO) << "IsPolygonInsideCircle out.";
            return -1;
        }
        // DLOG(INFO) << "IsPolygonInsideCircle out.";
        return 0;
    }
    // tested, correct.
    Eigen::Vector2f GetUnitVector(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
    {
        Eigen::Vector2f diff = end - start;

        return diff.normalized();
    }
    // checked, it`s correct.
    std::vector<Eigen::Vector2f> GetIntersectionPointsBetweenCircleAndLine(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &unit_vector)
    {
        // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndLine in:";
        std::vector<Eigen::Vector2f> out;
        // 1. check if distance from line to circle larger than radius
        float distance = GetDistanceFromPointToLine(center, start, unit_vector);
        // distance greater than radius, then no intersection
        if (distance > radius)
        {
            DLOG(INFO) << "no intersection since line is too far from circle!!";
            // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndLine out.";
            return out;
        }
        // just tangent, only one intersection
        if (abs(distance - radius) < 1e-3)
        {
            out.emplace_back(FindProjectionPoint(center, start, unit_vector));
            // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndLine out.";
            return out;
        }
        // rest case is two intersection:
        // 1. find projection point on line for center
        Eigen::Vector2f projection = FindProjectionPoint(center, start, unit_vector);
        // 2. intersection=projection point +- k* unit vector, k=sqrt(radius - distance from center to line)
        float k = sqrt(radius * radius - distance * distance);
        out.emplace_back(projection + k * unit_vector);
        out.emplace_back(projection - k * unit_vector);
        // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndLine out.";
        return out;
    }
    // checked, correct
    Eigen::Vector2f FindProjectionPoint(const Eigen::Vector2f &point, const Eigen::Vector2f &start, const Eigen::Vector2f &unit_vector)
    {
        if (IsPointOnLine(point, start, unit_vector))
        {
            return point;
        }
        Eigen::Vector2f normal_vector(unit_vector.y(), -unit_vector.x());
        float distance = GetDistanceFromPointToLine(point, start, unit_vector);
        Eigen::Vector2f potential_point_1 = point + distance * normal_vector;
        Eigen::Vector2f potential_point_2 = point - distance * normal_vector;
        if (IsPointOnLine(potential_point_1, start, unit_vector))
        {
            return potential_point_1;
        }
        return potential_point_2;
    }
    // checked, correct
    AngleRange CombineAngleRange(const AngleRange &ar1,
                                 const AngleRange &ar2)
    {
        // DLOG(INFO) << "CombineAngleRange in:";
        AngleRange out;
        // check if ar1 and ar2 are include?
        if (IsAngleRangeInclude(ar1, ar2))
        {
            out = ar1;
            // DLOG(INFO) << "CombineAngleRange out.";
            return out;
        }
        if (IsAngleRangeInclude(ar2, ar1))
        {
            out = ar2;
            // DLOG(INFO) << "CombineAngleRange out.";
            return out;
        }
        // check if ar1 and ar2 are overlap
        if (IsOverlap(ar1, ar2))
        {

            float ar1_end = ar1.first + ar1.second;

            float ar2_end = ar2.first + ar2.second;
            out.first = std::min(ar1.first, ar2.first);
            out.second = std::max(ar1_end, ar2_end) - out.first;
            // DLOG(INFO) << "CombineAngleRange out.";
            return out;
        }
        // ar1 and ar2 are not overlap and included, make start and range -1
        out.first = -1;
        out.second = -1;
        DLOG(INFO) << "CombineAngleRange out.";
        return out;
    }

} // namespace Utility
