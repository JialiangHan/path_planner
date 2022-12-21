#include "utility.h"

namespace Utility
{
    //*******************type conversion*******************

    void TypeConversion(const geometry_msgs::PoseStamped &start, HybridAStar::Node3D &node3d)
    {
        node3d.setX(start.pose.position.x);
        node3d.setY(start.pose.position.y);
        node3d.setT(tf::getYaw(start.pose.orientation));
        return;
    }
    void TypeConversion(const HybridAStar::Node3D &node3d, geometry_msgs::PoseStamped &pose)
    {
        tf::Quaternion pose_quat = tf::createQuaternionFromYaw(node3d.getT());

        pose.pose.position.x = node3d.getX();
        pose.pose.position.y = node3d.getY();

        pose.pose.orientation.x = pose_quat.x();
        pose.pose.orientation.y = pose_quat.y();
        pose.pose.orientation.z = pose_quat.z();
        pose.pose.orientation.w = pose_quat.w();
    }

    void TypeConversion(const Path3D &path3d, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        for (const auto &element : path3d)
        {
            geometry_msgs::PoseStamped pose;
            TypeConversion(element, pose);
            plan.push_back(pose);
        }
    }

    void TypeConversion(const HybridAStar::Node3D &node3d, HybridAStar::Node2D &node_2d)
    {
        node_2d.setX(node3d.getX());
        node_2d.setY(node3d.getY());
    }

    void TypeConversion(const HybridAStar::Node2D &node2d, HybridAStar::Node3D &node_3d)
    {
        node_3d.setX(node2d.getX());
        node_3d.setY(node2d.getY());
        node_3d.setT(0);
    }
    void TypeConversion(
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

    void TypeConversion(
        const std::vector<Eigen::Vector3f> &vector_3d_vec, nav_msgs::Path &path)
    {

        path.header.stamp = ros::Time::now();
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
            path.poses.emplace_back(vertex);
        }
    }
    void TypeConversion(const Eigen::Vector3f &vector_3d, Eigen::Vector2f &vector_2d)
    {
        vector_2d.x() = vector_3d.x();
        vector_2d.y() = vector_3d.y();
    }

    void TypeConversion(const HybridAStar::Node3D &node3d, Eigen::Vector3f &vector_3d)
    {
        vector_3d.x() = node3d.getX();
        vector_3d.y() = node3d.getY();
        vector_3d.z() = node3d.getT();
    }

    void TypeConversion(const HybridAStar::Node3D &node3d, Eigen::Vector2f &vector2f)
    {
        Eigen::Vector3f vector3f;
        TypeConversion(node3d, vector3f);
        TypeConversion(vector3f, vector2f);
    }
    void TypeConversion(const HybridAStar::Node2D &node2d, Eigen::Vector2f &vector2f)
    {
        vector2f.x() = node2d.getX();
        vector2f.y() = node2d.getY();
    }
    void TypeConversion(const Eigen::Vector3f &vector3d, HybridAStar::Node3D &node_3d)
    {
        node_3d.setX(vector3d.x());
        node_3d.setY(vector3d.y());
        node_3d.setT(vector3d.z());
    }

    void TypeConversion(const Eigen::Vector2f &vector2d, HybridAStar::Node3D &node3d)
    {
        node3d.setX(vector2d.x());
        node3d.setY(vector2d.y());
    }

    void TypeConversion(const Path2D &path_2d, Path3D &path_3d)
    {
        HybridAStar::Node3D node_3d;
        for (const auto &element : path_2d)
        {
            TypeConversion(element, node_3d);
            path_3d.emplace_back(node_3d);
        }
    }

    void TypeConversion(const std::vector<std::shared_ptr<HybridAStar::Node3D>> &smart_ptr_vec, std::vector<HybridAStar::Node3D *> &ptr_vec)
    {
        // DLOG(INFO) << "in typeconversion. ptr_vec size is " << ptr_vec.size();
        DLOG_IF(WARNING, smart_ptr_vec.size() <= 0) << "smart_ptr_vec size is ZERO!!!";
        // ptr_vec.clear();
        HybridAStar::Node3D *ptr;
        for (const auto &element : smart_ptr_vec)
        {
            ptr = element.get();
            ptr_vec.emplace_back(ptr);
            // LOG(INFO) << "inserting " << ptr->getX() << " " << ptr->getY() << " " << ConvertRadToDeg(ptr->getT()) << "to ptr_vec.";
        }
        // DLOG(INFO) << "ptr_vec size is " << ptr_vec.size();
    }

    void TypeConversion(costmap_2d::Costmap2D *_costmap, std::string frame_id, nav_msgs::OccupancyGrid::Ptr &map)
    {
        double resolution = _costmap->getResolution();

        map->header.frame_id = frame_id;
        map->header.stamp = ros::Time::now();
        map->info.resolution = resolution;

        map->info.width = _costmap->getSizeInCellsX();
        map->info.height = _costmap->getSizeInCellsY();

        double wx, wy;
        _costmap->mapToWorld(0, 0, wx, wy);
        map->info.origin.position.x = wx - resolution / 2;
        map->info.origin.position.y = wy - resolution / 2;
        map->info.origin.position.z = 0.0;
        map->info.origin.orientation.w = 1.0;

        map->data.resize(map->info.width * map->info.height);
        char *cost_translation_table_;
        cost_translation_table_ = new char[256];

        // special values:
        cost_translation_table_[0] = 0;     // NO obstacle
        cost_translation_table_[253] = 99;  // INSCRIBED obstacle
        cost_translation_table_[254] = 100; // LETHAL obstacle
        cost_translation_table_[255] = -1;  // UNKNOWN

        // regular cost values scale the range 1 to 252 (inclusive) to fit
        // into 1 to 98 (inclusive).
        for (int i = 1; i < 253; i++)
        {
            cost_translation_table_[i] = char(1 + (97 * (i - 1)) / 251);
        }
        unsigned char *data = _costmap->getCharMap();
        for (unsigned int i = 0; i < map->data.size(); i++)
        {
            map->data[i] = cost_translation_table_[data[i]];
        }
        delete[] cost_translation_table_;
    }
    //**********************computational geometry****************

    bool OnSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p3,
                   const Eigen::Vector2f &p2)
    {
        // special case: all three points have same x or y coordinate
        // same x coordinate
        if (abs(p2.x() - p1.x()) < 1e-3 && abs(p2.x() - p3.x()) < 1e-3 &&
            p2.y() <= std::max(p1.y(), p3.y()) &&
            p2.y() >= std::min(p1.y(), p3.y()))
        {
            // DLOG(INFO) << "all three points p1 " << p1.x() << " " << p1.y() << " p3 " << p3.x() << " " << p3.y() << " p2 " << p2.x() << " " << p2.y() << " have same x coordinate.";
            return true;
        }
        // same y coordinate
        if (p2.x() <= std::max(p1.x(), p3.x()) &&
            p2.x() >= std::min(p1.x(), p3.x()) && abs(p2.y() - p1.y()) < 1e-3 && abs(p2.y() - p3.y()) < 1e-3)
        {
            // DLOG(INFO) << "all three points p1 " << p1.x() << " " << p1.y() << " p3 " << p3.x() << " " << p3.y() << " p2 " << p2.x() << " " << p2.y() << " have same y coordinate.";
            return true;
        }
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
        // DLOG(INFO) << "IsPointOnLine in:";
        Eigen::Vector2f diff = point - start;
        if (abs(diff.x()) < 1e-3 && abs(diff.y()) < 1e-3)
        {
            // DLOG(INFO) << "point " << point.x() << " " << point.y() << " is on the line start at " << start.x() << " " << start.y() << " unit vector is " << unit_vector.x() << " " << unit_vector.y();
            return true;
        }
        float angle = acos(diff.dot(unit_vector) / diff.norm() / unit_vector.norm());
        // DLOG(INFO) << "angle is " << angle;
        // DLOG(INFO) << "angle smaller than 1e-3: " << (angle < 1e-3);
        // DLOG(INFO) << "(angle -3.14) smaller than 1e-3: " << (abs(angle - 3.14) < 1e-3);
        if (abs(angle) < 1e-3 || (abs(angle - M_PI) < 1e-3))
        {
            // DLOG(INFO) << "point " << point.x() << " " << point.y() << " is on the line start at " << start.x() << " " << start.y() << " unit vector is " << unit_vector.x() << " " << unit_vector.y();
            return true;
        }
        // DLOG(INFO) << "point " << point.x() << " " << point.y() << " is not on the line start at " << start.x() << " " << start.y() << " unit vector is " << unit_vector.x() << " " << unit_vector.y();
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
        Eigen::Vector2f point_2d;
        TypeConversion(point, point_2d);
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
        Eigen::Vector2f vector2d_1;
        TypeConversion(vector_3d, vector2d_1);
        return GetDistanceFromPointToPoint(vector2d_1,
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
    // TODO: bug exist
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
        DLOG(INFO) << "angle is " << ConvertRadToDeg(angle) << " sign is " << ConvertRadToDeg(sign);
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
        float end_angle = GetAngleBetweenTwoVector(point, horizontal, point, end);
        // DLOG(INFO) << "start angle is " << ConvertRadToDeg(start_angle) << " "                   << " end_angle is " << ConvertRadToDeg(end_angle);
        std::pair<float, float> temp = AngleNormalization(start_angle, end_angle);
        std::pair<float, float> angle_pair = CompareAngle(temp.first, temp.second);
        // DLOG(INFO) << "angle pair first is " << ConvertRadToDeg(angle_pair.first) << " angle pair second is " << ConvertRadToDeg(angle_pair.second);
        std::vector<AngleRange> out = FindAngleRange(angle_pair.first, angle_pair.second);

        // DLOG(INFO) << "range start is " << ConvertRadToDeg(out[0].first) << " " << " range is " << ConvertRadToDeg(out[0].second);
        // DLOG(INFO) << "GetAngleRangeFromPointToSegment out.";
        if (out.size() > 1)
        {
            DLOG(WARNING) << "WARNING: Something Wrong!!!";
        }
        return out[0];
    }

    // checked
    AngleRange GetAngleRangeFromPointToPolygon(const Polygon &polygon,
                                               const Eigen::Vector2f &point)
    {
        // DLOG(INFO) << "GetAngleRangeFromPointToPolygon in:";
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
        // DLOG(INFO) << "GetAngleRangeFromPointToPolygon out.";
        return std::make_pair(starting_angle, abs(range));
    }

    AngleRange GetAngleRangeFromPointToPolygon(const Polygon &polygon,
                                               const Eigen::Vector2f &point, const float &radius)
    {
        // DLOG(INFO) << "GetAngleRangeFromPointToPolygon in:";
        AngleRange out;

        // 1. check if polygon is inside this circle
        float distance = Utility::GetDistanceFromPolygonToPoint(polygon, point);
        // DLOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << "  and distance to point: " << point.x() << " " << point.y() << " is " << distance << " range is " << radius;
        // if distance from polygon to point too small,make range is zero
        if (distance < 1e-3)
        {
            out.second = 0;
            // DLOG(INFO) << "too close to polygon!!";
            return out;
        }
        if (distance > radius)
        {
            // DLOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << "  and distance to point: " << point.x() << " " << point.y() << " is " << distance << " range is " << radius;
            // DLOG(INFO) << "polygon is outside the range!!";
            return out;
        }

        // 2. find angle which has distance equal to radius from point to polygon
        out = GetAngleRangeFromPointToPolygonAtRadius(point, radius, polygon);
        DLOG_IF(INFO, out.first == -1) << "angle range start from " << ConvertRadToDeg(out.first) << " range is " << ConvertRadToDeg(out.second);
        // DLOG(INFO) << "angle range start from " << ConvertRadToDeg(out.first) << " range is " << ConvertRadToDeg(out.second);
        // DLOG(INFO) << "GetAngleRangeFromPointToPolygon out.";
        return out;
    }

    std::vector<Eigen::Vector2f>
    FindVisibleVertexFromNode(const Polygon &polygon,
                              const Eigen::Vector2f &point)
    {
        std::vector<Eigen::Vector2f> out;
        // check if point is polygon, return empty vector if on polygon
        if (IsOnPolygon(polygon, point))
        {
            return out;
        }
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
        // DLOG(INFO) << "IsOverlap in:";
        float angle_range_1_start = angle_range_1.first;
        float angle_range_1_end = GetAngleRangeEnd(angle_range_1);
        float angle_range_2_start = angle_range_2.first;
        float angle_range_2_end = GetAngleRangeEnd(angle_range_2);
        // DLOG(INFO) << "ar1 start is " << ConvertRadToDeg(angle_range_1_start) << " ar1 end is " << ConvertRadToDeg(angle_range_1_end) << " ar2 start is " << ConvertRadToDeg(angle_range_2_start) << " ar2 end is " << ConvertRadToDeg(angle_range_2_end);

        // include
        if (IsAngleRangeInclude(angle_range_1, angle_range_2) || IsAngleRangeInclude(angle_range_2, angle_range_1))
        {
            // DLOG(INFO) << "one ar include another.";
            return false;
        }
        // overlap
        // one angle is in another angle range
        else if ((IsAngleRangeInclude(angle_range_1, angle_range_2_start) && !IsAngleRangeInclude(angle_range_1, angle_range_2_end)) || (!IsAngleRangeInclude(angle_range_1, angle_range_2_start) && IsAngleRangeInclude(angle_range_1, angle_range_2_end)) || (IsAngleRangeInclude(angle_range_2, angle_range_1_start) && !IsAngleRangeInclude(angle_range_2, angle_range_1_end)) || (!IsAngleRangeInclude(angle_range_2, angle_range_1_start) && IsAngleRangeInclude(angle_range_2, angle_range_1_end)))
        {
            // DLOG(INFO) << "yes, two angle ranges are overlap.";
            return true;
        }
        // DLOG(INFO) << "no, two angle ranges are not overlapped.";
        return false;
    }
    // checked
    bool IsAngleRangeInclude(const AngleRange &angle_range_1,
                             const AngleRange &angle_range_2)
    {
        // DLOG(INFO) << "IsAngleRangeInclude in.";
        // DLOG(INFO) << "ar1 start is " << Utility::ConvertRadToDeg(angle_range_1.first) << " end is " << Utility::ConvertRadToDeg(GetAngleRangeEnd(angle_range_1)) << " ar2 start is " << Utility::ConvertRadToDeg(angle_range_2.first) << " end is " << Utility::ConvertRadToDeg(GetAngleRangeEnd(angle_range_2));
        // special case: if ar2 range is zero
        if (angle_range_2.second == 0)
        {
            // DLOG(INFO) << "ar2 range is zero!!";
            return IsAngleRangeInclude(angle_range_1, angle_range_2.first);
        }
        // DLOG(INFO) << "IsEqual(angle_range_1.first, GetAngleRangeEnd(angle_range_2) " << IsEqual(angle_range_1.first, GetAngleRangeEnd(angle_range_2)) << " IsEqual(GetAngleRangeEnd(angle_range_1), angle_range_2.second) " << IsEqual(GetAngleRangeEnd(angle_range_1), angle_range_2.first);
        // if ar1 start ==ar2 end and ar1 end =ar2 start
        if (IsEqual(angle_range_1.first, GetAngleRangeEnd(angle_range_2)) && IsEqual(GetAngleRangeEnd(angle_range_1), angle_range_2.first))
        {
            // DLOG(INFO) << "ar1 start == ar2 end and ar1 end =ar2 start!";
            return false;
        }
        // all input should be in rad.
        float angle_range_2_start = angle_range_2.first;
        float angle_range_2_end = GetAngleRangeEnd(angle_range_2);
        // if ar2 start and end are both included by ar1, then ar1 include ar2
        if (IsAngleRangeInclude(angle_range_1, angle_range_2_start) && IsAngleRangeInclude(angle_range_1, angle_range_2_end) && (angle_range_1.second > angle_range_2.second))
        {
            // DLOG(INFO) << "ar1 include ar2.";
            return true;
        }
        // DLOG(INFO) << "ar1 doesn`t include ar2.";
        return false;
    }
    // checked
    AngleRange MinusAngleRangeOverlap(const AngleRange &angle_range_1,
                                      const AngleRange &angle_range_2)
    {
        // DLOG(INFO) << "MinusAngleRangeOverlap in:";
        AngleRange out;
        if (!IsOverlap(angle_range_1, angle_range_2))
        {
            DLOG(INFO) << "angle range 1 and angle range 2 is not overlap!";
            return out;
        }
        float angle_range_1_start = angle_range_1.first;
        float angle_range_1_end = GetAngleRangeEnd(angle_range_1);
        float angle_range_2_start = angle_range_2.first;
        float angle_range_2_end = GetAngleRangeEnd(angle_range_2);
        // DLOG(INFO) << "ar1 start is " << Utility::ConvertRadToDeg(angle_range_1_start) << " end is " << Utility::ConvertRadToDeg(angle_range_1_end) << " ar2 start is " << Utility::ConvertRadToDeg(angle_range_2_start) << " end is " << Utility::ConvertRadToDeg(angle_range_2_end);
        // special case, ar1 and ar2 just have an angle boundary in common. 1. find in common angle boundary. 2. move this boundary inside ar1
        if (IsEqual(angle_range_1_start, angle_range_2_end) &&
            !IsEqual(angle_range_1_end, angle_range_2_end))
        {
            out.first = angle_range_1_start + ConvertDegToRad(0.1);
            out.second = angle_range_1.second - ConvertDegToRad(0.1);
            // DLOG(INFO) << "ar1 and ar2 share ar1 start!!!";
            return out;
        }
        if (IsEqual(angle_range_1_end, angle_range_2_start) &&
            !IsEqual(angle_range_1_start, angle_range_2_end))
        {
            out.first = angle_range_1_start;
            out.second = angle_range_1.second - ConvertDegToRad(0.1);
            // DLOG(INFO) << "ar1 and ar2 share ar1 end!!!";
            return out;
        }

        // case 1, start of first<start of second<end of first<end of second
        if (Angle1RightAngle2(angle_range_1_start, angle_range_2_start) && Angle1RightAngle2(angle_range_2_start, angle_range_1_end) && Angle1RightAngle2(angle_range_1_end, angle_range_2_end))
        {
            out.first = angle_range_1_start;
            out.second = GetAngleDistance(angle_range_1_start, angle_range_2_start);
        }
        // case 2, start of second<start of first<end of second<end of first
        if (Angle1RightAngle2(angle_range_2_start, angle_range_1_start) && Angle1RightAngle2(angle_range_1_start, angle_range_2_end) && Angle1RightAngle2(angle_range_2_end, angle_range_1_end))
        {
            out.first = angle_range_2_end;
            out.second = GetAngleDistance(angle_range_2_end, angle_range_1_end);
        }
        out.first = RadToZeroTo2P(out.first);
        // DLOG(INFO) << "out start from " << ConvertRadToDeg(out.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(out));
        // DLOG(INFO) << "MinusAngleRangeOverlap in:";
        return out;
    }
    // checked
    std::vector<AngleRange> MinusAngleRange(const AngleRange &ar1, const AngleRange &ar2)
    {
        // DLOG(INFO) << "MinusAngleRange in:";
        // DLOG(INFO) << "ar1 start at " << ConvertRadToDeg(ar1.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(ar1)) << " ar2 start at " << ConvertRadToDeg(ar2.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(ar2));
        std::vector<AngleRange> out;
        if (IsEqual(ar1, ar2))
        {
            out.emplace_back(FindAngleRange(ar1.first, ar1.first)[0]);
            return out;
        }
        if (IsOverlap(ar1, ar2))
        {
            // DLOG(INFO) << "ar1 and ar2 are overlapped.";
            out.emplace_back(MinusAngleRangeOverlap(ar1, ar2));
        }
        if (IsAngleRangeInclude(ar1, ar2))
        {
            // DLOG(INFO) << "ar1 include a2!";
            float ar1_start = ar1.first;
            float ar1_end = GetAngleRangeEnd(ar1);
            float ar2_start = ar2.first;
            float ar2_end = GetAngleRangeEnd(ar2);
            if (!IsEqual(ar1_start, ar2_start))
            {
                std::pair<float, float> angle_pair = CompareAngle(ar1_start, ar2_start);
                std::vector<AngleRange> temp = FindAngleRange(angle_pair.first, angle_pair.second);
                for (const auto item : temp)
                {
                    out.emplace_back(item);
                }
            }
            if (!IsEqual(ar2_end, ar1_end))
            {
                std::pair<float, float> angle_pair = CompareAngle(ar2_end, ar1_end);
                std::vector<AngleRange> temp = (FindAngleRange(angle_pair.first, angle_pair.second));
                for (const auto item : temp)
                {
                    out.emplace_back(item);
                }
            }
        }
        // for (const auto &pair : out)
        // {
        //     DLOG(INFO) << "angle range start from " << ConvertRadToDeg(pair.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(pair)) << " range is " << ConvertRadToDeg(pair.second);
        // }
        return out;
    }

    float CalculateCurvature(const Eigen::Vector2f &pre, const Eigen::Vector2f &current, const Eigen::Vector2f &succ)
    {
        float curvature = 0;
        // get three points from path

        if (pre == current || current == succ)
        {
            DLOG(WARNING) << "WARNING: In CalculateCurvature: some points are equal, skip these points for curvature calculation!!";
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

    Polygon CombinePolygon(const Polygon &polygon1, const Polygon &polygon2)
    {
        Polygon out;
        if (!IsPolygonInNeighbor(polygon1, polygon2))
        {
            return out;
        }
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
        // if point on polygon, then return 0
        if (IsOnPolygon(polygon, point))
        {
            return 0;
        }

        if (!Utility::IsSegmentIntersectWithPolygon(polygon, segment.GetStart(), segment.GetEnd()))
        {
            // DLOG(INFO) << "NO intersect with obstacles!!";
            // DLOG(INFO) << "GetDistanceFromPolygonToPointAtAngle out.";
            // DLOG_IF(INFO, out == 0) << "distance is " << out;
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
                    // DLOG_IF(INFO, out == 0) << "distance is " << out << " point is " << point.x() << " " << point.y() << " segment on polygon start from " << polygon[index].x() << " " << polygon[index].y() << " end at " << polygon[index + 1].x() << " " << polygon[index + 1].y();
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
        Eigen::Vector2f start_vec, goal_vec;
        TypeConversion(start, start_vec);
        TypeConversion(goal, goal_vec);
        return GetDistanceFromPointToPoint(start_vec, goal_vec);
    }
    float GetDistance(const HybridAStar::Node3D &start,
                      const HybridAStar::Node3D &goal)
    {
        Eigen::Vector2f start_vec, goal_vec;
        TypeConversion(start, start_vec);
        TypeConversion(goal, goal_vec);
        return GetDistanceFromPointToPoint(start_vec, goal_vec);
    }
    bool IsCloseEnough(const HybridAStar::Node3D &start,
                       const HybridAStar::Node3D &goal, const float &distance_range,
                       const float &angle_range, bool consider_orientation)
    {
        float distance = GetDistance(start, goal);
        if (distance < distance_range)
        {
            if (consider_orientation)
            {
                float angle_diff = RadNormalization(start.getT() - goal.getT());
                if (abs(angle_diff) <= angle_range)
                {
                    DLOG(INFO)
                        << "two node distance and orientation are close enough, return true";
                    DLOG(INFO) << "current node is " << start.getX() << " " << start.getY() << " " << Utility::ConvertRadToDeg(start.getT());
                    DLOG(INFO) << "goal is " << goal.getX() << " " << goal.getY() << " " << Utility::ConvertRadToDeg(goal.getT());
                    DLOG(INFO) << "angle diff is " << Utility::ConvertRadToDeg(angle_diff) << " angle range is " << Utility::ConvertRadToDeg(angle_range);
                    return true;
                }
                else
                {
                    DLOG(INFO) << "two node distance is close enough but orientation is too far is " << Utility::ConvertRadToDeg(angle_diff);
                    return false;
                }
            }
            else
            {
                return true;
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
        if (t >= -180 && t <= 360)
        {
            return t;
        }
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
        if (rad >= -M_PI && rad <= M_PI)
        {
            return rad;
        }
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
        if (abs(out) < 1e-6)
        {
            out = 0;
        }
        return out;
    }

    float DegToZeroTo2P(const float &deg)
    {
        if (deg >= 0 && deg <= 360)
        {
            return deg;
        }
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

        if (rad >= 0 && rad < 2.f * M_PI)
        {
            return rad;
        }
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
        // DLOG(INFO) << "start node is " << start.getX() << " " << start.getY() << " goal is " << goal.getX() << " " << goal.getY();
        Eigen::Vector2f start_2d, goal_2d;
        TypeConversion(start, start_2d);
        TypeConversion(goal, goal_2d);
        return GetAngle(start_2d, goal_2d);
    }
    float GetAngle(const HybridAStar::Node2D &start,
                   const HybridAStar::Node2D &goal)
    {
        // DLOG(INFO) << "start node is " << start.getX() << " " << start.getY() << " goal is " << goal.getX() << " " << goal.getY();
        Eigen::Vector2f start_2d, goal_2d;
        TypeConversion(start, start_2d);
        TypeConversion(goal, goal_2d);
        return GetAngle(start_2d, goal_2d);
    }
    // checked, correct
    float GetAngle(const Eigen::Vector2f &start, const Eigen::Vector2f &goal)
    {
        Eigen::Vector2f diff;
        diff = goal - start;
        float angle = RadToZeroTo2P(atan2(diff.y(), diff.x()));
        // make sure angle is in [0,2pi]
        // DLOG(INFO) << "start node is " << start.x() << " " << start.y() << " goal is " << goal.x() << " " << goal.y() << " angle is " << ConvertRadToDeg(angle) << " atan2 is " << ConvertRadToDeg(atan2(diff.y(), diff.x()));
        return angle;
    }

    bool IsAngleRangeInclude(const AngleRange &angle_range, const float &angle)
    {
        // DLOG(INFO) << "IsAngleRangeInclude in:";
        // if angle is in between angle range start and angle range end, then it`s include
        float angle_range_end = GetAngleRangeEnd(angle_range);
        // DLOG(INFO) << "angle range start " << ConvertRadToDeg(angle_range.first) << " angle range end " << ConvertRadToDeg(angle_range_end) << " angle is " << ConvertRadToDeg(angle);
        if (IsEqual(angle, angle_range.first))
        {
            // DLOG(INFO) << "angle is inside angle range.";
            return true;
        }
        if (IsEqual(angle, angle_range_end))
        {
            // DLOG(INFO) << "angle is inside angle range.";
            return true;
        }
        // angle is inside angle range only if distance from angle to ar start is smaller than range
        // float distance_to_start = GetAngleDistance(angle, angle_range.first);
        // DLOG(INFO) << "distance to start is " << ConvertRadToDeg(GetAngleDistance(angle, angle_range.first));
        // DLOG(INFO) << "distance to end is " << ConvertRadToDeg(GetAngleDistance(angle, angle_range_end));
        if ((RadToZeroTo2P(angle) >= RadToZeroTo2P(angle_range.first)) && (RadToZeroTo2P(angle) <= RadToZeroTo2P(angle_range_end)))
        {
            // DLOG(INFO) << "angle is inside angle range.";
            return true;
        }
        if ((RadNormalization(angle) >= RadNormalization(angle_range.first)) && (RadNormalization(angle) <= RadNormalization(angle_range_end)))
        {
            // DLOG(INFO) << "angle is inside angle range.";
            return true;
        }
        // DLOG(INFO) << "angle is not inside angle range.";
        return false;
    }

    AngleRange GetAngleRangeFromPointToEdgeAtRadius(const HybridAStar::Node3D &point, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end)
    {
        Eigen::Vector2f point_vec;
        TypeConversion(point, point_vec);
        return GetAngleRangeFromPointToEdgeAtRadius(point_vec, radius, start, end);
    }
    // checked ,correct
    AngleRange GetAngleRangeFromPointToEdgeAtRadius(const Eigen::Vector2f &point, const float &radius, const Eigen::Vector2f &start, const Eigen::Vector2f &end)
    {
        // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius in:";
        AngleRange out;
        // DLOG(INFO) << "edge start from " << start.x() << " " << start.y() << " end " << end.x() << " " << end.y() << ". circle: center is " << point.x() << " " << point.y() << " radius is " << radius;
        // 1. check if distance from point to edge is smaller than radius?
        float distance = GetDistanceFromSegmentToPoint(start, end, point);
        if (distance > radius)
        {
            // DLOG(INFO) << "point is too far from edge!!!";
            // DLOG(INFO) << "distance from point: " << point.x() << " " << point.y() << " edge start from " << start.x() << " " << start.y() << " end " << end.x() << " " << end.y() << " is " << distance << " radius is " << radius;
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
            // DLOG(INFO) << "edge start from " << start.x() << " " << start.y() << " end " << end.x() << " " << end.y() << " is fully inside circle: center is " << point.x() << " " << point.y() << " radius is " << radius;
            out = GetAngleRangeFromPointToSegment(start, end, point);
            // DLOG(INFO) << "out start from " << ConvertRadToDeg(out.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(out));
            // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
            return out;
        }

        if (intersection_points.size() == 1)
        {

            bool is_start_inside_circle = IsPointInsideCircle(point, radius, start);
            bool is_end_inside_circle = IsPointInsideCircle(point, radius, end);
            // DLOG(INFO) << "edge start from " << start.x() << " " << start.y() << " end " << end.x() << " " << end.y() << " has one intersection " << intersection_points[0].x() << " " << intersection_points[0].y() << " with circle: center is " << point.x() << " " << point.y() << " radius is " << radius;
            // 3.2.1 edge is tangent to the circle, size is one and two end points are outside circle
            if (!is_start_inside_circle && !is_end_inside_circle)
            {
                out.second = 0;
                // DLOG(INFO) << "edge is tangent to the circle.";
                return out;
            }
            // 3.2.2 edge is partially inside circle
            float intersection_angle = GetAngle(point, intersection_points[0]);
            float start_angle = 0;
            if (is_start_inside_circle && !is_end_inside_circle)
            {
                start_angle = GetAngle(point, start);
            }
            if (!is_start_inside_circle && is_end_inside_circle)
            {
                start_angle = GetAngle(point, end);
            }

            std::pair<float, float> angle_pair = CompareAngle(intersection_angle, start_angle);
            std::vector<AngleRange> temp = FindAngleRange(angle_pair.first, angle_pair.second);
            out = temp[0];
            // DLOG(INFO) << "out start from " << ConvertRadToDeg(out.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(out));
            return out;
        }
        // 3.3 edge intersect with circle two times
        // intersection_points size must be two.
        if (intersection_points.size() != 2)
        {
            DLOG(WARNING) << "WARNING: more than two intersection points found, impossible!!!";
            out.second = 0;
            // DLOG(INFO) << "GetAngleRangeFromPointToEdgeAtRadius out.";
            return out;
        }
        // DLOG(INFO) << "intersection point is " << intersection_points[0].x() << " " << intersection_points[0].y() << " second point is " << intersection_points[1].x() << " " << intersection_points[1].y();
        float first_angle = GetAngle(point, intersection_points[0]);
        float second_angle = GetAngle(point, intersection_points[1]);

        std::pair<float, float> angle_pair = CompareAngle(first_angle, second_angle);
        std::vector<AngleRange> temp = FindAngleRange(angle_pair.first, angle_pair.second);
        DLOG_IF(WARNING, temp.size() > 1) << "WARNING: angle range size greater than one!!!";
        out = temp[0];
        // DLOG(INFO) << "out start from " << ConvertRadToDeg(out.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(out));
        return out;
    }
    // checked, correct
    AngleRange GetAngleRangeFromPointToPolygonAtRadius(const Eigen::Vector2f &point, const float &radius, const Polygon &polygon)
    {
        // DLOG(INFO) << "GetAngleRangeFromPointToPolygonAtRadius in:";
        AngleRange out(-1, -1);
        // DLOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << " current point " << point.x() << " " << point.y();
        // three states of polygon position:
        // 1. fully inside circle
        // 2. partially inside circle
        // 3. fully outside circle
        // for 1 and 3, radius has nothing to do with result, just use normal function
        if (IsPolygonInsideCircle(point, radius, polygon) != 0)
        {
            out = GetAngleRangeFromPointToPolygon(polygon, point);
            // DLOG(INFO) << "GetAngleRangeFromPointToPolygonAtRadius out.";
            return out;
        };
        // loop all the edge for polygon
        AngleRangeVec angle_range_vec;
        // find edges facing  point
        std::vector<Edge> facing_edges_vec = GetPolygonEdgesFacingPoint(polygon, point);
        // DLOG(INFO) << "size of facing edges vec is " << facing_edges_vec.size();
        // find their corrsponding angle range
        for (const auto &edge : facing_edges_vec)
        {
            angle_range_vec.emplace_back(GetAngleRangeFromPointToEdgeAtRadius(point, radius, edge.first, edge.second));
        }
        // DLOG(INFO) << "size of angle_range_vec is " << angle_range_vec.size();
        for (const auto &angle_range : angle_range_vec)
        {
            // DLOG(INFO) << "before combination angle range start from " << ConvertRadToDeg(angle_range.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(angle_range)) << " out start from " << ConvertRadToDeg(out.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(out));

            if (out.second == -1)
            {
                out = angle_range;
            }
            if (!IsEqual(out, angle_range))
            {
                out = CombineAngleRange(out, angle_range);
                // DLOG(INFO) << "after combination out start from " << ConvertRadToDeg(out.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(out));
            }
        }
        DLOG_IF(INFO, out.first == -1) << "out start from " << ConvertRadToDeg(out.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(out));
        // DLOG(INFO) << "GetAngleRangeFromPointToPolygonAtRadius out.";
        return out;
    }

    std::vector<AngleRange> FindAngleRange(const float &a1, const float &a2)
    {
        std::vector<AngleRange> out;
        // DLOG(INFO) << "two angle is " << ConvertRadToDeg(a1) << " " << ConvertRadToDeg(a2);
        float temp1, temp2, range;
        temp1 = RadToZeroTo2P(a1);
        temp2 = RadToZeroTo2P(a2);
        // check if two angles are equal:
        if (IsEqual(temp1, temp2))
        {
            out.emplace_back(AngleRange(temp1, 0));
            // DLOG(INFO) << "two angles are equal final range start from " << ConvertRadToDeg(out[0].first) << " range is " << ConvertRadToDeg(out[0].second);
            return out;
        }

        // if range greater than PI, than reverse start and end
        range = temp2 - temp1;
        // DLOG(INFO) << "range between a2 and a1 is " << ConvertRadToDeg(range);
        out.emplace_back(AngleRange(temp1, range));
        if (range > M_PI)
        {
            out[0].second = M_PI;
            out.emplace_back(AngleRange(RadToZeroTo2P(temp1 + M_PI), range - M_PI));
        }

        // DLOG(INFO) << " final range start from " << ConvertRadToDeg(out[0].first) << " range is " << ConvertRadToDeg(out[0].second);
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
            // DLOG(INFO) << "No intersection for circle and segment since two ends of segment are all inside circle!!!";
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
        // for (const auto &point : out)
        // {
        // DLOG(INFO) << "intersection point is " << point.x() << " " << point.y();
        // }
        // DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndSegment out.";
        return out;
    }
    // checked, correct
    bool IsPointInsideCircle(const Eigen::Vector2f &center, const float &radius, const Eigen::Vector2f &point)
    {
        float distance = GetDistanceFromPointToPoint(center, point);
        if (distance >= radius)
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
        float distance = GetDistanceFromSegmentToPoint(start, end, center);

        // edge is inside circle
        if (start_inside && end_inside)
        {
            return 1;
        }
        // fully outside
        if (!start_inside && !end_inside && (distance > radius))
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
        // DLOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << " current point " << center.x() << " " << center.y() << " radius is " << radius;
        for (uint index = 0; index < polygon.size() - 1; ++index)
        {
            out += IsEdgeInsideCircle(center, radius, polygon[index], polygon[index + 1]);
            // DLOG(INFO) << "out is " << out;
        }
        if (out == 4)
        {
            // DLOG(INFO) << "polygon is fully inside circle.";
            return 1;
        }
        if (out == -4)
        {
            // DLOG(INFO) << "polygon is fully outside circle.";
            return -1;
        }
        // DLOG(INFO) << "polygon is partially inside circle.";
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
            DLOG(INFO) << "GetIntersectionPointsBetweenCircleAndLine out.";
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
        // DLOG(INFO) << "FindProjectionPoint in:";
        if (IsPointOnLine(point, start, unit_vector))
        {
            // DLOG(INFO) << "Point: " << point.x() << " " << point.y() << " is on the line start from " << start.x() << " " << start.y() << " unit vector is " << unit_vector.x() << " " << unit_vector.y();
            return point;
        }
        Eigen::Vector2f normal_vector(unit_vector.y(), -unit_vector.x());
        float distance = GetDistanceFromPointToLine(point, start, unit_vector);
        Eigen::Vector2f potential_point_1 = point + distance * normal_vector;
        Eigen::Vector2f potential_point_2 = point - distance * normal_vector;
        if (IsPointOnLine(potential_point_1, start, unit_vector))
        {
            // DLOG(INFO) << "Point: " << point.x() << " " << point.y() << " has projection point: " << potential_point_1.x() << " " << potential_point_1.y() << " on the line start from " << start.x() << " " << start.y() << " unit vector is " << unit_vector.x() << " " << unit_vector.y();
            return potential_point_1;
        }
        // DLOG(INFO) << "Point: " << point.x() << " " << point.y() << " has projection point: " << potential_point_2.x() << " " << potential_point_2.y() << " on the line start from " << start.x() << " " << start.y() << " unit vector is " << unit_vector.x() << " " << unit_vector.y();
        return potential_point_2;
    }
    // checked, correct
    AngleRange CombineAngleRange(const AngleRange &ar1,
                                 const AngleRange &ar2)
    {
        // DLOG(INFO) << "CombineAngleRange in:";
        // DLOG(INFO) << "ar1 start " << ConvertRadToDeg(ar1.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(ar1)) << " ar2 start " << ConvertRadToDeg(ar2.first) << " end is " << ConvertRadToDeg(GetAngleRangeEnd(ar2));
        AngleRange out(-1, -1);
        if (IsEqual(ar1, out) && !IsEqual(ar2, out))
        {
            out = ar2;
            // DLOG(INFO) << "CombineAngleRange out.";
            return out;
        }
        if (!IsEqual(ar1, out) && IsEqual(ar2, out))
        {
            out = ar1;
            // DLOG(INFO) << "CombineAngleRange out.";
            return out;
        }
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
            float ar1_end = GetAngleRangeEnd(ar1);
            float ar2_end = GetAngleRangeEnd(ar2);
            if (IsAngleRangeInclude(ar1, ar2.first) && IsAngleRangeInclude(ar2, ar1_end))
            {
                out.first = ar1.first;
                out.second = ar2_end - ar1.first;
                if (out.second < 0)
                {
                    out.second = out.second + 2 * M_PI;
                }
                // DLOG(INFO) << "CombineAngleRange out.";
                return out;
            }
            if (IsAngleRangeInclude(ar1, ar2_end) && IsAngleRangeInclude(ar2, ar1.first))
            {
                out.first = ar2.first;
                out.second = ar1_end - ar2.first;
                if (out.second < 0)
                {
                    out.second = out.second + 2 * M_PI;
                }
            }
            // DLOG(INFO) << "CombineAngleRange out.";
            return out;
        }
        // check if ar1 and ar2 share boundary
        if (ShareBoundary(ar1, ar2))
        {
            if (IsEqual(ar1.first, GetAngleRangeEnd(ar2)))
            {
                out.first = ar2.first;
                out.second = ar2.second + ar1.second;
            }
            if (IsEqual(ar2.first, GetAngleRangeEnd(ar1)))
            {
                out.first = ar1.first;
                out.second = ar2.second + ar1.second;
            }
            // set start angle to zero if range is 2*PI
            if (abs(out.second - 2 * M_PI) < 0.0001)
            {
                out.first = 0;
            }
            // DLOG(INFO) << "CombineAngleRange out.";
            return out;
        }
        // ar1 and ar2 are not overlap and included, make start and range -1

        // DLOG(INFO) << "two angle range have nothing in common, can`t combine them!!";
        return out;
    }
    // checked, correct
    AngleRange FindCommonAngleRange(const AngleRange &ar1,
                                    const AngleRange &ar2)
    {
        AngleRange out;
        if (IsAngleRangeInclude(ar1, ar2))
        {
            out = ar2;
            // DLOG(INFO) << "ar1: start from " << Utility::ConvertRadToDeg(ar1.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(ar1)) << " include ar2: start from " << Utility::ConvertRadToDeg(ar2.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(ar2)) << " . their common angle range start from " << Utility::ConvertRadToDeg(out.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(out));
            return out;
        }
        if (IsAngleRangeInclude(ar2, ar1))
        {
            out = ar1;
            // DLOG(INFO) << "ar1: start from " << Utility::ConvertRadToDeg(ar1.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(ar1)) << " is included by ar2: start from " << Utility::ConvertRadToDeg(ar2.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(ar2)) << " . their common angle range start from " << Utility::ConvertRadToDeg(out.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(out));
            return out;
        }
        // if ar1 is equal is to ar2 than return ar1
        if (IsEqual(ar1, ar2))
        {
            return ar1;
        }
        if (IsOverlap(ar1, ar2))
        {
            float ar1_end = GetAngleRangeEnd(ar1);
            float ar2_end = GetAngleRangeEnd(ar2);
            if (IsAngleRangeInclude(ar1, ar2.first) && IsAngleRangeInclude(ar2, ar1_end))
            {
                std::vector<AngleRange> temp = FindAngleRange(ar2.first, ar1_end);
                out = temp[0];
                // DLOG(INFO) << "ar1: start from " << Utility::ConvertRadToDeg(ar1.first) << " end is " << Utility::ConvertRadToDeg(ar1_end) << " is overlapped with ar2: start from " << Utility::ConvertRadToDeg(ar2.first) << " end is " << Utility::ConvertRadToDeg(ar2_end) << " . their common angle range start from " << Utility::ConvertRadToDeg(out.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(out));
                return out;
            }
            if (IsAngleRangeInclude(ar1, ar2_end) && IsAngleRangeInclude(ar2, ar1.first))
            {
                std::vector<AngleRange> temp = FindAngleRange(ar1.first, ar2_end);
                out = temp[0];
                // DLOG(INFO) << "ar1: start from " << Utility::ConvertRadToDeg(ar1.first) << " end is " << Utility::ConvertRadToDeg(ar1_end) << " is overlapped with ar2: start from " << Utility::ConvertRadToDeg(ar2.first) << " end is " << Utility::ConvertRadToDeg(ar2_end) << " . their common angle range start from " << Utility::ConvertRadToDeg(out.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(out));
                return out;
            }
        }
        // ar1 and ar2 are not overlap and included, make start and range -1
        out.first = -1;
        out.second = -1;
        // DLOG(INFO) << "ar1: start from " << Utility::ConvertRadToDeg(ar1.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(ar1)) << " has nothing to do with ar2: start from " << Utility::ConvertRadToDeg(ar2.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(ar2)) << " . their common angle range start from " << Utility::ConvertRadToDeg(out.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(out));
        return out;
    }

    float GetAngleRangeStart(const AngleRange &ar)
    {
        return ar.first;
    }

    float GetAngleRangeEnd(const AngleRange &ar)
    {
        return ar.first + ar.second;
    }

    bool AngleRange1RightAngleRange2(const AngleRange &ar1, const AngleRange &ar2)
    {
        return (ar1.first < ar2.first);
    }

    bool Angle1RightAngle2(const float &a1, const float &a2)
    {
        bool out = false;
        // a1 and a2 are in rad
        // return true only if a1-a2> angle resolution(set to 0.1 deg)
        if (GetAngleDistance(a1, a2) > ConvertDegToRad(0.001))
        {
            out = true;
        }
        // DLOG(INFO) << "a2: " << ConvertRadToDeg(a1) << " is at Left of a1: " << ConvertRadToDeg(a2) << " ???: " << out;
        return out;
    }

    bool IsEqual(const float &a1, const float &a2)
    {
        // a1 and a2 are in rad
        // return true only if a1-a2< angle resolution(set to 0.1 deg)
        // what if a1 and a2 are greater than 2*pi??
        if (abs(GetAngleDistance(a1, a2)) < ConvertDegToRad(0.01))
        {
            return true;
        }
        DLOG_IF(INFO, abs(GetAngleDistance(a1, a2)) >= 2 * M_PI) << "distance from " << ConvertRadToDeg(a1) << " to " << ConvertRadToDeg(a2) << " is " << ConvertRadToDeg(GetAngleDistance(a1, a2));
        return false;
    }
    bool IsEqual(const AngleRange &ar1, const AngleRange &ar2)
    {

        return IsEqual(ar1.first, ar2.first) &&
               IsEqual(GetAngleRangeEnd(ar1), GetAngleRangeEnd(ar2));
    }

    float GetAngleDistance(const float &angle, const AngleRange &ar)
    {
        return std::min(abs(GetAngleDistance(angle, ar.first)), abs(GetAngleDistance(angle, GetAngleRangeEnd(ar))));
    }

    float GetAngleDistance(const float &angle1, const float &angle2)
    {
        // float a1 = RadToZeroTo2P(angle1);
        // float a2 = RadToZeroTo2P(angle2);
        // just angle1 - angle2,normalization is needed.
        float out = angle2 - angle1;

        if (out >= M_PI || out <= -M_PI)
        {
            out = RadNormalization(out);
        }
        // DLOG(INFO) << "distance from a1: " << ConvertRadToDeg(angle1) << " to a2: " << ConvertRadToDeg(angle2) << " is " << ConvertRadToDeg(out);
        return out;
    }

    std::vector<Edge> GetPolygonEdgesFacingPoint(const Polygon &polygon, const Eigen::Vector2f &point)
    {
        std::vector<Edge> out;
        // check if point inside polygon, if yes, just return

        // DLOG(INFO) << "Point " << point.x() << " " << point.y() << " . polygon  first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y();
        if (IsInsidePolygon(polygon, point) > 0)
        {
            DLOG(INFO) << "point is inside polygon!!";
            return out;
        }

        // find min distance from point to polygon
        float min_distance = GetDistanceFromPolygonToPoint(polygon, point);
        // count how many min_distance, if just one, there is only one edge facing point, if two, then two edges
        // DLOG(INFO) << "min distance is " << min_distance << " . polygon size is " << polygon.size();
        for (uint index = 0; index < polygon.size() - 1; ++index)
        {
            float distance = GetDistanceFromSegmentToPoint(polygon[index], polygon[index + 1], point);
            // DLOG(INFO) << "distance is " << distance;
            if (abs(distance - min_distance) < 1e-3)
            {

                out.emplace_back(Edge(polygon[index], polygon[index + 1]));
                // DLOG(INFO) << "edge start from " << polygon[index].x() << " " << polygon[index].y() << " end at " << polygon[index + 1].x() << " " << polygon[index + 1].y() << " is facing point " << point.x() << " " << point.y();
            }
        }

        return out;
    }

    bool ShareBoundary(const AngleRange &ar1, const AngleRange &ar2)
    {
        float ar1_end = GetAngleRangeEnd(ar1);
        float ar2_end = GetAngleRangeEnd(ar2);
        // DLOG(INFO) << "ar1: start from " << Utility::ConvertRadToDeg(ar1.first) << " end is " << Utility::ConvertRadToDeg(ar1_end) << " . ar2: start from " << Utility::ConvertRadToDeg(ar2.first) << " end is " << Utility::ConvertRadToDeg(ar2_end);
        if (IsAngleRangeInclude(ar1, ar2) || IsAngleRangeInclude(ar2, ar1))
        {
            // DLOG(INFO) << "ar1 and ar2 include each other.";
            return false;
        }
        // if (!IsOverlap(ar1, ar2))
        // {
        //     DLOG(INFO) << "ar1 and ar2 doesn`t overlap.";
        //     return false;
        // }
        if (IsEqual(ar1.first, ar2.first) || IsEqual(ar1_end, ar2_end) || IsEqual(ar1.first, ar2_end) || IsEqual(ar1_end, ar2.first))
        {
            // DLOG(INFO) << "ar1 and ar2 share boundary.";
            return true;
        }
        return false;
    }

    std::vector<AngleRange> SortAngleRange(const std::vector<AngleRange> &ar_vec)
    {
        std::vector<AngleRange> out;
        out = ar_vec;
        sort(out.begin(), out.end(), AngleRange1RightAngleRange2);
        return out;
    }

    bool DuplicateCheck(const std::vector<std::pair<float, float>> &vector, const std::pair<float, float> &element)
    {
        for (const auto &pair : vector)
        {
            if (abs(pair.first - element.first) < 1e-2)
            {
                if (abs(pair.second - element.second) < ConvertDegToRad(1))
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool DuplicateCheck(const std::vector<std::pair<float, float>> &vector, const std::pair<float, float> &element, bool flag)
    {
        for (const auto &pair : vector)
        {
            if (abs(pair.second - element.second) < ConvertDegToRad(1))
            {
                if (flag)
                {
                    if (abs(pair.first - element.first) < 1e-2)
                    {
                        return true;
                    }
                }
                else
                {
                    return true;
                }
            }
        }
        return false;
    }

    std::pair<float, float> CompareAngle(const float &first_angle, const float &second_angle)
    {
        std::pair<float, float> out;
        float greater_angle = 0, smaller_angle = 0;
        // float normalized_first_angle = RadToZeroTo2P(first_angle);
        // float normalized_second_angle = RadToZeroTo2P(second_angle);
        if (first_angle > second_angle)
        {
            greater_angle = first_angle;
            smaller_angle = second_angle;
        }
        else
        {
            greater_angle = second_angle;
            smaller_angle = first_angle;
        }
        out.first = smaller_angle;
        out.second = greater_angle;
        return out;
    }

    int CheckAngleQuadrant(const float &angle)
    {
        float normalized_angle = RadNormalization(angle);
        if (normalized_angle > M_PI / 2)
        {
            return 2;
        }
        if (normalized_angle > 0)
        {
            return 1;
        }
        if (normalized_angle > -M_PI / 2)
        {
            return 4;
        }
        if (normalized_angle > -M_PI)
        {
            return 3;
        }
        return 0;
    }

    std::pair<float, float> AngleNormalization(const float &first_angle, const float &second_angle)
    {
        std::pair<float, float> out;
        int first_quadrant = CheckAngleQuadrant(first_angle);
        int second_quadrant = CheckAngleQuadrant(second_angle);

        // 1. if two angle are both above(1,2) or below(3, 4) x axis, normalize them to 0-2Pi
        if ((first_quadrant == 1 && second_quadrant == 2) ||
            (first_quadrant == 2 && second_quadrant == 1) ||
            (first_quadrant == 3 && second_quadrant == 4) ||
            (first_quadrant == 4 && second_quadrant == 3))
        {
            out.first = RadToZeroTo2P(first_angle);
            out.second = RadToZeroTo2P(second_angle);
        }
        // 2. if two angle are left(2,3)  y axis, normalize them 0 to 2*pi.
        else if ((first_quadrant == 2 && second_quadrant == 3) ||
                 (first_quadrant == 3 && second_quadrant == 2))
        {
            out.first = RadToZeroTo2P(first_angle);
            out.second = RadToZeroTo2P(second_angle);
        }
        // 3. if two angle are  right(1,4) y axis, normalize them to -pi to pi.
        else if (
            (first_quadrant == 1 && second_quadrant == 4) ||
            (first_quadrant == 4 && second_quadrant == 1))
        {
            out.first = RadNormalization(first_angle);
            out.second = RadNormalization(second_angle);
        }
        else if ((first_quadrant == 1 && second_quadrant == 3) ||
                 (first_quadrant == 3 && second_quadrant == 1) ||
                 (first_quadrant == 2 && second_quadrant == 4) ||
                 (first_quadrant == 4 && second_quadrant == 2))
        {
            out.first = RadNormalization(first_angle);
            out.second = RadNormalization(second_angle);
            if (out.first - out.second > M_PI || out.first - out.second < -M_PI)
            {
                out.first = RadToZeroTo2P(first_angle);
                out.second = RadToZeroTo2P(second_angle);
            }
        }
        // 3. rest situation just use the same way to normalize them
        else
        {
            out.first = RadNormalization(first_angle);
            out.second = RadNormalization(second_angle);
        }
        return out;
    }

    int FindIndex(const std::vector<std::pair<HybridAStar::Node3D, float>> &node3d_vec, const std::pair<HybridAStar::Node3D, float> &element)
    {
        int current_index = -1;
        for (size_t i = 0; i < node3d_vec.size(); i++)
        {
            if (element == node3d_vec[i])
            {
                // DLOG(INFO) << "found, index is: " << i;
                current_index = i;
            }
        }
        return current_index;
    }

    int FindIndex(const std::vector<HybridAStar::Node3D> &vector, const HybridAStar::Node3D &element)
    {
        int current_index = -1;
        for (size_t i = 0; i < vector.size(); i++)
        {
            if (Utility::GetDistance(element, vector[i]) < 0.1)
            {
                // DLOG(INFO) << "found, index is: " << i;
                current_index = i;
            }
        }
        return current_index;
    }

    std::vector<float> FormSteeringAngleVec(const float &steering_angle, const int &number_of_successors)
    {
        std::vector<float> out;
        out.emplace_back(0);
        if (number_of_successors <= 1)
        {
            return out;
        };
        for (int i = 1; i <= (number_of_successors - 1) / 2; i++)
        {
            out.emplace_back(i * ConvertDegToRad(steering_angle));
            out.emplace_back(-(i * ConvertDegToRad(steering_angle)));
            // LOG(INFO) << "i is " << i << " i * ConvertDegToRad(steering_angle) " << i * ConvertDegToRad(steering_angle) << " -i * ConvertDegToRad(steering_angle)" << -(i * ConvertDegToRad(steering_angle));
        }
        LOG_IF(INFO, out.size() <= 0) << "out size is ZERO!!";
        return out;
    }

} // namespace Utility
