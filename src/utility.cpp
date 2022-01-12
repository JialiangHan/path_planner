#include "utility.h"

    namespace Utility
    {
        float GetDistance(const HybridAStar::Node2D &start, const HybridAStar::Node2D &goal)
        {
            float dx = std::abs(start.GetX() - goal.GetX());
            float dy = std::abs(start.GetY() - goal.GetY());
            float distance = std::sqrt((dx * dx) + (dy * dy));
            return distance;
        }
        float GetDistance(const HybridAStar::Node3D &start, const HybridAStar::Node3D &goal)
        {
            float dx = std::abs(start.GetX() - goal.GetX());
            float dy = std::abs(start.GetY() - goal.GetY());
            float distance = std::sqrt((dx * dx) + (dy * dy));
            return distance;
        }
        bool IsCloseEnough(const HybridAStar::Node3D &start, const HybridAStar::Node3D &goal, const float &distance_range, const float &angle_range)
        {
            float distance = GetDistance(start, goal);
            if (distance < distance_range)
            {
                float angle_diff = RadNormalization(start.GetT() - goal.GetT());
                if (angle_diff <= angle_range)
                {
                    DLOG(INFO) << "two node distance and orientation are close enough, return true";
                    return true;
                }
                else
                {
                    DLOG(INFO) << "two node distance is close enough but orientation is too far is " << angle_diff;
                    return false;
                }
            }
            else
            {
                // DLOG(INFO) << "too far, distance is " << distance;
                return false;
            }
        }
        HybridAStar::Node3D ConvertVector3fToNode3D(const Eigen::Vector3f &vector3d)
        {
            float x = vector3d.x();
            float y = vector3d.y();
            float t = vector3d.z();
            HybridAStar::Node3D node3d(x, y, t, 0, 0, nullptr);
            return node3d;
        }

        Eigen::Vector3f ConvertNode3DToVector3f(const HybridAStar::Node3D &node3d)
        {
            Eigen::Vector3f out;
            out.x() = node3d.GetX();
            out.y() = node3d.GetY();
            out.z() = node3d.GetT();
            return out;
        }
        nav_msgs::Path ConvertVectorVector3DToRosPath(const std::vector<Eigen::Vector3f> &vector_3d_vec)
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

        void ConvertRosPathToVectorVector3D(const nav_msgs::Path::ConstPtr &path, std::vector<Eigen::Vector3f> &vector_3d_vec)
        {
            vector_3d_vec.clear();
            for (uint i = 0; i < path->poses.size(); ++i)
            {
                Eigen::Vector3f point;
                point.x() = (path->poses[i].pose.position.x);
                point.y() = (path->poses[i].pose.position.y);
                //not sure this is correct;
                point.z() = (path->poses[i].pose.position.z);
                vector_3d_vec.emplace_back(point);
            }
        }
        Eigen::Vector3f ConvertVector2fToVector3f(const Eigen::Vector2f &vector_2d)
        {
            Eigen::Vector3f out;
            out.x() = vector_2d.x();
            out.y() = vector_2d.y();
            out.z() = 0;
            return out;
        }
        float ConvertDegToRad(const float &deg)
        {
            float rad;
            rad = deg / 360 * 2 * M_PI;
            return RadToZeroTo2P(rad);
        }
        float ConvertRadToDeg(const float &rad)
        {
            float deg;
            deg = rad / 2 / M_PI * 360;
            return DegToZeroTo2P(deg);
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

            return out;
        }
        Eigen::Vector2f ConvertVector3fToVector2f(const Eigen::Vector3f &vector_3d)
        {
            Eigen::Vector2f out;
            out.x() = vector_3d.x();
            out.y() = vector_3d.y();
            return out;
        }

        Eigen::Vector2f ConvertIndexToEigenVector2f(const int &index, const int &map_width)
        {
            Eigen::Vector2f out;
            out.x() = (index % map_width);
            out.y() = (index / map_width);
            return out;
        }

        bool OnSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p3, const Eigen::Vector2f &p2)
        {
            if (p2.x() <= std::max(p1.x(), p3.x()) && p2.x() >= std::min(p1.x(), p3.x()) && p2.y() <= std::max(p1.y(), p3.y()) && p2.y() >= std::min(p1.y(), p3.y()))
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
            //DLOG(INFO) << "result is " << out;
            return out;
        }

        int IsIntersect(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, const Eigen::Vector2f &p4)
        {
            if (std::max(p3.x(), p4.x()) < std::min(p1.x(), p2.x()) ||
                std::max(p3.y(), p4.y()) < std::min(p1.y(), p2.y()) ||
                std::max(p1.x(), p2.x()) < std::min(p3.x(), p4.x()) ||
                std::max(p1.y(), p2.y()) < std::min(p3.y(), p4.y()))
            {
                //DLOG(INFO) << "these two segments are far away!";
                return 0;
            }
            else
            {
                if (CrossProduct(p1 - p3, p4 - p3) * CrossProduct(p2 - p3, p4 - p3) <= 0 &&
                    CrossProduct(p3 - p2, p1 - p2) * CrossProduct(p4 - p2, p1 - p2) <= 0)
                {
                    //DLOG(INFO) << "Intersection!!";
                    return 1;
                }
                return 0;
            }
        }

        Eigen::Vector2f FindIntersectionPoint(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, const Eigen::Vector2f &p4)
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
        int IsAboveSegment(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3)
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
        int IsInsidePolygon(const std::vector<Eigen::Vector2f> &polygon, const Eigen::Vector2f &point)
        {
            Eigen::Vector2f far_away_point(10000, point.y());
            int number_intersection = 0;
            for (uint i = 0; i < polygon.size() - 1; ++i)
            {
                //check if point is on polygon edge
                if (OnSegment(polygon[i], polygon[i + 1], point))
                {
                    //DLOG(INFO) << "Point is on polygon edge.";
                    return 1;
                }
                //if edge intersect with vector from point to far away point
                if (IsIntersect(point, far_away_point, polygon[i], polygon[i + 1]) == 1)
                {
                    Eigen::Vector2f intersection_point = FindIntersectionPoint(point, far_away_point, polygon[i], polygon[i + 1]);
                    //DLOG(INFO) << "point horizontal vector is intersecting polygon!";
                    if ((polygon[i] - intersection_point).norm() < 1e-3)
                    {
                        if (IsAboveSegment(point, far_away_point, polygon[i + 1]) == 0)
                        {
                            number_intersection += 1;
                            //DLOG(INFO) << "true intersection!! +1";
                        }
                    }
                    else if ((polygon[i + 1] - intersection_point).norm() < 1e-3)
                    {
                        if (IsAboveSegment(point, far_away_point, polygon[i]) == 0)
                        {
                            number_intersection += 1;
                            //DLOG(INFO) << "true intersection!! +1";
                        }
                    }
                    else
                    {
                        number_intersection += 1;
                        //DLOG(INFO) << "true intersection!! +1";
                    }
                }
            }
            //DLOG(INFO) << "number of intersection is " << number_intersection;
            if ((number_intersection % 2) == 0)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        int IsInsidePolygon(const std::vector<Eigen::Vector2f> &polygon, const Eigen::Vector3f &point)
        {
            Eigen::Vector2f point_2d = Utility::ConvertVector3fToVector2f(point);
            return IsInsidePolygon(polygon, point_2d);
        }
        std::vector<Eigen::Vector2f> CreatePolygon(const float &width, const float &height)
        {
            std::vector<Eigen::Vector2f> polygon;
            polygon.emplace_back(Eigen::Vector2f(0, 0));
            polygon.emplace_back(Eigen::Vector2f(width, 0));
            polygon.emplace_back(Eigen::Vector2f(width, height));
            polygon.emplace_back(Eigen::Vector2f(0, height));
            polygon.emplace_back(Eigen::Vector2f(0, 0));
            return polygon;
        }

        float GetDistanceFromVector2fToVector3f(const Eigen::Vector3f &vector_3d, const Eigen::Vector2f &vector_2d)
        {
            float distance;
            float delta_x = vector_2d.x() - vector_3d.x();
            float delta_y = vector_2d.y() - vector_3d.y();
            distance = sqrt(delta_x * delta_x + delta_y * delta_y);
            return distance;
        }

        float Clamp(const float &number, const float &upper_bound, const float &lower_bound)
        {
            return std::max(lower_bound, std::min(number, upper_bound));
        }
    }
