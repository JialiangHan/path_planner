/**
 * @file cubic_bezier.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief for cubic_bezier
 * @version 0.1
 * @date 2021-12-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "cubic_bezier.h"

namespace CubicBezier
{
    Eigen::Vector4f CubicBezier::CalculateCoefficient(const float &t)
    {
        Eigen::Vector4f out;

        for (uint i = 0; i < out.size(); ++i)
        {
            out[i] = std::pow(t, i);
            // DLOG(INFO) << i << "th element in coefficient matrix is " << out[i];
        }
        return out;
    }

    Eigen::Vector4f CubicBezier::CalculateFirstOrderDerivativeCoefficient(const float &t)
    {
        Eigen::Vector4f out;

        for (uint i = 0; i < out.size(); ++i)
        {
            out[i] = i * (i - 1 < 0 ? 0 : std::pow(t, i - 1));
            // DLOG(INFO) << "t: " << t << " i-1 " << i - 1 << " std::pow(t, i - 1)" << (std::pow(t, i - 1));
            // DLOG(INFO) << i << "th element in first order derivation coefficient matrix is " << out[i];
        }
        return out;
    }
    Eigen::Vector4f CubicBezier::CalculateSecondOrderDerivativeCoefficient(const float &t)
    {
        Eigen::Vector4f out;

        for (uint i = 0; i < out.size(); ++i)
        {
            out[i] = i * (i - 1) * (i - 2 < 0 ? 0 : std::pow(t, i - 2));
            // DLOG(INFO) << "t: " << t << " i-1 " << i - 1 << " std::pow(t, i - 1)" << (std::pow(t, i - 1));
            // DLOG(INFO) << i << "th element in first order derivation coefficient matrix is " << out[i];
        }
        return out;
    }
    Eigen::Vector3f CubicBezier::GetFirstOrderDerivativeValueAt(const float &t)
    {
        Eigen::Vector3f out;
        // DLOG(INFO) << "t " << t;
        Eigen::Vector4f coefficient = CalculateFirstOrderDerivativeCoefficient(t);
        out = geometrical_constraint_matrix_ * basis_matrix_ * coefficient;
        // DLOG_IF(INFO, out.x() == 0 && out.y() == 0) << "first order derivative coefficient is " << coefficient[0] << " " << coefficient[1] << " " << coefficient[2] << " " << coefficient[3];
        return out;
    }
    Eigen::Vector3f CubicBezier::GetSecondOrderDerivativeValueAt(const float &t)
    {
        Eigen::Vector3f out;
        // DLOG(INFO) << "t " << t;
        out = geometrical_constraint_matrix_ * basis_matrix_ * CalculateSecondOrderDerivativeCoefficient(t);

        return out;
    }

    void CubicBezier::CalculateControlPoints()
    {

        // control_points_vec_.clear();
        float start_angle = start_point_.z();
        float goal_angle = goal_point_.z();
        Eigen::Vector3f first_control_point(0, 0, 0), second_control_point(0, 0, 0), direction(0, 0, 0);
        direction.x() = std::cos(start_angle);
        direction.y() = std::sin(start_angle);
        if (use_random_)
        {
            Utility::Polygon polygon;
            polygon = Utility::CreatePolygon(map_width_, map_height_);
            // DLOG(INFO) << " map width is " << map_width_ << " height is " << map_height_;
            //t is some random number;
            uint t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
            first_control_point = start_point_ + direction * t;
            // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
            int flag = Utility::IsInsidePolygon(polygon, first_control_point);
            while (flag < 1)
            {
                t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
                first_control_point = start_point_ + direction * t;
                // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
                flag = Utility::IsInsidePolygon(polygon, first_control_point);
                // DLOG(INFO) << " is this point inside map? " << flag;
            }
            direction.x() = std::cos(goal_angle);
            direction.y() = std::sin(goal_angle);
            //t is another random number;
            t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
            second_control_point = goal_point_ - direction * t;
            // DLOG(INFO) << "second control point is " << second_control_point.x() << " " << second_control_point.y();
            flag = Utility::IsInsidePolygon(polygon, second_control_point);
            while (flag < 1)
            {
                t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
                second_control_point = goal_point_ - direction * t;
                // DLOG(INFO) << "second control point is " << second_control_point.x() << " " << second_control_point.y();
                flag = Utility::IsInsidePolygon(polygon, second_control_point);
                // DLOG(INFO) << " is this point inside map? " << flag;
            }
        }
        else
        { //use the way in paper
            direction.x() = std::cos(start_angle);
            direction.y() = std::sin(start_angle);
            Eigen::Vector2f temp;
            Utility::TypeConversion(goal_point_ - start_point_, temp);
            float t = temp.norm() / 3;
            first_control_point = start_point_ + direction * t;
            direction.x() = std::cos(goal_angle);
            direction.y() = std::sin(goal_angle);
            second_control_point = goal_point_ - direction * t;
        }
        // control_points_vec_.emplace_back(first_control_point);
        // control_points_vec_.emplace_back(second_control_point);

        geometrical_constraint_matrix_.block<3, 1>(0, 0) = start_point_;
        geometrical_constraint_matrix_.block<3, 1>(0, 1) = first_control_point;
        geometrical_constraint_matrix_.block<3, 1>(0, 2) = second_control_point;
        geometrical_constraint_matrix_.block<3, 1>(0, 3) = goal_point_;
        // LOG(INFO) << "start point is " << start_point_.x() << " " << start_point_.y();
        // LOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
        // LOG(INFO) << "second control point is " << second_control_point.x() << " " << second_control_point.y();
        // LOG(INFO) << "end point is " << goal_point_.x() << " " << goal_point_.y();
    }

    void CubicBezier::CalculateAnchorPoints()
    {
        anchor_points_vec_.clear();
        anchor_points_vec_.emplace_back(start_point_);
        anchor_points_vec_.emplace_back(goal_point_);
        // for (const auto &point : anchor_points_vec_)
        // {
        // DLOG(INFO) << "anchor points is " << point.x() << " " << point.y();
        // }
    }

    Eigen::Vector3f CubicBezier::GetValueAt(const float &t)
    {
        Eigen::Vector3f out;
        out = geometrical_constraint_matrix_ * basis_matrix_ * CalculateCoefficient(t);
        // DLOG(INFO) << "out is " << out.x() << " " << out.y() << " " << out.z();
        return out;
    }

    float CubicBezier::GetAngleAt(const float &t)
    {
        // DLOG(INFO) << "t " << t;
        Eigen::Vector3f derivative_value = GetFirstOrderDerivativeValueAt(t);
        float angle = std::atan2(derivative_value.y(), derivative_value.x());
        return angle;
    }
    void CubicBezier::CalculateLength()
    {
        // DLOG(INFO) << "In calculateLength()";
        for (uint i = 0; i < 100; ++i)
        {
            Eigen::Vector2f temp;
            Utility::TypeConversion(GetValueAt((i + 1) / 100.0) - GetValueAt(i / 100.0), temp);
            length_ += temp.norm();
        }
        // DLOG(INFO) << "length is " << length_;
    }
    std::vector<Eigen::Vector3f> CubicBezier::ConvertCubicBezierToVector3f(const uint &number_of_points)
    {
        std::vector<Eigen::Vector3f> out;

        CalculateLength();
        // uint size = GetLength() / 2;
        uint size = number_of_points;
        DLOG_IF(WARNING, size == 0) << "WARNING: path size is zero!!!";
        for (uint i = 0; i < size + 1; ++i)
        {
            Eigen::Vector3f point3d;
            // DLOG(INFO) << " i/size = " << (float)i / size;
            point3d = GetValueAt((float)i / size);
            point3d.z() = GetAngleAt((float)i / size);
            // DLOG(INFO) << "point3d is " << point3d.x() << " " << point3d.y() << " " << point3d.z();
            out.emplace_back(point3d);
        }
        return out;
    }

    float CubicBezier::GetTotalCurvature()
    {
        float total_curvature = 0;
        // DLOG(INFO) << "In calculateLength()";
        for (uint i = 0; i < 100; ++i)
        {
            total_curvature += GetCurvatureAt(i / 100.0);
        }
        // DLOG(INFO) << "total curvature is " << total_curvature;
        return total_curvature;
    }

    float CubicBezier::GetMaxCurvature()
    {
        float max_curvature = 0;
        // DLOG(INFO) << "In calculateLength()";
        for (uint i = 0; i < 100; ++i)
        {
            float current_curvature = GetCurvatureAt(i / 100.0);
            if (max_curvature < current_curvature)
            {
                max_curvature = current_curvature;
            }
        }
        // DLOG(INFO) << "total curvature is " << total_curvature;
        return max_curvature;
    }

    float CubicBezier::GetCurvatureAt(const float &t)
    {
        float curvature = 0;
        Eigen::Vector2f first_order_derivative, second_order_derivative;
        Utility::TypeConversion(GetFirstOrderDerivativeValueAt(t), first_order_derivative);
        Utility::TypeConversion(GetSecondOrderDerivativeValueAt(t), second_order_derivative);
        if (first_order_derivative.norm() != 0)
        {
            curvature = std::abs(Utility::CrossProduct(first_order_derivative, second_order_derivative)) / std::pow(first_order_derivative.norm(), 3);
        }
        else
        {
            curvature = 100000;
        }

        // DLOG_IF(INFO, std::isnan(curvature)) << "curvature at " << t << " is " << curvature << " its first derivative norm is " << first_order_derivative.norm() << " first derivative is " << first_order_derivative.x() << " " << first_order_derivative.y();
        return curvature;
    }
}