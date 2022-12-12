/**
 * @file piecewise_cubic_bezier.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this class generate a piecewise cubic bezier curve
 * @version 0.1
 * @date 2021-12-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "piecewise_cubic_bezier.h"

namespace HybridAStar
{

    void PiecewiseCubicBezier::GetAnchorPointDirection()
    {
        // DLOG(INFO) << "GetAnchorPointDirection in:";
        anchor_points_dir_vec_.clear();
        if (anchor_points3d_vec_.size() == 0)
        {
            DLOG(WARNING) << "anchor points are zero!!!";
        }
        else
        {
            for (const auto &point3d : anchor_points3d_vec_)
            {
                float angle = point3d.z();
                Eigen::Vector3f direction;
                direction.x() = std::cos(angle);
                direction.y() = std::sin(angle);
                anchor_points_dir_vec_.emplace_back(direction);
            }
        }
        // DLOG(INFO) << "GetAnchorPointDirection out.";
    }

    void PiecewiseCubicBezier::CalculateControlPoints()
    {
        // DLOG(INFO) << "CalculateControlPoints in:";
        control_points_vec_.clear();
        float start_angle = start_point_.z();
        float goal_angle = goal_point_.z();
        Eigen::Vector3f direction_start;
        direction_start.x() = std::cos(start_angle);
        direction_start.y() = std::sin(start_angle);
        direction_start.z() = 0;
        Eigen::Vector3f direction_goal;
        direction_goal.x() = std::cos(goal_angle);
        direction_goal.y() = std::sin(goal_angle);
        direction_goal.z() = 0;
        Eigen::Vector3f first_control_point, last_control_point;
        uint anchor_points_vec_size = anchor_points3d_vec_.size();

        // DLOG(INFO) << "free anchor points size is " << anchor_points_vec_size << "; " << anchor_points_vec_size + 1 << " bezier!";
        float t_start, t_goal;
        if (anchor_points_vec_size == 0)
        {
            //use the way in paper
            Eigen::Vector2f dist_vector;
            Utility::TypeConversion(goal_point_ - start_point_, dist_vector);
            t_start = dist_vector.norm() / 3;
            first_control_point = start_point_ + direction_start * t_start;
            t_goal = dist_vector.norm() / 3;
            last_control_point = goal_point_ - direction_goal * t_goal;

            control_points_vec_.emplace_back(first_control_point);
            control_points_vec_.emplace_back(last_control_point);
            // DLOG(INFO) << "start is " << start_point_.x() << " " << start_point_.y() << " angle " << Utility::ConvertRadToDeg(start_point_.z()) << " first control point is " << first_control_point.x() << " " << first_control_point.y() << " t_start is " << t_start << " direction start is " << direction_start.x() << " " << direction_start.y() << " " << direction_start.z();
        }
        else if (anchor_points_vec_size == 1)
        {
            Eigen::Vector3f anchor_point_3d = anchor_points3d_vec_.front();
            Eigen::Vector2f anchor_to_start, anchor_to_goal;
            Utility::TypeConversion(anchor_point_3d - start_point_, anchor_to_start);
            Utility::TypeConversion(anchor_point_3d - goal_point_, anchor_to_goal);
            t_start = anchor_to_start.norm() / 3;
            first_control_point = start_point_ + direction_start * t_start;
            t_goal = anchor_to_goal.norm() / 3;
            control_points_vec_.emplace_back(first_control_point);

            last_control_point = goal_point_ - direction_goal * t_goal;
            Eigen::Vector3f c2 = (first_control_point - last_control_point + 4 * anchor_point_3d) / 4;

            control_points_vec_.emplace_back(c2);

            Eigen::Vector3f c3 = 2 * anchor_point_3d - c2;
            control_points_vec_.emplace_back(c3);

            control_points_vec_.emplace_back(last_control_point);
            // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
            // DLOG(INFO) << "second control point is " << c2.x() << " " << c2.y();
            // DLOG(INFO) << "anchor  point is " << anchor_point_3d.x() << " " << anchor_point_3d.y();
            // DLOG(INFO) << "c3 control point is " << c3.x() << " " << c3.y();
            // DLOG(INFO) << "c4 control point is " << last_control_point.x() << " " << last_control_point.y();
        }
        else
        {
            //basically is Ax=b, x is the distance factor, its pre and succ control point is control point pre= anchor-distance_factor*direction.
            Eigen::MatrixXf A = Eigen::MatrixXf::Zero(3 * anchor_points_vec_size, 3 * anchor_points_vec_size);
            Eigen::VectorXf b(3 * anchor_points_vec_size);
            //calculate first and last control points
            Eigen::Vector3f first_anchor_point_3d = anchor_points3d_vec_.front();
            Eigen::Vector3f last_anchor_point_3d = anchor_points3d_vec_.back();
            Eigen::Vector2f anchor_to_start, anchor_to_goal;
            Utility::TypeConversion(first_anchor_point_3d - start_point_, anchor_to_start);
            Utility::TypeConversion(last_anchor_point_3d - goal_point_, anchor_to_goal);
            t_start = anchor_to_start.norm() / 10;
            first_control_point = start_point_ + direction_start * t_start;
            t_goal = anchor_to_goal.norm() / 10;
            control_points_vec_.emplace_back(first_control_point);
            last_control_point = goal_point_ - direction_goal * t_goal;
            // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y() << " " << first_control_point.z();
            // DLOG(INFO) << "last_control_point is " << last_control_point.x() << " " << last_control_point.y() << " " << last_control_point.z();
            // DLOG(INFO) << "direction_goal is " << direction_goal.x() << " " << direction_goal.y() << " " << direction_goal.z();
            // DLOG(INFO) << "t_goal is " << t_goal;
            // for (const auto &point : anchor_points3d_vec_)
            // {
            //     DLOG(INFO) << "anchor points are " << point.x() << " " << point.y() << " " << point.z();
            // }
            for (uint i = 0; i < anchor_points_vec_size; ++i)
            {
                //for first row
                if (i == 0)
                {
                    //initialize A
                    A.block<3, 1>(3 * i, 3 * i + 1) = anchor_points_dir_vec_[i + 1];
                    //initialize b
                    b.block<3, 1>(3 * i, 0) = anchor_points3d_vec_[i + 1] - first_control_point;
                    // DLOG(INFO) << "A first row is " << anchor_points_dir_vec_[i + 1].x() << " " << anchor_points_dir_vec_[i + 1].y() << " " << anchor_points_dir_vec_[i + 1].z();
                    // DLOG(INFO) << "b first row is " << (anchor_points3d_vec_[i + 1] - first_control_point).x() << " " << (anchor_points3d_vec_[i + 1] - first_control_point).y() << " " << (anchor_points3d_vec_[i + 1] - first_control_point).z();
                }
                // for last row
                else if (i == anchor_points_vec_size - 1)
                {
                    //initialize A
                    A.block<3, 1>(3 * i, 3 * i - 1) = anchor_points_dir_vec_[i - 1];
                    //initialize b
                    b.block<3, 1>(3 * i, 0) = last_control_point - anchor_points3d_vec_[i - 1];
                    // DLOG(INFO) << "A last row is " << anchor_points_dir_vec_[i - 1].x() << " " << anchor_points_dir_vec_[i - 1].y() << " " << anchor_points_dir_vec_[i - 1].z();
                    // DLOG(INFO) << "b last row is " << (last_control_point - anchor_points3d_vec_[i - 1]).x() << " " << (last_control_point - anchor_points3d_vec_[i - 1]).y() << " " << (last_control_point - anchor_points3d_vec_[i - 1]).z();
                }
                else
                {
                    //initialize A
                    A.block<3, 1>(3 * i, 3 * i + 1) = anchor_points_dir_vec_[i + 1];
                    A.block<3, 1>(3 * i, 3 * i - 1) = anchor_points_dir_vec_[i - 1];
                    //initialize b
                    b.block<3, 1>(3 * i, 0) = anchor_points3d_vec_[i + 1] - anchor_points3d_vec_[i - 1];
                }
                A.block<3, 1>(3 * i, 3 * i) = 4 * anchor_points_dir_vec_[i];
            }
            Eigen::VectorXf distance_factor(3 * anchor_points_vec_size);
            distance_factor = A.colPivHouseholderQr().solve(b);
            // for (int i = 0; i < distance_factor.size(); ++i)
            // {
            //     DLOG(INFO) << i << " distance factor is " << distance_factor[i];
            // }
            for (uint i = 0; i < anchor_points_vec_size; ++i)
            {
                Eigen::Vector3f current_dir = anchor_points_dir_vec_[i];

                Eigen::Vector3f pre_control_point = anchor_points3d_vec_[i] - current_dir * std::abs(distance_factor[3 * i]);
                Eigen::Vector3f succ_control_point = anchor_points3d_vec_[i] + current_dir * std::abs(distance_factor[3 * i]);
                // DLOG(INFO) << i << " anchor point is " << anchor_points3d_vec_[i].x() << " " << anchor_points3d_vec_[i].y() << " " << anchor_points3d_vec_[i].z();
                // DLOG(INFO) << i << " current_dir is " << current_dir.x() << " " << current_dir.y() << " " << current_dir.z();
                // DLOG(INFO) << i << " distance factor is " << distance_factor[3 * i];
                control_points_vec_.emplace_back(pre_control_point);
                control_points_vec_.emplace_back(succ_control_point);
            }
            control_points_vec_.emplace_back(last_control_point);
        }
        // DLOG(INFO) << "control_points_vec_ size is " << control_points_vec_.size();
        // int index = 0;
        // for (const auto &point : control_points_vec_)
        // {
        //     DLOG(INFO) << index << " control point is " << point.x() << " " << point.y();
        //     index++;
        // }
        // DLOG(INFO) << "CalculateControlPoints out.";
    }

    void PiecewiseCubicBezier::CalculatePointsVec()
    {
        // DLOG(INFO) << "CalculatePointsVec in:";
        points_vec_.clear();

        points_vec_.emplace_back(start_point_);

        uint count = 0;
        uint anchor_point_index = 0;
        for (uint control_point_index = 0; control_point_index < control_points_vec_.size(); ++control_point_index)
        {
            points_vec_.emplace_back(control_points_vec_[control_point_index]);
            // DLOG(INFO) << "control point is " << control_points_vec_[control_point_index].x() << " " << control_points_vec_[control_point_index].y();
            count++;
            if (count % 3 == 2)
            {
                if (anchor_points3d_vec_.size() != 0 && anchor_point_index < anchor_points3d_vec_.size())
                {

                    Eigen::Vector3f anchor_point_3d = anchor_points3d_vec_[anchor_point_index];
                    points_vec_.emplace_back(anchor_point_3d);
                    // DLOG(INFO) << "anchor point 2d is " << anchor_point_3d.x() << " " << anchor_point_3d.y();
                    // DLOG(INFO) << "anchor point index is " << anchor_point_index;
                    anchor_point_index++;
                    count++;
                }
            }
        }
        points_vec_.emplace_back(goal_point_);
        // DLOG(INFO) << "CalculatePointsVec out.";
        // for (const auto &point : points_vec_)
        // {
        //     DLOG(INFO) << "points are " << point.x() << " " << point.y();
        // }
        // DLOG(INFO) << "point vec size is " << points_vec_.size();
    }

    void PiecewiseCubicBezier::CalculateCubicBezier()
    {
        // DLOG(INFO) << "CalculateCubicBezier in:";
        cubic_bezier_vec_.clear();
        Eigen::Matrix<float, 3, 4> points_lists;
        uint j = 0;
        for (uint i = 0; i < points_vec_.size(); ++i)
        {
            // DLOG(INFO) << "j equal to " << j;
            points_lists.block<3, 1>(0, j) = points_vec_[i];
            // points_lists.emplace_back(points_vec_[i]);
            // DLOG(INFO) << i << "th points are " << points_vec_[i].x() << " " << points_vec_[i].y();
            j++;
            if (j == 4)
            {
                cubic_bezier_vec_.emplace_back(CubicBezier::CubicBezier(points_lists));
                // for (int index = 0; index < 4; index++)
                // {
                //     DLOG(INFO) << "point x " << points_lists(0, index) << " y " << points_lists(1, index);
                // }
                i = i - 1;

                j = 0;
            }
        }
        // DLOG(INFO) << "CalculateCubicBezier out.";
        // DLOG(INFO) << "size cubic bezier vec is " << cubic_bezier_vec_.size();
    }

    float PiecewiseCubicBezier::GetAngleAt(const float &u)
    {
        float angle;
        int total_number_of_bezier = cubic_bezier_vec_.size();
        if (total_number_of_bezier == 0)
        {
            DLOG(INFO) << "No bezier, Please calculate first";
            angle = -100000;
        }
        else
        {
            int current_index_of_bezier;
            current_index_of_bezier = (int)std::floor(u * total_number_of_bezier);
            float current_factor = total_number_of_bezier * u - current_index_of_bezier;
            angle = cubic_bezier_vec_[current_index_of_bezier].GetAngleAt(current_factor);
        }
        return angle;
    }

    Eigen::Vector3f PiecewiseCubicBezier::GetValueAt(const float &u)
    {
        Eigen::Vector3f out;

        int total_number_of_bezier = cubic_bezier_vec_.size();
        if (total_number_of_bezier == 0)
        {
            DLOG(INFO) << "No bezier, Please calculate first";
            out.x() = -10000;
            out.y() = -10000;
        }
        else
        {
            int current_index_of_bezier;
            current_index_of_bezier = (int)std::floor(u * total_number_of_bezier);
            float current_factor = total_number_of_bezier * u - current_index_of_bezier;
            // DLOG(INFO) << "u is " << u << " total number of bezier is " << total_number_of_bezier << " current index of bezier is " << current_index_of_bezier << " current factor is " << current_factor;
            out = cubic_bezier_vec_[current_index_of_bezier].GetValueAt(current_factor);
        }
        return out;
    }

    void PiecewiseCubicBezier::CalculateLength()
    {
        if (cubic_bezier_vec_.size() == 0)
        {
            length_ = 0;
            // DLOG(INFO) << "length is 0 due to no cubic bezier;";
        }
        else
        {
            for (auto &cubic_bezier : cubic_bezier_vec_)
            {
                float cubic_bezier_length = cubic_bezier.GetLength();
                // DLOG(INFO) << "cubic bezier length is " << cubic_bezier_length;
                length_ += cubic_bezier_length;
            }
            // DLOG(INFO) << "length is " << length_;
        }
    }
    float PiecewiseCubicBezier::GetCurvatureAt(const float &u)
    {
        float out;

        int total_number_of_bezier = cubic_bezier_vec_.size();
        if (total_number_of_bezier == 0)
        {
            DLOG(INFO) << "No bezier, Please calculate first";
            out = -10000;
        }
        else
        {
            int current_index_of_bezier;
            current_index_of_bezier = (int)std::floor(u * total_number_of_bezier);
            float current_factor = total_number_of_bezier * u - current_index_of_bezier;
            // DLOG(INFO) << "u is " << u << " total number of bezier is " << total_number_of_bezier << " current index of bezier is " << current_index_of_bezier << " current factor is " << current_factor;
            out = cubic_bezier_vec_[current_index_of_bezier].GetCurvatureAt(current_factor);
        }
        return out;
    }

    float PiecewiseCubicBezier::GetTotalCurvature()
    {
        float total_curvature = 0;
        if (cubic_bezier_vec_.size() == 0)
        {
            length_ = 0;
            // DLOG(INFO) << "length is 0 due to no cubic bezier;";
        }
        else
        {
            for (auto &cubic_bezier : cubic_bezier_vec_)
            {
                // DLOG(INFO) << "cubic bezier length is " << cubic_bezier.GetTotalCurvature();
                total_curvature += cubic_bezier.GetTotalCurvature();
            }
            // DLOG(INFO) << "length is " << length_;
        }
        return total_curvature;
    }

    std::vector<Eigen::Vector3f> PiecewiseCubicBezier::ConvertPiecewiseCubicBezierToVector3f(const int &number_of_points)
    {
        // DLOG(INFO) << "ConvertPiecewiseCubicBezierToVector3f in:";
        CalculateCubicBezier();
        // DLOG(INFO) << "size cubic bezier vec is " << cubic_bezier_vec_.size();
        std::vector<Eigen::Vector3f> out;
        int index = 0;
        for (auto &bezier : cubic_bezier_vec_)
        {
            std::vector<Eigen::Vector3f> path;
            path = bezier.ConvertCubicBezierToVector3f(number_of_points);
            // DLOG(INFO) << index << "th cubic bezier";
            // for (const auto &point : path)
            // {
            //     DLOG(INFO) << "point x " << point.x() << " y " << point.y() << " z " << Utility::ConvertRadToDeg(point.z());
            // }
            out.insert(out.end(), path.begin(), path.end());
            index++;
        }
        // DLOG(INFO) << "ConvertPiecewiseCubicBezierToVector3f out.";
        return out;
    }
}