/**
 * @file bezier.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this is a basic bezier class
 * @version 0.1
 * @date 2021-12-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <vector>
#include "utility.h"
#include <cstdlib>
#include <math.h>

namespace CubicBezier
{
    class CubicBezier
    {
    public:
        CubicBezier()
        {
            basis_matrix_ << 1, -3, 3, -1,
                0, 3, -6, 3,
                0, 0, 3, -3,
                0, 0, 0, 1;
        };
        CubicBezier(const Eigen::Vector3f &start, const Eigen::Vector3f &goal, uint width, uint height)
        {
            basis_matrix_ << 1, -3, 3, -1,
                0, 3, -6, 3,
                0, 0, 3, -3,
                0, 0, 0, 1;
            start_point_ = start;
            goal_point_ = goal;
            map_width_ = width;
            map_height_ = height;
            CalculateControlPoints();
            // CalculateLength();
            CalculateAnchorPoints();
        };
        CubicBezier(const Eigen::Matrix<float, 3, 4> &point_matrix)
        {

            basis_matrix_ << 1, -3, 3, -1,
                0, 3, -6, 3,
                0, 0, 3, -3,
                0, 0, 0, 1;
            if (point_matrix.rows() != 3 || point_matrix.cols() != 4)
            {
                // DLOG(WARNING) << "points_vec size is not correct!!!";
            }
            else
            {
                geometrical_constraint_matrix_ = point_matrix;
                start_point_ = geometrical_constraint_matrix_.block<3, 1>(0, 0);
                goal_point_ = geometrical_constraint_matrix_.block<3, 1>(0, 3);
            }
            // CalculateLength();

            CalculateAnchorPoints();
        }

        Eigen::Vector3f GetValueAt(const float &t);

        float GetAngleAt(const float &t);

        // std::vector<Eigen::Vector3f> GetControlPoints() const { return control_points_vec_; };
        std::vector<Eigen::Vector3f> GetAnchorPoints() const { return anchor_points_vec_; };

        float GetLength()
        {
            CalculateLength();
            return length_;
        };

        float GetCurvatureAt(const float &t);
        float GetMaxCurvature();
        float GetTotalCurvature();

        std::vector<Eigen::Vector3f> ConvertCubicBezierToVector3f(const uint &number_of_points);

    private:
        /**
         * @brief split curve into 100, sum all euler distance.
         * 
         */
        void CalculateLength();

        Eigen::Vector4f CalculateCoefficient(const float &t);

        Eigen::Vector4f CalculateFirstOrderDerivativeCoefficient(const float &t);

        Eigen::Vector4f CalculateSecondOrderDerivativeCoefficient(const float &t);

        void CalculateControlPoints();

        void CalculateAnchorPoints();

        Eigen::Vector3f GetFirstOrderDerivativeValueAt(const float &t);

        Eigen::Vector3f GetSecondOrderDerivativeValueAt(const float &t);

    private:
        /**
         * @brief this is matrix form of anchor and control points
         * row is 3 due to eigen vector3d, column is 4 due to this is a cubic bezier
         */
        Eigen::Matrix<float, 3, 4> geometrical_constraint_matrix_;

        Eigen::Matrix<float, 4, 4> basis_matrix_;

        /**
         * @brief actually is start and goal points
         * 
         */
        std::vector<Eigen::Vector3f> anchor_points_vec_;

        Eigen::Vector3f start_point_;
        Eigen::Vector3f goal_point_;

        float length_ = 0;

        float map_width_;
        float map_height_;
        /**
         * @brief determine how to calculate control points, random or use the way in paper
         * 
         */
        bool use_random_ = false;
    };
}