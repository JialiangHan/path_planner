/**
 * @file computational_geometry.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this file contain some classes and function which is useful in computational geometry
 * @version 0.1
 * @date 2022-02-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>
namespace ComputationalGeometry
{
    // node: use eigen vector
    /**
     * @brief this is only work for vector2d, for 3d please use cross in eigen
     *
     * @param p1
     * @param p2
     * @return float
     */
    float CrossProduct(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);
    // segment: write a new class
    class Segment
    {
    public:
        Segment(){};
        Segment(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
        {
            start_ = start;
            end_ = end;
            CalculateLength();
        }
        /**
         * @brief Construct a new Segment object
         *
         * @param start
         * @param radius
         * @param rad angle in rad is start at 3 o`clock and positive is CCW
         */
        Segment(const Eigen::Vector2f &start, const float &radius, const float &rad);

        Eigen::Vector2f GetStart() const { return start_; };
        Eigen::Vector2f GetEnd() const { return end_; };

        bool IsIntersect(const Segment &another_segment) const;
        Eigen::Vector2f FindIntersectionPoint(const Segment &another_segment) const;
        /**
         * @brief check if point lines on this segment
         *
         * @param point
         * @return true
         * @return false
         */
        bool OnSegment(const Eigen::Vector2f &point) const;

    public:
        float length_;

    private:
        void CalculateLength() { length_ = (end_ - start_).norm(); };

    private:
        Eigen::Vector2f start_;
        Eigen::Vector2f end_;
    };
    // current this polygon is only rectangle
    class Polygon
    {
    public:
        Polygon(){};
        Polygon(const std::vector<Segment> &segment_vec)
        {
            segment_vec_ = segment_vec;
        }

    private:
        void CalculateArea();

    private:
        std::vector<Segment> segment_vec_;
        float area_;
    };
}