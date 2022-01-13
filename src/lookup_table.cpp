/**
 * @file lookup_table.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief see lookup_table.h
 * @version 0.1
 * @date 2022-01-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "lookup_table.h"

namespace HybridAStar
{
    LookupTable::LookupTable(const ParameterCollisionDetection &params)
    {
        params_ = params;
    }

    void LookupTable::Initialize(const int &width, const int &height)
    {
        if (map_width_ != width || map_height_ != height)
        {
            Clear();
            map_width_ = width;
            map_height_ = height;
            ros::Time t0 = ros::Time::now();
            if (params_.curve_type == 0)
            {
                CalculateDubinsLookup();
            }
            else if (params_.curve_type == 1)
            {

                CalculateReedsSheppLookup();
            }
            else
            {
                CalculateCubicBezierLookupV1();
            }
            ros::Time t1 = ros::Time::now();
            ros::Duration d(t1 - t0);
            LOG(INFO) << "build lookup table in ms: " << d * 1000;
        }
    }

    void LookupTable::Clear()
    {
        dubins_lookup_.clear();
        reeds_shepp_lookup_.clear();
        cubic_bezier_lookup_.clear();
    }

    float LookupTable::GetDubinsCost(const Node3D &node3d) const
    {
        float out = 0;
        int index = CalculateNode3DIndex(node3d);
        if (dubins_lookup_.find(index) != dubins_lookup_.end())
        {
            out = dubins_lookup_.at(index);
        }
        else
        {
            DLOG(INFO) << "index not found in dubins map, node is " << node3d.GetX() << " " << node3d.GetY() << " " << node3d.GetT();
        }
        return out;
    }
    float
    LookupTable::GetReedsSheppCost(const Node3D &node3d) const
    {

        float out = 0;
        int index = CalculateNode3DIndex(node3d);
        if (reeds_shepp_lookup_.find(index) != reeds_shepp_lookup_.end())
        {
            out = reeds_shepp_lookup_.at(index);
        }
        else
        {
            DLOG(INFO) << "index not found in rs map, node is " << node3d.GetX() << " " << node3d.GetY() << " " << node3d.GetT();
        }
        return out;
    }
    float LookupTable::GetCubicBezierCost(const Node3D &node3d) const
    {
        float out = 0;
        int index = CalculateNode3DIndex(node3d);
        if (cubic_bezier_lookup_.find(index) != cubic_bezier_lookup_.end())
        {
            out = cubic_bezier_lookup_.at(index);
        }
        else
        {
            DLOG(INFO) << "index " << index << " not found in cubic bezier map, node is " << node3d.GetX() << " " << node3d.GetY() << " " << node3d.GetT();
        }
        return out;
    }
    int LookupTable::CalculateNode3DIndex(const Node3D &node3d) const
    {
        int out;
        float x_float = node3d.GetX();
        float y_float = node3d.GetY();
        float t_float = Utility::RadToZeroTo2P(node3d.GetT());
        out = CalculateNode3DIndex(x_float, y_float, t_float);
        return out;
    }
    int LookupTable::CalculateNode3DIndex(const float &x, const float &y, const float &theta) const
    {
        int out;
        const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
        out = (int)(theta / delta_heading_in_rad) * map_width_ * map_height_ * params_.position_resolution * params_.position_resolution + (int)(y * params_.position_resolution) * map_width_ + (int)(x * params_.position_resolution);

        return out;
    }
    void LookupTable::CalculateDubinsLookup()
    {
        float x = 0, y = 0, theta = 0, cost = 0;
        ompl::base::DubinsStateSpace dubinsPath(params_.min_turning_radius);
        State *dbStart = (State *)dubinsPath.allocState();
        State *dbEnd = (State *)dubinsPath.allocState();
        dbStart->setXY(x, y);
        dbStart->setYaw(theta);
        const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
        while (x < map_width_)
        {
            y = 0;
            theta = 0;
            while (y < map_height_)
            {
                theta = 0;
                if (x == 0 && y == 0)
                {
                    continue;
                }
                while (theta < 2 * M_PI)
                {
                    dbEnd->setXY(x, y);
                    dbEnd->setYaw(theta);
                    cost = dubinsPath.distance(dbStart, dbEnd);
                    int index = CalculateNode3DIndex(x, y, theta);
                    dubins_lookup_.emplace(index, cost);
                    DLOG(INFO) << "point is " << x << " " << y << " " << theta << " index is " << index << " cost is " << cost;
                    theta += delta_heading_in_rad;
                }
                y += 1.0f / params_.position_resolution;
            }
            x += 1.0f / params_.position_resolution;
        }
        // DLOG(INFO) << "CalculateDubinsLookup done.";
    }
    void LookupTable::CalculateReedsSheppLookup()
    {
        float x = 0, y = 0, theta = 0, cost = 0;
        ompl::base::ReedsSheppStateSpace reedsSheppPath(params_.min_turning_radius);
        State *rsStart = (State *)reedsSheppPath.allocState();
        State *rsEnd = (State *)reedsSheppPath.allocState();
        rsStart->setXY(x, y);
        rsStart->setYaw(theta);
        const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
        while (x < map_width_)
        {
            y = 0;
            theta = 0;
            while (y < map_height_)
            {
                theta = 0;
                if (x == 0 && y == 0)
                {
                    continue;
                }
                while (theta < 2 * M_PI)
                {
                    int index = CalculateNode3DIndex(x, y, theta);
                    rsEnd->setXY(x, y);
                    rsEnd->setYaw(theta);
                    cost = reedsSheppPath.distance(rsStart, rsEnd);
                    reeds_shepp_lookup_.emplace(index, cost);
                    DLOG(INFO) << "point is " << x << " " << y << " " << theta << " index is " << index << " cost is " << cost;
                    theta += delta_heading_in_rad;
                }
                y += 1.0f / params_.position_resolution;
            }
            x += 1.0f / params_.position_resolution;
        }
        // DLOG(INFO) << "CalculateRSLookup done.";
    }

    void LookupTable::CalculateCubicBezierLookupV1()
    {
        float x = 0, y = 0, theta = 0, cost = 0;
        Bezier::Point start(x, y), goal;
        // control_points_vec_.clear();
        float start_angle = theta;
        const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
        Bezier::Point first_control_point, second_control_point, direction;
        while (x < map_width_)
        {
            y = 0;
            theta = 0;
            while (y < map_height_)
            {
                theta = 0;
                if (x == 0 && y == 0)
                {
                    continue;
                }
                while (theta < 2 * M_PI)
                {
                    int index = CalculateNode3DIndex(x, y, theta);
                    Bezier::Point goal(x, y);
                    float goal_angle = theta;
                    direction.x = std::cos(start_angle);
                    direction.y = std::sin(start_angle);
                    float t = (goal - start).length() / 3;
                    first_control_point = start + direction * t;
                    direction.x = std::cos(goal_angle);
                    direction.y = std::sin(goal_angle);
                    second_control_point = goal - direction * t;
                    Bezier::Bezier<3> cubic_bezier({start, first_control_point, second_control_point, goal});

                    cost = cubic_bezier.length();
                    // float max_curvature = cubic_bezier.GetMaxCurvature();
                    // if (max_curvature > params_.min_turning_radius)
                    // {
                    //     cost = 100000;
                    // }
                    cubic_bezier_lookup_.emplace(index, cost);
                    DLOG(INFO) << "point is " << x << " " << y << " " << theta << " index is " << index << " cost is " << cost;
                    theta += delta_heading_in_rad;
                }
                y += 1.0f / params_.position_resolution;
            }
            x += 1.0f / params_.position_resolution;
        }
        // DLOG(INFO) << "CalculateCubicBezier done.";
    }
    void LookupTable::CalculateCubicBezierLookup()
    {
        //since cubic bezier has no limit to its curvature, we have to make the cost to inf when curvature greater then the limit
        float x = 0, y = 0, theta = 0, cost = 0;
        Eigen::Vector3f vector3d_start(x, y, theta);
        const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
        while (x < map_width_)
        {
            y = 0;
            theta = 0;
            while (y < map_height_)
            {
                theta = 0;
                if (x == 0 && y == 0)
                {
                    continue;
                }
                while (theta < 2 * M_PI)
                {
                    int index = CalculateNode3DIndex(x, y, theta);
                    Eigen::Vector3f vector3d_goal(x, y, theta);
                    CubicBezier::CubicBezier cubic_bezier(vector3d_start, vector3d_goal, map_width_, map_height_);
                    cost = cubic_bezier.GetLength();
                    float max_curvature = cubic_bezier.GetMaxCurvature();
                    if (max_curvature > params_.min_turning_radius)
                    {
                        cost = 100000;
                    }
                    cubic_bezier_lookup_.emplace(index, cost);
                    // DLOG(INFO) << "point is " << x << " " << y << " " << theta << " index is " << index << " cost is " << cost;
                    theta += delta_heading_in_rad;
                }
                y += 1.0f / params_.position_resolution;
            }
            x += 1.0f / params_.position_resolution;
        }
        // DLOG(INFO) << "CalculateCubicBezier done.";
    }
}