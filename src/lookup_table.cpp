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
        LOG(INFO) << "lookup table initializing";
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
        std::unique_lock<std::mutex> lock(map_access_);
        dubins_lookup_.clear();
        reeds_shepp_lookup_.clear();
        cubic_bezier_lookup_.clear();
    }

    float LookupTable::GetDubinsCost(const Node3D &node3d)
    {
        std::unique_lock<std::mutex> lock(map_access_);
        float out = 1000000;
        int index = CalculateNode3DIndex(node3d);
        if (dubins_lookup_.find(index) != dubins_lookup_.end())
        {
            out = dubins_lookup_.at(index);
        }
        else
        {
            if (node3d.getX() < params_.position_resolution && node3d.getY() < params_.position_resolution)
            {
                // DLOG(INFO) << "index : " << index << " not found in dubins map, node is " << node3d.getX() << " " << node3d.getY() << " " << Utility::ConvertRadToDeg(node3d.getT());
            }
            else
            {
                // DLOG(WARNING) << "Warning: index : " << index << " not found in dubins map, node is " << node3d.getX() << " " << node3d.getY() << " " << Utility::ConvertRadToDeg(node3d.getT());
            }

            // const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
            // DLOG(INFO) << "(int)(theta / delta_heading_in_rad) " << (int)(Utility::RadToZeroTo2P(node3d.getT()) / delta_heading_in_rad);
        }
        return out;
    }
    float
    LookupTable::GetReedsSheppCost(const Node3D &node3d)
    {
        std::unique_lock<std::mutex> lock(map_access_);
        float out = 100000000;
        int index = CalculateNode3DIndex(node3d);
        if (reeds_shepp_lookup_.find(index) != reeds_shepp_lookup_.end())
        {
            out = reeds_shepp_lookup_.at(index);
        }
        else
        {
            // DLOG(INFO) << "index not found in rs map, node is " << node3d.getX() << " " << node3d.getY() << " " << node3d.getT();
        }
        return out;
    }
    float LookupTable::GetCubicBezierCost(const Node3D &node3d)
    {
        std::unique_lock<std::mutex> lock(map_access_);
        // if index is not found, cost should be set to a very large number not zero
        float out = 10000000;
        int index = CalculateNode3DIndex(node3d);
        if (cubic_bezier_lookup_.find(index) != cubic_bezier_lookup_.end())
        {
            out = cubic_bezier_lookup_.at(index);
        }
        else
        {
            // DLOG(INFO) << "index " << index << " not found in cubic bezier map, node is " << node3d.getX() << " " << node3d.getY() << " " << node3d.getT();
        }
        return out;
    }
    int LookupTable::CalculateNode3DIndex(const Node3D &node3d) const
    {
        int out;
        float x_float = node3d.getX();
        float y_float = node3d.getY();
        float t_float = Utility::RadToZeroTo2P(node3d.getT());
        // DLOG(INFO) << "t is " << t_float;
        out = CalculateNode3DIndex(x_float, y_float, t_float);
        return out;
    }
    int LookupTable::CalculateNode3DIndex(const float &x, const float &y, const float &theta) const
    {
        // DLOG(INFO) << "x is " << x << " y is " << y << " theta is " << theta;
        int out, x_index, y_index;
        const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
        float angle;
        if (abs(theta) < 1e-4)
        {
            angle = 0;
        }
        else
        {
            angle = theta;
        }
        // DLOG(INFO) << "angle is " << angle;
        // DLOG(INFO) << "delta heading in rad is " << delta_heading_in_rad;
        // DLOG(INFO) << "(int)(theta / delta_heading_in_rad) " << (int)(theta / delta_heading_in_rad);
        x_index = (int)(x * params_.position_resolution);
        y_index = (int)(y * params_.position_resolution);

        out = (int)(angle / delta_heading_in_rad) * map_width_ * map_height_ * params_.position_resolution * params_.position_resolution + (int)(y_index * params_.position_resolution) * map_width_ + (int)(x_index);
        // DLOG(INFO) << "index is " << out;
        return out;
    }
    void LookupTable::CalculateDubinsLookup()
    {
        std::unique_lock<std::mutex> lock(map_access_);
        // DLOG(INFO) << "CalculateDubinsLookup start:";
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
                    y += 1.0f / params_.position_resolution;
                    continue;
                }
                while (theta < 2 * M_PI)
                {
                    dbEnd->setXY(x, y);
                    dbEnd->setYaw(theta);
                    cost = dubinsPath.distance(dbStart, dbEnd);
                    int index = CalculateNode3DIndex(x, y, theta);
                    dubins_lookup_.emplace(index, cost);
                    // DLOG(INFO) << "point is " << x << " " << y << " " << Utility::ConvertRadToDeg(theta) << " index is " << index << " cost is " << cost;
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
        std::unique_lock<std::mutex> lock(map_access_);
        // DLOG(INFO) << "CalculateRSLookup start.";
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
                    y += 1.0f / params_.position_resolution;
                    continue;
                }
                while (theta < 2 * M_PI)
                {
                    int index = CalculateNode3DIndex(x, y, theta);
                    rsEnd->setXY(x, y);
                    rsEnd->setYaw(theta);
                    cost = reedsSheppPath.distance(rsStart, rsEnd);
                    reeds_shepp_lookup_.emplace(index, cost);
                    // DLOG(INFO) << "point is " << x << " " << y << " " << theta << " index is " << index << " cost is " << cost;
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
        std::unique_lock<std::mutex> lock(map_access_);
        // DLOG(INFO) << "CalculateCubicBezierV1 start.";
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
                    y += 1.0f / params_.position_resolution;
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
                    // DLOG(INFO) << "point is " << x << " " << y << " " << theta << " index is " << index << " cost is " << cost;
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
        std::unique_lock<std::mutex> lock(map_access_);
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