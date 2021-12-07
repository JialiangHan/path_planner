/**
 * @file path_evaluator.cpp
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.1
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021
 * 
 **/
#include "path_evaluator.h"
#include <cmath>
#include "matplotlibcpp.h"

namespace PathEvaluator
{
    int PathEvaluator::CalculateCurvature(const std::vector<HybridAStar::Node3D> &path, const std::string &topic_name)
    {
        if (path.size() < 3)
        {
            DLOG(INFO) << "In CalculateCurvature: path does not have enough points!!!";
            return 0;
        }

        std::vector<float> curvature_vec;
        float curvature;
        DLOG(INFO) << "In CalculateCurvature: " << topic_name << " path size is :" << path.size();

        // use three points to calculate curvature;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            //get three points from path
            HybridAStar::Vector2D xp(path[i].getX(), path[i].getY());
            // DLOG(INFO) << "xp x is :" << xp.getX() << "y is: " << xp.getY();
            HybridAStar::Vector2D xi(path[i + 1].getX(), path[i + 1].getY());
            // DLOG(INFO) << "xi x is :" << xi.getX() << "y is: " << xi.getY();
            HybridAStar::Vector2D xs(path[i + 2].getX(), path[i + 2].getY());
            if (xp == xi || xi == xs)
            {
                DLOG(INFO) << "In CalculateCurvature: some points are equal, skip these points for curvature calculation!!";
                continue;
            }
            // DLOG(INFO) << "xs x is :" << xs.getX() << "y is: " << xs.getY();
            //get two vector between these three nodes
            HybridAStar::Vector2D pre_vector = xi - xp;
            // DLOG(INFO) << "pre_vector x is :" << pre_vector.getX() << "y is: " << pre_vector.getY();
            HybridAStar::Vector2D succ_vector = xs - xi;
            // DLOG(INFO) << "succ_vector x is :" << succ_vector.getX() << "y is: " << succ_vector.getY();
            //calculate delta distance and delta angle
            double delta_distance = succ_vector.length();
            double pre_vector_length = pre_vector.length();
            // DLOG(INFO) << "delta_distance is:" << delta_distance;
            // DLOG(INFO) << "pre_vector_length is: " << pre_vector_length;
            // there would some calculation error here causing number inside acos greater than 1 or smaller than -1.
            double temp = pre_vector.dot(succ_vector) / (delta_distance * pre_vector_length);
            if (temp > 1 || temp < -1)
            {
                temp = round(temp);
            }
            double delta_angle = std::acos(temp);
            // DLOG(INFO) << "delta_angle is: " << delta_angle;
            //curvature = abs(delta_angle)/abs(delta_distance)
            curvature = delta_angle/delta_distance;
            curvature_vec.emplace_back(curvature);
            DLOG(INFO) << "In CalculateCurvature:" << i << "th curvature is:" << curvature;
            // if (std::isnan(curvature))
            // {
            //     if (std::isnan(delta_distance) || delta_distance == 0)
            //     {
            //         DLOG(INFO) << "delta_distance is:" << delta_distance;
            //         DLOG(INFO) << "succ_vector x is :" << succ_vector.getX() << "y is: " << succ_vector.getY();
            //         DLOG(INFO) << "xp x is :" << xp.getX() << "y is: " << xp.getY();
            //         DLOG(INFO) << "xi x is :" << xi.getX() << "y is: " << xi.getY();
            //     }
            //     else if (std::isnan(delta_angle))
            //     {
            //         DLOG(INFO) << "inside std::Cos is: " << temp;
            //         DLOG(INFO) << "pre_vector_length is: " << pre_vector_length;
            //         DLOG(INFO) << "xs x is :" << xs.getX() << "y is: " << xs.getY();
            //         DLOG(INFO) << "pre_vector x is :" << pre_vector.getX() << "y is: " << pre_vector.getY();
            //     }
            // }
            // DLOG(INFO) << " in curvature_vec is:" << curvature_vec.back();
        }
        if (curvature_map_.count(topic_name) > 0)
        {
            curvature_map_.at(topic_name).clear();
            curvature_map_.at(topic_name) = curvature_vec;
            DLOG(INFO) << "In CalculateCurvature: " << topic_name << " is already in curvature map, clear vector and put new curvature into vector.";
        }
        else
        {
            curvature_map_.insert({topic_name, curvature_vec});
            DLOG(INFO) << "In CalculateCurvature: " << topic_name << " is not in the curvature map, insert into the map.";
        }
        return 1;
    }

    int PathEvaluator::CalculateSmoothness(const std::vector<HybridAStar::Node3D> &path, const std::string &topic_name)
    {
        if (path.size() < 3)
        {
            DLOG(INFO) << "In CalculateSmoothness: path does not have enough points!!!";
            return 0;
        }
        // smoothness = (deltax(i+1)-delta(xi))^2
        // deltax(i+1)= x(i+1)-x(i), the same for deltaxi
        std::vector<float> smoothness_vec;
        float smoothness;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            //get three points from path
            HybridAStar::Vector2D xp(path[i].getX(), path[i].getY());
            HybridAStar::Vector2D xi(path[i + 1].getX(), path[i + 1].getY());
            HybridAStar::Vector2D xs(path[i + 2].getX(), path[i + 2].getY());
            if (xp == xi || xi == xs)
            {
                DLOG(INFO) << "In CalculateSmoothness: some points are equal, skip these points for curvature calculation!!";
                continue;
            }
            //get two vector between these three nodes
            HybridAStar::Vector2D pre_vector = xi - xp;
            HybridAStar::Vector2D succ_vector = xs - xi;

            smoothness = std::pow((succ_vector - pre_vector).length(), 2);
            smoothness_vec.emplace_back(smoothness);
        }
        if (smoothness_map_.count(topic_name) > 0)
        {
            smoothness_map_.at(topic_name).clear();
            smoothness_map_.at(topic_name) = smoothness_vec;
            DLOG(INFO) << "In CalculateSmoothness:" << topic_name << " is already in smoothness map, clear vector and put new curvature into vector.";
        }
        else
        {
            smoothness_map_.insert({topic_name, smoothness_vec});
            DLOG(INFO) << "In CalculateSmoothness:" << topic_name << " is not in the smoothness map, insert into the map.";
        }
        return 1;
    }

    void PathEvaluator::CallbackPath(const nav_msgs::PathConstPtr path, const std::string &topic_name)
    {
        std::vector<HybridAStar::Node3D> node_3d_vec;
        ConvertRosPathToVectorNode3D(path, node_3d_vec);
        CalculateCurvature(node_3d_vec, topic_name);
        CalculateSmoothness(node_3d_vec, topic_name);
    }

    void PathEvaluator::ConvertRosPathToVectorNode3D(const nav_msgs::PathConstPtr path, std::vector<HybridAStar::Node3D> &node_3d_vec)
    {
        node_3d_vec.clear();
        for (uint i = 0; i < path->poses.size(); ++i)
        {
            HybridAStar::Node3D point;
            point.setX(path->poses[i].pose.position.x);
            point.setY(path->poses[i].pose.position.y);
            //not sure this is correct;
            point.setT(path->poses[i].pose.position.z);
            node_3d_vec.emplace_back(point);
        }
    }

    void PathEvaluator::Plot()
    {
        matplotlibcpp::ion();
        matplotlibcpp::clf();
        matplotlibcpp::subplot(2, 2, 1);
        for (const auto &curvature_vec : curvature_map_)
        {
            if (curvature_vec.first == "/path")
            {
                matplotlibcpp::plot(curvature_vec.second, {{"label", "raw path"}});
            }
            else
            {
                matplotlibcpp::plot(curvature_vec.second, {{"label", "smoothed path"}});
            }

            matplotlibcpp::legend();
            // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;
        }
        matplotlibcpp::title("curvature");
        matplotlibcpp::ylabel("curvature");
        matplotlibcpp::grid(true);

        matplotlibcpp::subplot(2, 2, 2);
        for (const auto &smoothness_vec : smoothness_map_)
        {
            if (smoothness_vec.first == "/path")
            {
                matplotlibcpp::plot(smoothness_vec.second, {{"label", "raw path"}});
            }
            else
            {
                matplotlibcpp::plot(smoothness_vec.second, {{"label", "smoothed path"}});
            }
            matplotlibcpp::legend();
            // DLOG(INFO) << "Plot smoothness for topic: " << smoothness_vec.first;
        }
        matplotlibcpp::title("smoothness");
        matplotlibcpp::ylabel("smoothness");
        matplotlibcpp::grid(true);

        matplotlibcpp::subplot(2, 2, 3);
        for (const auto &clearance_vec : clearance_map_)
        {
            matplotlibcpp::plot(clearance_vec.second, {{"label", clearance_vec.first}});
            matplotlibcpp::legend();
            // DLOG(INFO) << "Plot clearance for topic: " << clearance_vec.first;
        }
        matplotlibcpp::title("clearance");
        matplotlibcpp::ylabel("clearance");
        matplotlibcpp::grid(true);

        matplotlibcpp::pause(0.1);
    }
}