/**
 * @file path_evaluator.cpp
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.3
 * @date 2021-12-17
 *
 * @copyright Copyright (c) 2021
 *
 **/
#include "path_evaluator.h"

namespace PathEvaluator
{
    int PathEvaluator::CalculateCurvature(const std::vector<Eigen::Vector3f> &path, const std::string &topic_name)
    {
        if (path.size() < 3)
        {
            // DLOG(WARNING) << "In CalculateCurvature: path does not have enough points!!!";
            return 0;
        }

        std::vector<float> curvature_vec;
        float curvature;
        // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " path size is :" << path.size();

        // use three points to calculate curvature;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            //get three points from path
            Eigen::Vector2f xp(path[i].x(), path[i].y());
            // DLOG(INFO) << "xp x is :" << xp(0,0) << "y is: " << xp.y();
            Eigen::Vector2f xi(path[i + 1].x(), path[i + 1].y());
            // DLOG(INFO) << "xi x is :" << xi(0,0) << "y is: " << xi.y();
            Eigen::Vector2f xs(path[i + 2].x(), path[i + 2].y());

            curvature = Utility::CalculateCurvature(xp, xi, xs);
            curvature_vec.emplace_back(curvature);
            // DLOG(INFO) << "In CalculateCurvature:" << i << "th curvature is:" << curvature;
        }
        if (curvature_map_.count(topic_name) > 0)
        {
            curvature_map_.at(topic_name).clear();
            curvature_map_.at(topic_name) = curvature_vec;
            // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " is already in curvature map, clear vector and put new curvature into vector.";
        }
        else
        {
            curvature_map_.insert({topic_name, curvature_vec});
            // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " is not in the curvature map, insert into the map.";
        }
        return 1;
    }

    int PathEvaluator::CalculateSmoothness(const std::vector<Eigen::Vector3f> &path, const std::string &topic_name)
    {
        if (path.size() < 3)
        {
            // DLOG(WARNING) << "In CalculateSmoothness: path does not have enough points!!!";
            return 0;
        }
        // smoothness = (deltax(i+1)-delta(xi))^2
        // deltax(i+1)= x(i+1)-x(i), the same for deltaxi
        std::vector<float> smoothness_vec;
        float smoothness;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            //get three points from path
            Eigen::Vector2f xp(path[i].x(), path[i].y());
            Eigen::Vector2f xi(path[i + 1].x(), path[i + 1].y());
            Eigen::Vector2f xs(path[i + 2].x(), path[i + 2].y());
            if (xp == xi || xi == xs)
            {
                DLOG(WARNING) << "In CalculateSmoothness: some points are equal, skip these points for curvature calculation!!";
                continue;
            }
            //get two vector between these three nodes
            Eigen::Vector2f pre_vector = xi - xp;
            Eigen::Vector2f succ_vector = xs - xi;

            smoothness = (succ_vector - pre_vector).norm() * (succ_vector - pre_vector).norm();
            smoothness_vec.emplace_back(smoothness);
        }
        if (smoothness_map_.count(topic_name) > 0)
        {
            smoothness_map_.at(topic_name).clear();
            smoothness_map_.at(topic_name) = smoothness_vec;
            // DLOG(INFO) << "In CalculateSmoothness:" << topic_name << " is already in smoothness map, clear vector and put new curvature into vector.";
        }
        else
        {
            smoothness_map_.insert({topic_name, smoothness_vec});
            // DLOG(INFO) << "In CalculateSmoothness:" << topic_name << " is not in the smoothness map, insert into the map.";
        }
        return 1;
    }

    int PathEvaluator::CalculateSteeringAngle(const std::vector<Eigen::Vector3f> &path, const std::string &topic_name)
    {
        if (path.size() < 3)
        {
            // DLOG(WARNING) << "In CalculateSteeringAngle: path does not have enough points!!!";
            return 0;
        }
        // steering angle = angle between x[i+1] and x[i] minus angle between x[i-1] and x[i]
        std::vector<float> steering_angle_vec;
        float steering_angle;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            HybridAStar::Node3D current, next;
            Utility::TypeConversion(path[i], current);
            Utility::TypeConversion(path[i + 1], next);

            steering_angle = Utility::ConvertRadToDeg(Utility::FindSteeringAngle(current, next));
            // LOG(INFO) << i << "th steering angle is " << steering_angle << " current node is " << current.getX() << " " << current.getY() << " " << Utility::ConvertRadToDeg(current.getT()) << " next is " << next.getX() << " " << next.getY() << " " << Utility::ConvertRadToDeg(next.getT());
            steering_angle_vec.emplace_back(steering_angle);
        }
        if (steering_angle_map_.count(topic_name) > 0)
        {
            steering_angle_map_.at(topic_name).clear();
            steering_angle_map_.at(topic_name) = steering_angle_vec;
            // DLOG(INFO) << "In CalculateSteeringAngle:" << topic_name << " is already in steering_angle map, clear vector and put new curvature into vector.";
        }
        else
        {
            steering_angle_map_.insert({topic_name, steering_angle_vec});
            // DLOG(INFO) << "In CalculateSteeringAngle:" << topic_name << " is not in the steering_angle map, insert into the map.";
        }
        return 1;
    }

    int PathEvaluator::CalculateClearance(const std::vector<Eigen::Vector3f> &path, const std::string &topic_name)
    {
        if (path.size() < 1)
        {
            // DLOG(WARNING) << "In CalculateClearance: path does not have enough points!!!";
            return 0;
        }
        // for clearance, node2d is enough, here clearance is the distance for current point to nearest obstacle.
        std::vector<float> clearance_vec;
        float clearance = INFINITY;
        int map_width = map_->info.width;
        int map_height = map_->info.height;
        float map_resolution = map_->info.resolution;
        for (const auto &vector_3d : path)
        {
            //naive algorithm, time complexity is n^2.
            for (int index = 0; index < map_height * map_width; ++index)
            {
                if (map_->data[index])
                {
                    Eigen::Vector2f obstacle_2d = Utility::ConvertIndexToEigenVector2f(index, map_width, map_resolution);
                    Utility::Polygon obstacle = Utility::CreatePolygon(obstacle_2d, map_resolution, map_resolution);
                    Eigen::Vector2f vehicle_2d;
                    Utility::TypeConversion(vector_3d, vehicle_2d);
                    Utility::Polygon vehicle = Utility::CreatePolygon(vehicle_2d, vehicle_width_, vehicle_length_, vector_3d.z());
                    float distance = Utility::GetDistanceFromPolygonToPolygon(vehicle, obstacle);
                    if (distance < clearance)
                    {
                        clearance = distance;
                    }
                    LOG(INFO) << "In CalculateClearance: current index: " << index << " converted x: " << obstacle_2d(0, 0) << " converted y: " << obstacle_2d.y() << " current path location x is: " << vector_3d(0, 0) << " y:" << vector_3d.y() << " distance is: " << distance << " clearance is: " << clearance;
                }
            }
            clearance_vec.emplace_back(clearance);
            clearance = INFINITY;
        }
        if (clearance_map_.count(topic_name) > 0)
        {
            clearance_map_.at(topic_name).clear();
            clearance_map_.at(topic_name) = clearance_vec;
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            clearance_map_.insert({topic_name, clearance_vec});
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is not in the clearance map, insert into the map.";
        }
        return 1;
    }
    void PathEvaluator::CallbackSetMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
    {
        map_ = map;
    }

    void PathEvaluator::CallbackPath(const nav_msgs::Path::ConstPtr &path, const std::string &topic_name)
    {
        std::vector<Eigen::Vector3f> vector_3d_vec;
        Utility::TypeConversion(path, vector_3d_vec);
        CalculateCurvature(vector_3d_vec, topic_name);
        CalculateSmoothness(vector_3d_vec, topic_name);
        CalculateClearance(vector_3d_vec, topic_name);
        CalculateSteeringAngle(vector_3d_vec, topic_name);
    }

    void PathEvaluator::Plot()
    {
        matplotlibcpp::ion();
        matplotlibcpp::clf();
        std::vector<std::string> title_vec = {"curvature", "smoothness", "clearance", "steering_angle"};
        for (size_t i = 0; i < title_vec.size(); i++)
        {
            matplotlibcpp::subplot(2, 2, i + 1);
            std::unordered_map<std::string, std::vector<float>> map;
            if (title_vec[i] == "curvature")
            {
                map = curvature_map_;
            }
            if (title_vec[i] == "smoothness")
            {
                map = smoothness_map_;
            }
            if (title_vec[i] == "clearance")
            {
                map = clearance_map_;
            }
            if (title_vec[i] == "steering_angle")
            {
                map = steering_angle_map_;
            }

            for (const auto &curvature_vec : map)
            {
                if (curvature_vec.first == path_topic_)
                {
                    matplotlibcpp::plot(curvature_vec.second, {{"label", "raw path"}});
                }
                else
                {
                    matplotlibcpp::plot(curvature_vec.second, {{"label", "smoothed path"}});
                }

                matplotlibcpp::legend({{"loc", "upper right"}});
                // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;
            }
            matplotlibcpp::title(title_vec[i]);
            matplotlibcpp::ylabel(title_vec[i]);
            // matplotlibcpp::ylim(0, 1);
            matplotlibcpp::grid(true);
        }

        matplotlibcpp::pause(0.1);
    }
    }
