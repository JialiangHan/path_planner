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
#ifndef PATH_EVALUATOR_H
#define PATH_EVALUATOR_H
#include <nav_msgs/Path.h>
#include <vector>
#include "vector2d.h"
#include "node3d.h"
#include "ros/ros.h"
#include <unordered_map>
#include "glog/logging.h"
#include "gflags/gflags.h"
namespace PathEvaluator
{
    class PathEvaluator
    {
    public:
        PathEvaluator(){};
        PathEvaluator(const std::string &path_topic, const std::string &smoothed_path_topic)
        {
            sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic, 1, boost::bind(&PathEvaluator::CallbackPath, this, _1, path_topic));
            sub_smoothed_path_ = nh_.subscribe<nav_msgs::Path>(smoothed_path_topic, 1, boost::bind(&PathEvaluator::CallbackPath, this, _1, smoothed_path_topic));
        };
        void CallbackPath(const nav_msgs::PathConstPtr path, const std::string &topic_name);

        void CallbackSmoothedPath(const nav_msgs::PathConstPtr smoothed_path);
        void ConvertRosPathToVectorNode3D(const nav_msgs::PathConstPtr path, std::vector<HybridAStar::Node3D> &node_3d_vec);
        /**
         * @brief calculate curvature for the path 
         * 
         * @param path got from planner
         * @return std::vector<float> 
         */
        int CalculateCurvature(const std::vector<HybridAStar::Node3D> &path, const std::string &topic_name);

        // int CalculateClearance(const std::vector<Node3D> &path, some kind of map);

        int CalculateSmoothness(const std::vector<HybridAStar::Node3D> &path, const std::string &topic_name);
        /**
         * @brief plot all the metrics for the path.
         * 
         * @return int 
         */
        void Plot();

    private:
        ros::NodeHandle nh_;

        ros::Subscriber sub_path_;

        ros::Subscriber sub_smoothed_path_;
        /**
         * @brief key is topic name for all three maps; 
         * 
         */
        std::unordered_map<std::string, std::vector<float>> clearance_map_;

        std::unordered_map<std::string, std::vector<float>> curvature_map_;

        std::unordered_map<std::string, std::vector<float>> smoothness_map_;

        // /some kind of map is need for the clearacne
    };
}

#endif