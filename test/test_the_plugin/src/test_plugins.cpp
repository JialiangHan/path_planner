/*********************************************************************
 *
 *  BSD 3-Clause License
 *
 *  Copyright (c) 2021, dengpw
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1 Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2 Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   3 Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <iostream>
#include <tf/tf.h>
#include <pluginlib/class_loader.h>
#include <pluginlib_tutorials/polygon_base.h>
#include "test_plugins.h"

TestPlanner::TestPlanner(tf2_ros::Buffer &_tf) : tf(_tf)
{
    // 订阅目标主题，绑定响应函数,这里使用suscribe订阅目标点，当目标点刷新就重新进行路径规划
    make_plane = n.subscribe("/move_base_simple/goal", 1, &TestPlanner::setgoal, this);
    // 定义类插件的名称，以便之后接入系统
    global_planner = std::string("HybridAStar/HybridAStarPlanner");
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"); // 导入插件
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    // 需要注意的是，这里的初始化函数Costmap2DROS在构造时候的问题，~代表的是私有域的数据
    costmap = new costmap_2d::Costmap2DROS("global_costmap", tf);

    LOG(INFO) << "creat the global costmap";
    // 指定costmap中的base_link为起始坐标
    robot_pose.header.frame_id = "base_link";

    transformStartPose();
    try
    {
        planner_ = bgp_loader_.createInstance(global_planner);
        planner_->initialize(bgp_loader_.getName(global_planner), costmap);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
        LOG(FATAL) << "Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what();
        exit(1);
    }
}
void TestPlanner::setgoal(const geometry_msgs::PoseStamped::ConstPtr &_goal)
{
    goal_pose.pose = _goal->pose;
    goal_pose.pose.orientation = _goal->pose.orientation;
    goal_pose.header = _goal->header;
    transformStartPose();
    costmap->start();
    if (true)
    {
        tf::Quaternion pose_quat = tf::createQuaternionFromYaw(0);
        // note: unit for pose is meter
        start_pose.pose.position.x = 2 * costmap->getCostmap()->getResolution() + costmap->getCostmap()->getOriginX();
        start_pose.pose.position.y = costmap->getCostmap()->getSizeInCellsY() * costmap->getCostmap()->getResolution() - 2 * costmap->getCostmap()->getResolution();
        start_pose.pose.position.z = 0;
        start_pose.pose.orientation.x = pose_quat.x();
        start_pose.pose.orientation.y = pose_quat.y();
        start_pose.pose.orientation.z = pose_quat.z();
        start_pose.pose.orientation.w = pose_quat.w();
        goal_pose.pose.position.x = costmap->getCostmap()->getSizeInCellsX() * costmap->getCostmap()->getResolution() - 2 * costmap->getCostmap()->getResolution();
        goal_pose.pose.position.y = 2 * costmap->getCostmap()->getResolution() + costmap->getCostmap()->getOriginY();
        goal_pose.pose.position.z = 0;
        goal_pose.pose.orientation.x = pose_quat.x();
        goal_pose.pose.orientation.y = pose_quat.y();
        goal_pose.pose.orientation.z = pose_quat.z();
        goal_pose.pose.orientation.w = pose_quat.w();
    }
    planner_->makePlan(start_pose, goal_pose, *planner_plan_);
}

bool TestPlanner::transformStartPose(void)
{
    try
    {
        start_transform = tf.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    start_pose.pose.position.x = start_transform.transform.translation.x;
    start_pose.pose.position.y = start_transform.transform.translation.y;
    start_pose.pose.position.z = start_transform.transform.translation.z;
    start_pose.pose.orientation.w = start_transform.transform.rotation.w;
    start_pose.pose.orientation.x = start_transform.transform.rotation.x;
    start_pose.pose.orientation.y = start_transform.transform.rotation.y;
    start_pose.pose.orientation.z = start_transform.transform.rotation.z;
    return true;
}
TestPlanner::~TestPlanner()
{
    planner_.reset();
    if (costmap)
    {
        delete costmap;
    }
    if (planner_plan_)
    {
        delete planner_plan_;
    }
}