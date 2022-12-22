#include <pluginlib/class_list_macros.h>
#include "hybrid_a_star_planner.h"
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ros/node_handle.h>
#include "a_star.h"
#include "hybrid_a_star.h"
// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(HybridAStar::HybridAStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

// Default Constructor
namespace HybridAStar
{

    HybridAStarPlanner::HybridAStarPlanner() : initialized_(false), costmap(NULL), resolution(1.0), hybrid_a_star_ptr_(NULL)
    {
        DLOG(INFO) << "creating the hybrid Astar planner";
    }

    HybridAStarPlanner::~HybridAStarPlanner()
    {
    }
    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {

            initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());

            initialized_ = true;
        }
        else
            DLOG(WARNING) << "This planner has already been initialized... doing nothing";
    }

    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2D *_costmap, std::string frame_id)
    {
        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle nh("~/" + name);
        // get parameters of TebConfig via the nodehandle and override the default config
        params_.loadRosParamFromNodeHandle(nh);
        visualization_ptr_.reset(new Visualize(params_.GetVisualizeParams()));

        DLOG(INFO) << "initializing the hybrid Astar planner";
        if (!params_.param_container_ptr_->planner_params.use_a_star)
        {
            DLOG(INFO) << "Using hybrid_astar mode!";
        }
        else
        {
            DLOG(INFO) << "Using Astar mode!";
        }
        costmap = _costmap;
        frame_id_ = frame_id;
        // DLOG(INFO) << frame_id;
        //  初始化发布路径的主题
        plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
        path_vehicles_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);
        make_plan_srv_ = nh.advertiseService("make_plan", &HybridAStarPlanner::makePlanService, this);

    } // end of constructor function HybridAStarPlanner

    bool HybridAStarPlanner::makePlanService(nav_msgs::GetPlan::Request &req,
                                             nav_msgs::GetPlan::Response &resp)
    {
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;
        return true;
    }

    bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            DLOG(ERROR) << "The planner has not been initialized, please call initialize() to use the planner";
            return false;
        }

        DLOG(INFO) << "Got a start: " << start.pose.position.x << " " << start.pose.position.y << " and a goal: " << goal.pose.position.x << " " << goal.pose.position.y;
        plan.clear();

        Expander *_planner;
        if (!params_.param_container_ptr_->planner_params.use_a_star)
        {
            DLOG(INFO) << "Using hybrid_astar mode!";
            // DLOG(INFO) << "params in hybrid astar is " << params_.GetHybridAStarParams().adaptive_steering_angle_and_step_size;
            _planner = new HybridAStar(frame_id_, costmap, params_.GetHybridAStarParams(), visualization_ptr_);
        }
        else
        {
            DLOG(INFO) << "Using Astar mode!";
            _planner = new AStar(frame_id_, costmap);
        }

        // 检查设定的目标点参数是否合规
        if (!(checkStartPose(start) && checkgoalPose(goal)))
        {
            DLOG(WARNING) << "Failed to create a global plan due to start or goal not available!";
            return false;
        }
        plan.clear();
        // 正式将参数传入规划器中
        if (!_planner->calculatePath(start, goal, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), plan, path_vehicles_pub_, pathNodes))
        {
            DLOG(INFO) << "failed to find a path!!!";
            return false;
        }

        // 参数后期处理，发布到RViz上进行可视化
        clearPathNodes();
        // path只能发布2D的节点
        publishPlan(plan);
        publishPathNodes(plan);
        return true;
    }

    bool HybridAStarPlanner::checkStartPose(const geometry_msgs::PoseStamped &start)
    {
        unsigned int startx, starty;
        if (costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty))
        {
            return true;
        }
        DLOG(WARNING) << "The Start pose is out of the map!";
        return false;
    } // end of checkStartPose

    bool HybridAStarPlanner::checkgoalPose(const geometry_msgs::PoseStamped &goal)
    {
        unsigned int goalx, goaly;
        if (costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly))
        {
            if (costmap->getCost(goalx, goaly) > 252)
            {

                DLOG(WARNING) << "The Goal pose is out of the map! %d", costmap->getCost(goalx, goaly);
                DLOG(WARNING) << "The Goal pose is occupied , please reset the goal!";
                return false;
            }
            return true;
        }
        return false;
    } // end of checkgoalPose

    void HybridAStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!initialized_)
        {
            DLOG(ERROR) << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
            return;
        }
        // create a message for the plan
        geometry_msgs::PoseStamped transform_path;
        nav_msgs::Path gui_path;
        int size = path.size();
        gui_path.poses.resize(size);

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (int i = 0; i < size; i++)
        {
            transform_path.pose.position = path[i].pose.position;
            gui_path.poses[i] = transform_path; //
        }

        plan_pub_.publish(gui_path);
        // LOG(INFO)<<("Publish the path to Rviz");

    } // end of publishPlan

    void HybridAStarPlanner::publishPathNodes(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!initialized_)
        {
            DLOG(ERROR) << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
            return;
        }
        visualization_msgs::Marker pathVehicle;
        int nodeSize = path.size();
        pathVehicle.header.stamp = ros::Time(0);
        pathVehicle.color.r = 52.f / 255.f;
        pathVehicle.color.g = 250.f / 255.f;
        pathVehicle.color.b = 52.f / 255.f;
        pathVehicle.type = visualization_msgs::Marker::ARROW;
        pathVehicle.header.frame_id = frame_id_;
        pathVehicle.scale.x = 0.22;
        pathVehicle.scale.y = 0.18;
        pathVehicle.scale.z = 0.12;
        pathVehicle.color.a = 0.1;
        // 转化节点，并同时加上时间戳等信息
        for (int i = 0; i < nodeSize; i++)
        {
            pathVehicle.header.stamp = ros::Time(0);
            pathVehicle.pose = path[i].pose;
            pathVehicle.id = i;
            pathNodes.markers.push_back(pathVehicle);
        }
        // 发布这些车辆位置标记点
        path_vehicles_pub_.publish(pathNodes);

    } // end of publishPathNodes

    void HybridAStarPlanner::clearPathNodes()
    {
        // 初始化并配置节点为全清空模式
        visualization_msgs::Marker node;
        pathNodes.markers.clear();
        node.action = visualization_msgs::Marker::DELETEALL;
        node.header.frame_id = frame_id_;
        node.header.stamp = ros::Time(0);
        node.id = 0;
        node.action = 3;
        pathNodes.markers.push_back(node);
        path_vehicles_pub_.publish(pathNodes);
        // LOG(INFO)<<("Clean the path nodes");
    }
};