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

        std::string log_dir = "/home/jialiang/Code/thesis_ws/src/hybrid_astar/log/hybrid_astar_";
        for (int severity = 0; severity < google::NUM_SEVERITIES; ++severity)
        {
            google::SetLogDestination(severity, log_dir.c_str());
            google::SetLogSymlink(severity, log_dir.c_str());
        }
        google::InitGoogleLogging("hybrid_a_star");

        google::InstallFailureSignalHandler();

        google::EnableLogCleaner(5);
        FLAGS_alsologtostderr = 1;
        LOG(INFO) << "creating the hybrid Astar planner";
    }

    HybridAStarPlanner::~HybridAStarPlanner()
    {
        delete _planner;
    }
    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {

            initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
            // LOG(INFO) << "frame id is " << costmap_ros->getGlobalFrameID();
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
        visualization_ptr_.reset(new Visualize());
        configuration_space_ptr_.reset(new CollisionDetection(params_.GetCollisionDetectionParams(), _costmap));
        nav_msgs::OccupancyGrid::Ptr map;
        map.reset(new nav_msgs::OccupancyGrid());
        Utility::TypeConversion(_costmap, frame_id, map);

        LOG(INFO) << "initializing the hybrid Astar planner";
        costmap = _costmap;
        frame_id_ = frame_id;
        //  初始化发布路径的主题
        plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
        path_vehicles_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);
        make_plan_srv_ = nh.advertiseService("make_plan", &HybridAStarPlanner::makePlanService, this);

        if (!params_.param_container_ptr_->planner_params.use_a_star)
        {
            LOG(INFO) << "Using hybrid_astar mode!";
            configuration_space_ptr_->UpdateGrid(map, true);
            _planner = new HybridAStar(frame_id_, costmap, params_.GetHybridAStarParams(), visualization_ptr_, configuration_space_ptr_);
        }
        else
        {
            LOG(INFO) << "Using Astar mode!";
            configuration_space_ptr_->UpdateGrid(map, false);
            _planner = new AStar(frame_id_, costmap, params_.GetAStarPlannerParams(), visualization_ptr_, configuration_space_ptr_);
        }

    } // end of constructor function HybridAStarPlanner

    bool HybridAStarPlanner::makePlanService(nav_msgs::GetPlan::Request &req,
                                             nav_msgs::GetPlan::Response &resp)
    {
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time();
        resp.plan.header.frame_id = frame_id_;
        // LOG(INFO) << "in makePlanService";
        return true;
    }

    bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            DLOG(ERROR) << "The planner has not been initialized, please call initialize() to use the planner";
            return false;
        }
        Clear(plan);
        LOG(INFO) << "Got a start: " << start.pose.position.x << " " << start.pose.position.y << " " << Utility::ConvertRadToDeg(start.pose.position.z) << " and a goal: " << goal.pose.position.x << " " << goal.pose.position.y << " " << Utility::ConvertRadToDeg(goal.pose.position.z);

        // 检查设定的目标点参数是否合规
        if (!(checkStartPose(start) && checkgoalPose(goal)))
        {
            LOG(WARNING) << "Failed to create a global plan due to start or goal not available!";
            return false;
        }
        // 正式将参数传入规划器中
        if (!_planner->calculatePath(start, goal, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), plan, path_vehicles_pub_, pathNodes))
        {
            LOG(INFO) << "failed to find a path!!!";
            return false;
        }

        publishPlan(plan);
        return true;
    }

    bool HybridAStarPlanner::checkStartPose(const geometry_msgs::PoseStamped &start)
    {
        if (configuration_space_ptr_->IsTraversable(start))
        {
            return true;
        }
        LOG(WARNING) << "The Start pose is out of the map!";
        return false;
    } // end of checkStartPose

    bool HybridAStarPlanner::checkgoalPose(const geometry_msgs::PoseStamped &goal)
    {
        unsigned int goalx, goaly;
        if (costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly))
        {
            if (configuration_space_ptr_->IsTraversable(goal))
            {
                if (costmap->getCost(goalx, goaly) > 252)
                {

                    LOG(WARNING) << "The Goal pose is out of the map! %d", costmap->getCost(goalx, goaly);
                    LOG(WARNING) << "The Goal pose is occupied , please reset the goal!";
                    return false;
                }
            }

            return true;
        }
        return false;
    } // end of checkgoalPose

    void HybridAStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!initialized_)
        {
            LOG(ERROR) << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
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
            transform_path.pose.orientation = path[i].pose.orientation;
            gui_path.poses[i] = transform_path; //
        }

        plan_pub_.publish(gui_path);
        // LOG(INFO) << ("Publish the path to Rviz");
    } // end of publishPlan
    void HybridAStarPlanner::Clear(std::vector<geometry_msgs::PoseStamped> &plan)
    {
        plan.clear();
        ros::Duration d(1);
        d.sleep();
        // CLEAR THE VISUALIZATION
        visualization_ptr_->clear();
        publishPlan(plan);
    }
};