#include "rrt.h"

using namespace HybridAStar;
namespace RRTPlanner
{
    RRTPlanner::RRTPlanner(const ParameterRRTPlanner &params,
                           const std::shared_ptr<Visualize> &visualization_ptr)
    {

        params_ = params;
        configuration_space_ptr_.reset(new CollisionDetection(params_.collision_detection_params));

        visualization_ptr_ = visualization_ptr;
    }

    void RRTPlanner::Initialize(nav_msgs::OccupancyGrid::Ptr map)
    {
        // update the configuration space with the current map
        //  DLOG(INFO) << "hybrid a star initializing";
        configuration_space_ptr_->UpdateGrid(map);
        map_width_ = configuration_space_ptr_->GetMap()->info.width;
        map_height_ = configuration_space_ptr_->GetMap()->info.height;

        // DLOG(INFO) << "hybrid a star initialized done.   ";
    }

    void RRTPlanner::Planning()
    {
        // actually no current node for RRT

        AddNodeToRRT(start_);
        Node3D current;
        int number_of_iterations = 0;
        // VISUALIZATION DELAY
        ros::Duration delay(0.003);
        while (number_of_iterations < params_.max_iterations)
        {
            // goal check
            if (GoalCheck() >= 0)
            {
                break;
            }
            // find successor for current node
            current = GenerateSuccessor();
            AddNodeToRRT(current);
            if (params_.visualization)
            {
                visualization_ptr_->publishNode3DPoses(current);
                visualization_ptr_->publishNode3DPose(current);
                delay.sleep();
            }
            number_of_iterations++;
        }

        TracePath();
    }

    Path3D RRTPlanner::GetPath(const Node3D &start, const Node3D &goal)
    {
        start_ = start;
        goal_ = goal;
        Planning();
        return path_;
    }

    void RRTPlanner::TracePath()
    {
        path_.clear();
        int goal_index = GoalCheck();
        std::shared_ptr<Node3D> node3d_ptr = std::make_shared<Node3D>(rrt_[goal_index]);
        while (node3d_ptr != nullptr)
        {
            path_.emplace_back(*node3d_ptr);
            if (*node3d_ptr == start_)
            {
                break;
            }
            // DLOG(INFO) << "current node is " << node->GetX() << " " << node->GetY() << " and its pred is " << node->GetPred()->GetX() << " " << node->GetPred()->GetY();
            node3d_ptr = node3d_ptr->GetPred();
        }

        std::reverse(path_.begin(), path_.end());
    }

    int RRTPlanner::GoalCheck()
    {
        // goal index, default is -1
        int index = -1;
        if (rrt_.size() == 0)
        {
            DLOG(WARNING) << "size of rrt is zero!!";
            return index;
        }
        // loop all the node in rrt
        for (size_t i = 0; i < rrt_.size(); i++)
        {
            if (Utility::IsCloseEnough(rrt_[i], goal_, params_.goal_range, 2 * M_PI / params_.headings))
            {
                DLOG(INFO) << "Goal reached, index is: " << i;
                index = i;
            }
        }
        return index;
    }

    Node3D RRTPlanner::GenerateSuccessor()
    {
        Node3D successor, random_node, closest_node;
        std::pair<float, float> step_size_steering_angle_pair;
        // 1. find a random node
        random_node = SelectRandomNode();
        // 2. find closet node on rrt to this random node
        closest_node = FindClosestNode(random_node);
        // 3. find step size and steering angle using random node
        step_size_steering_angle_pair = FindStepSizeAndSteeringAngle(closest_node, random_node);
        // 4. create successor using step size and steering angle
        successor = GenerateSuccessor(closest_node, step_size_steering_angle_pair);
        return successor;
    }

    Node3D RRTPlanner::GenerateSuccessor(const Node3D &closest_node, const std::pair<float, float> stepsize_steering_angle)
    {

        float dx, dy, dt, xSucc, ySucc, tSucc, turning_radius, steering_angle;
        std::shared_ptr<Node3D> closest_node_ptr = std::make_shared<Node3D>(closest_node);
        // DLOG_IF(INFO, (pred.GetX() > 76) && (pred.GetX() < 77) && (pred.GetY() > 1) && (pred.GetY() < 2)) << "in create successor, current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT());
        if (stepsize_steering_angle.first <= 1e-3)
        {
            DLOG(INFO) << "current step size is zero, no need to create successor!!";
        }

        steering_angle = stepsize_steering_angle.second;

        turning_radius = stepsize_steering_angle.first / abs(steering_angle);
        dt = steering_angle;
        // DLOG(INFO) << "step size is " << pair.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(steering_angle);
        // forward, checked
        // right
        if (steering_angle < 0)
        {
            dx = turning_radius * sin(abs(steering_angle));
            dy = -(turning_radius * (1 - cos(steering_angle)));
            // DLOG(INFO) << "forward right, dx " << dx << " dy " << dy;
        }
        // left
        else if (steering_angle > 1e-3)
        {
            dx = turning_radius * sin(abs(steering_angle));
            dy = (turning_radius * (1 - cos(steering_angle)));
            // DLOG(INFO) << "forward left, dx " << dx << " dy " << dy;
        }
        // straight forward,checked
        else
        {
            dx = stepsize_steering_angle.first;
            dy = 0;
            // DLOG(INFO) << "forward straight";
        }

        xSucc = closest_node.GetX() + dx * cos(closest_node.GetT()) - dy * sin(closest_node.GetT());
        ySucc = closest_node.GetY() + dx * sin(closest_node.GetT()) + dy * cos(closest_node.GetT());
        tSucc = Utility::RadToZeroTo2P(closest_node.GetT() + dt);
        // DLOG_IF(INFO, (closest_node.GetX() > 76) && (closest_node.GetX() < 77) && (closest_node.GetY() > 1) && (closest_node.GetY() < 2)) << "successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);

        Node3D successor(xSucc, ySucc, tSucc, 0, 0, closest_node_ptr);

        return successor;
    }

    Node3D RRTPlanner::SelectRandomNode()
    {
        Node3D random_node;
        while (1)
        {
            // 1. randomly choose x
            float x = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / map_width_));
            // 2. randomly choose y
            float y = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / map_height_));
            // 3. collision check
            random_node.setX(x);
            random_node.setY(y);
            if (configuration_space_ptr_->IsTraversable(random_node))
            {
                return random_node;
            }
        }
    }

    Node3D RRTPlanner::FindClosestNode(const Node3D &random_node)
    {
        Node3D closest_node;
        if (rrt_.size() <= 0)
        {
            DLOG(WARNING) << "size of rrt tree is smaller than zero!!!!";
            return closest_node;
        }
        size_t closest_index = -1;
        float min_distance = 100000000000000;
        for (size_t index = 0; index < rrt_.size(); index++)
        {
            float current_distance = Utility::GetDistance(random_node, rrt_[index]);
            if (current_distance < min_distance)
            {
                min_distance = current_distance;
                closest_index = index;
            }
        }
        closest_node = rrt_[closest_index];
        return closest_node;
    }

    std::pair<float, float> RRTPlanner::FindStepSizeAndSteeringAngle(const Node3D &closest_node, const Node3D &random_node)
    {
        float step_size = 0, steering_angle = 0;
        // 1. use closest_node find obstacle density
        // 2. use obstacle density to determine step size like hybrid a star
        float distance_to_goal = Utility::GetDistance(closest_node, random_node);
        // DLOG(INFO) << "distance to goal is " << distance_to_goal;
        // 2. if distance to goal is less than step size above. than make it new step size, otherwise use old one
        // DLOG_IF(WARNING, step_size_steering_angle_pair.size() == 0) << "step_size_steering_angle_pair size is zero!!!";

        float weight_step_size = -0.8 * configuration_space_ptr_->GetNormalizedObstacleDensity(closest_node) + 0.9;
        step_size = weight_step_size * distance_to_goal;
        // DLOG_IF(INFO, step_size < 1) << "step size is " << step_size;

        // 3. steering angle is the angle between closest node and random node
        float angle_between_two_nodes = Utility::GetAngle(random_node, closest_node);
        steering_angle = -Utility::RadNormalization(closest_node.GetT() - angle_between_two_nodes);

        return std::make_pair(step_size, steering_angle);
    }

    void RRTPlanner::AddNodeToRRT(const Node3D &current)
    {
        rrt_.emplace_back(current);
    }
}