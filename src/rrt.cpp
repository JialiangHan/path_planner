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
        // DLOG(INFO) << "In Planning!!!";
        // actually no current node for RRT
        AddNodeToRRT(start_);
        Node3D current;
        int number_of_iterations = 0;
        // VISUALIZATION DELAY
        ros::Duration delay(0.003);

        while (number_of_iterations < params_.max_iterations)
        {
            // goal check
            if (GoalCheck(params_.consider_orientation) >= 0)
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
        DLOG(INFO) << "number of nodes explored is " << rrt_.size();
        TracePath();
    }

    Path3D RRTPlanner::GetPath(const Node3D &start, const Node3D &goal)
    {
        // DLOG(INFO) << "In GetPath!!!";
        start_ = start;
        goal_ = goal;
        rrt_.clear();
        Planning();
        return path_;
    }

    void RRTPlanner::TracePath()
    {
        // DLOG(INFO) << "In TracePath!!!";
        path_.clear();
        int goal_index = GoalCheck(params_.consider_orientation);
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

    int RRTPlanner::GoalCheck(bool consider_orientation)
    {
        // DLOG(INFO) << "In GoalCheck!!!";
        // goal index, default is -1
        int index = -1;
        if (rrt_.size() == 0)
        {
            DLOG(WARNING) << "WARNING: size of rrt is zero!!";
            return index;
        }
        // loop all the node in rrt
        for (size_t i = 0; i < rrt_.size(); i++)
        {
            if (Utility::IsCloseEnough(rrt_[i], goal_, params_.goal_range, 2 * M_PI / params_.headings, consider_orientation))
            {
                DLOG(INFO) << "Goal reached, index is: " << i;
                index = i;
            }
        }
        return index;
    }

    Node3D RRTPlanner::GenerateSuccessor()
    {
        // DLOG(INFO) << "In GenerateSuccessor()!!!";
        Node3D successor, direction_node, closest_node;
        while (1)
        {
            std::pair<float, float> step_size_steering_angle_pair;
            float random_number = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            // 1. if probability greater than parameter, then find a random node, otherwise use goal node
            if (random_number > params_.possibility_to_goal)
            {
                // DLOG(INFO) << "Towards Random node!";
                direction_node = SelectRandomNode();
            }
            else
            {
                // DLOG(INFO) << "Towards Goal!";
                direction_node = goal_;
            }
            // 2. find closet node on rrt to this random node
            closest_node = FindClosestNode(direction_node);
            // 3. find step size and steering angle using random node
            step_size_steering_angle_pair = FindStepSizeAndSteeringAngle(closest_node, direction_node);
            // 4. create successor using step size and steering angle
            if (step_size_steering_angle_pair.first != 0)
            {
                successor = GenerateSuccessor(closest_node, step_size_steering_angle_pair);
                if (configuration_space_ptr_->IsTraversable(successor))
                {
                    break;
                }
            }
        }
        DLOG(INFO) << "successor is " << successor.GetX() << " " << successor.GetY() << " " << Utility::ConvertRadToDeg(successor.GetT());
        return successor;
    }

    Node3D RRTPlanner::GenerateSuccessor(const Node3D &closest_node, const std::pair<float, float> stepsize_steering_angle)
    {
        float dx, dy, xSucc, ySucc, tSucc, turning_radius, steering_angle;
        std::shared_ptr<Node3D> closest_node_ptr = std::make_shared<Node3D>(closest_node);
        // DLOG_IF(INFO, (pred.GetX() > 76) && (pred.GetX() < 77) && (pred.GetY() > 1) && (pred.GetY() < 2)) << "in create successor, current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT());
        if (stepsize_steering_angle.first <= 1e-3)
        {
            // DLOG(INFO) << "current step size is zero, no need to create successor!!";
        }
        steering_angle = stepsize_steering_angle.second;
        turning_radius = stepsize_steering_angle.first / abs(steering_angle);
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
        tSucc = Utility::RadToZeroTo2P(closest_node.GetT() + steering_angle);
        // DLOG_IF(INFO, (closest_node.GetX() > 76) && (closest_node.GetX() < 77) && (closest_node.GetY() > 1) && (closest_node.GetY() < 2)) << "successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
        // DLOG(INFO) << "successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
        Node3D successor(xSucc, ySucc, tSucc, 0, 0, closest_node_ptr);
        return successor;
    }
    // looks correct
    Node3D RRTPlanner::SelectRandomNode()
    {
        // DLOG(INFO) << "In SelectRandomNode!!!";
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
                // DLOG(INFO) << "random node is " << x << " " << y;
                return random_node;
            }
        }
    }

    Node3D RRTPlanner::FindClosestNode(const Node3D &random_node)
    {
        // DLOG(INFO) << "In FindClosestNode!!!";
        Node3D closest_node;
        if (rrt_.size() <= 0)
        {
            DLOG(WARNING) << "WARNING: size of rrt tree is smaller than zero!!!!";
            return closest_node;
        }
        size_t closest_index = -1;
        float min_distance = 1000000000000;
        for (size_t index = 0; index < rrt_.size(); index++)
        {
            float current_distance = Utility::GetDistance(random_node, rrt_[index]);
            // DLOG(INFO) << "current distance is " << current_distance;
            if (current_distance < min_distance)
            {
                min_distance = current_distance;
                closest_index = index;
            }
        }
        closest_node = rrt_[closest_index];
        DLOG(INFO) << "closest node is " << closest_node.GetX() << " " << closest_node.GetY() << " " << Utility::ConvertRadToDeg(closest_node.GetT()) << " random node is " << random_node.GetX() << " " << random_node.GetY() << " " << Utility::ConvertRadToDeg(random_node.GetT()) << " distance is " << min_distance;
        return closest_node;
    }

    std::pair<float, float> RRTPlanner::FindStepSizeAndSteeringAngle(const Node3D &closest_node, const Node3D &random_node)
    {
        // DLOG(INFO) << "closest node is " << closest_node.GetX() << " " << closest_node.GetY() << " " << Utility::ConvertRadToDeg(closest_node.GetT()) << " random node is " << random_node.GetX() << " " << random_node.GetY() << " " << Utility::ConvertRadToDeg(random_node.GetT());
        float step_size = 0, steering_angle = 0;
        //  float distance_to_goal = Utility::GetDistance(closest_node, random_node);
        float distance_to_goal = Utility::GetDistance(closest_node, goal_);
        // float distance_from_start_to_goal = Utility::GetDistance(start_, goal_);
        // 3. steering angle is the angle between closest node and random node, and when close to goal(distance to goal < some certain number), steering angle need to be goal.GetT()- closest node.GetT()
        float angle_between_two_nodes = Utility::RadNormalization(Utility::GetAngle(closest_node, random_node));
        // DLOG(INFO) << "angle between two nodes is " << Utility::ConvertRadToDeg(angle_between_two_nodes) << " goal angle is " << Utility::ConvertRadToDeg(random_node.GetT()) << " target angle is " << Utility::ConvertRadToDeg(target_angle);
        // DLOG(INFO) << "angle between two node is " << Utility::ConvertRadToDeg(angle_between_two_nodes);
        steering_angle = -Utility::RadNormalization(Utility::RadNormalization(closest_node.GetT()) - angle_between_two_nodes);
        // 2. use obstacle density to determine step size like hybrid a star
        if (params_.adaptive_step_size)
        {
            float step_size_obstacle = 10000;
            bool consider_steering_angle_range = false;
            std::vector<std::pair<float, Utility::AngleRange>> available_angle_range_vec = configuration_space_ptr_->FindFreeAngleRangeAndObstacleAngleRange(closest_node, consider_steering_angle_range);
            // find distance to obstacle in steering angle direction
            for (const auto &pair : available_angle_range_vec)
            {
                if (Utility::IsAngleRangeInclude(pair.second, angle_between_two_nodes))
                {
                    // DLOG(INFO) << "distance is " << pair.first << " angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " angle between two nodes is " << Utility::ConvertRadToDeg(angle_between_two_nodes);
                    step_size_obstacle = pair.first;
                    // DLOG(INFO) << "angle is included by angle range, set distance to step size obstacle";
                    break;
                }
            }
            float weight_step_size = -0.8 * configuration_space_ptr_->GetNormalizedObstacleDensity(closest_node) + 0.9;
            float available_step_size_obstacle = ((step_size_obstacle - 0.5 * params_.collision_detection_params.vehicle_length) > 0) ? (step_size_obstacle - 0.5 * params_.collision_detection_params.vehicle_length) : 0;
            if (available_step_size_obstacle != 0)
            {
                // DLOG(INFO) << "available_step_size_obstacle is " << available_step_size_obstacle;
                if (available_step_size_obstacle > distance_to_goal)
                {
                    available_step_size_obstacle = distance_to_goal;
                    // DLOG(INFO) << "weight step size is " << weight_step_size << " distance to goal is " << distance_to_goal;
                }
                step_size = weight_step_size * available_step_size_obstacle;
                DLOG(INFO) << "step_size is " << step_size << " weight is " << weight_step_size;
                DLOG(INFO) << "available step size obstacle is " << available_step_size_obstacle;
                if (step_size < 1)
                {
                    if (available_step_size_obstacle > 1)
                    {
                        step_size = 1;
                        DLOG(INFO) << "step size is smaller than  predefined min distance, make it to one!!";
                    }
                    else
                    {
                        step_size = available_step_size_obstacle;
                        DLOG(INFO) << "step size is smaller than  predefined min distance, make it to available step size: " << available_step_size_obstacle << " !!";
                    }
                }
            }
            else
            {
                // DLOG(INFO) << "min distance to obstacle is " << step_size_obstacle << " weight is " << weight_step_size;
                // DLOG(INFO) << "available step size obstacle is " << available_step_size_obstacle;
                DLOG(WARNING) << "WARNING: step size is zero!!!";
            }
        }
        else
        {
            step_size = 1;
            if (step_size > distance_to_goal)
            {
                step_size = distance_to_goal;
                // DLOG(INFO) << "weight step size is " << weight_step_size << " distance to goal is " << distance_to_goal;
            }
        }
        DLOG(INFO) << "step size is " << step_size << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
        return std::make_pair(step_size, steering_angle);
    }

    void RRTPlanner::AddNodeToRRT(const Node3D &current)
    {
        // DLOG(INFO) << "In AddNodeToRRT!!!";
        rrt_.emplace_back(current);
        DLOG(INFO) << "add node " << current.GetX() << " " << current.GetY() << " " << Utility::ConvertRadToDeg(current.GetT());
    }

    Path3D RRTPlanner::ShortCut(bool consider_steering_angle_limit)
    {
        Path3D out;
        // start from last point of path, check if segment between end point and path[n-1] is in collision, if not, then check end point and path[n-2] is in collision
        Node3D current_point, previous_point;
        previous_point = path_[path_.size() - 2];
        for (uint index1 = path_.size() - 1; index1 >= 0;)
        {
            DLOG(INFO) << "index1 is " << index1;
            current_point = path_[index1];
            out.emplace_back(current_point);
            for (uint index2 = index1 - 1; index2 >= 0; index2--)
            {
                DLOG(INFO) << "index2 is " << index2;
                if (configuration_space_ptr_->IsTraversable(current_point, path_[index2]))
                {
                    previous_point = path_[index2];
                    // end check
                    if (index2 == 0)
                    {
                        out.emplace_back(previous_point);
                    }
                }
                else
                {
                    out.emplace_back(previous_point);
                    if (index2 == 0)
                    {
                        index1 = 0;
                    }
                    else
                    {
                        index1 = index2 - 1;
                    }
                    break;
                }
                if (index2 == 0)
                {
                    index1 = 0;
                    break;
                }
            }
            if (index1 == 0)
            {
                break;
            }
        }
        std::reverse(out.begin(), out.end());
        return out;
    }
}