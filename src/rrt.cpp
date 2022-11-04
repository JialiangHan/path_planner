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

    Utility::Path3D RRTPlanner::GetPath(const Node3D &start, const Node3D &goal)
    {
        // DLOG(INFO) << "In GetPath!!!";
        start_ = start;
        goal_ = goal;
        Planning();
        return path_;
    }

    void RRTPlanner::Planning()
    {
        DLOG(INFO) << "In Planning!!!";
        DLOG_IF(INFO, params_.twoD_rrt) << "2D rrt!";
        DLOG_IF(INFO, params_.adaptive_possibility_to_goal) << "adaptive_possibility_to_goal!";
        DLOG_IF(INFO, params_.use_AEB_rrt) << "use_AEB_rrt!";
        DLOG_IF(INFO, params_.rewire) << "rewire!";
        DLOG_IF(INFO, params_.use_rrt_connect) << "use_rrt_connect!";
        if (params_.use_rrt_connect || params_.use_AEB_rrt)
        {
            RRTConnectPlanner();
            return;
        }
        RRT rrt_start = InitializeRRT(start_);

        Node3D current;
        int number_of_iterations = 0;

        int goal_index;
        while (number_of_iterations < params_.max_iterations)
        {
            // goal check
            goal_index = GoalCheck(rrt_start, params_.consider_orientation);
            if (goal_index >= 0)
            {
                break;
            }
            // find successor for current node
            current = GenerateSuccessor(rrt_start);

            if (params_.analytical_expansion && !params_.twoD_rrt)
            {
                if (AnalyticExpansion(rrt_start, current, goal_))
                {
                    goal_index = GoalCheck(rrt_start, params_.consider_orientation);
                    break;
                }
            }
            number_of_iterations++;
        }
        DLOG(INFO) << "number of nodes explored is " << rrt_start.size();
        path_ = TracePath(rrt_start, goal_index);
    }

    Utility::Path3D RRTPlanner::TracePath(const RRT &rrt, const int &goal_index)
    {
        Utility::Path3D path;
        // DLOG(INFO) << "In TracePath!!!";
        CHECK(goal_index > 0) << "goal index smaller than zero!!!";

        std::shared_ptr<Node3D> node3d_ptr = std::make_shared<Node3D>(rrt[goal_index]);
        while (node3d_ptr != nullptr)
        {
            path.emplace_back(*node3d_ptr);
            if (*node3d_ptr == start_)
            {
                break;
            }
            // DLOG(INFO) << "current node is " << node3d_ptr->GetX() << " " << node3d_ptr->GetY() << " and its pred is " << node3d_ptr->GetPred()->GetX() << " " << node3d_ptr->GetPred()->GetY();
            node3d_ptr = node3d_ptr->GetPred();
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    int RRTPlanner::GoalCheck(const RRT &rrt, bool consider_orientation)
    {
        // DLOG(INFO) << "In GoalCheck!!!";
        // goal index, default is -1
        int index = -1;
        if (rrt.size() == 0)
        {
            DLOG(WARNING) << "WARNING: size of rrt is zero!!";
            return index;
        }
        // loop all the node in rrt
        for (size_t i = 0; i < rrt.size(); i++)
        {
            if (params_.twoD_rrt)
            {
                if (Utility::GetDistance(rrt[i], goal_) < 0.1)
                {
                    DLOG(INFO) << "Goal reached, index is: " << i;
                    index = i;
                }
            }

            // DLOG(INFO) << "in for loop.";
            if (Utility::IsCloseEnough(rrt[i], goal_, params_.goal_range, 2 * M_PI / params_.headings, consider_orientation))
            {
                DLOG(INFO) << "Goal reached, index is: " << i;
                index = i;
            }
        }
        return index;
    }

    Node3D RRTPlanner::GenerateSuccessor(RRT &rrt_start)
    {
        // DLOG(INFO) << "In GenerateSuccessor!!!";
        Node3D successor, direction_node, closest_node;

        int failure_counts = 0;
        while (1)
        {
            direction_node = FindDirectionNode(failure_counts, rrt_start);
            std::pair<Status, Node3D> status_successor = Extend(rrt_start, direction_node);
            if (status_successor.first == Status::Trapped)
            {
                failure_counts++;
            }
            else
            {
                successor = status_successor.second;
                break;
            }
        }

        return successor;
    }

    Node3D RRTPlanner::FindDirectionNode(const int &failure_counts, const RRT &rrt, const RRTFlag &flag)
    {
        Node3D direction_node;
        float random_number = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float possibility_to_goal = GetPossibilityToGoal(failure_counts);
        // DLOG(INFO) << "random number is " << random_number;
        // 1. if probability greater than parameter, then find a random node, otherwise use goal node
        if (random_number < possibility_to_goal)
        {
            // DLOG(INFO) << "Towards Random node!";

            while (1)
            {
                direction_node = SelectRandomNode();
                if (!CheckNodeOnRRT(rrt, direction_node))
                {
                    break;
                }
            }
        }
        else
        {
            // DLOG(INFO) << "Towards Goal!";
            Node3D target;
            if (flag == RRTFlag::Start)
            {
                target = goal_;
            }
            else
            {
                target = TreatNode(start_);
            }

            direction_node = target;
        }
        return direction_node;
    }

    Node3D RRTPlanner::GenerateSuccessor(const Node3D &closest_node, const std::pair<float, float> stepsize_steering_angle)
    {
        float dx, dy, xSucc, ySucc, tSucc, turning_radius, steering_angle, step_size;
        std::shared_ptr<Node3D> closest_node_ptr = std::make_shared<Node3D>(closest_node);
        if (params_.twoD_rrt)
        {
            xSucc = closest_node.GetX() + stepsize_steering_angle.first * cos(stepsize_steering_angle.second);
            ySucc = closest_node.GetY() + stepsize_steering_angle.first * sin(stepsize_steering_angle.second);
            tSucc = Utility::RadToZeroTo2P(closest_node.GetT() + stepsize_steering_angle.second);
        }
        else
        {

            // DLOG_IF(INFO, (pred.GetX() > 76) && (pred.GetX() < 77) && (pred.GetY() > 1) && (pred.GetY() < 2)) << "in create successor, current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT());
            if (stepsize_steering_angle.first <= 1e-3)
            {
                // DLOG(INFO) << "current step size is zero, no need to create successor!!";
                step_size = 0;
            }
            else
            {
                step_size = stepsize_steering_angle.first;
            }

            steering_angle = stepsize_steering_angle.second;
            turning_radius = step_size / abs(steering_angle);
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
                dx = step_size;
                dy = 0;
                // DLOG(INFO) << "forward straight";
            }
            xSucc = closest_node.GetX() + dx * cos(closest_node.GetT()) - dy * sin(closest_node.GetT());
            ySucc = closest_node.GetY() + dx * sin(closest_node.GetT()) + dy * cos(closest_node.GetT());
            tSucc = Utility::RadToZeroTo2P(closest_node.GetT() + steering_angle);
        }

        // DLOG_IF(INFO, (closest_node.GetX() > 76) && (closest_node.GetX() < 77) && (closest_node.GetY() > 1) && (closest_node.GetY() < 2)) << "successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
        DLOG(INFO) << "successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
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

    Node3D RRTPlanner::FindClosestNode(const RRT &rrt, const Node3D &random_node, const bool &flag)
    {
        // DLOG(INFO) << "In FindClosestNode!!!";
        Node3D closest_node;
        if (rrt.size() <= 0)
        {
            DLOG(WARNING) << "WARNING: size of rrt tree is smaller than zero!!!!";
            return closest_node;
        }
        int closest_index = -1;
        float min_distance = 1000000000000;
        // DLOG(INFO) << "index is " << closest_index;
        for (size_t index = 0; index < rrt.size(); index++)
        {
            float current_distance = Utility::GetDistance(random_node, rrt[index]);
            // DLOG(INFO) << "current index is " << index << " current distance is " << current_distance;

            if (current_distance < min_distance)
            {
                if (!flag)
                {
                    if (configuration_space_ptr_->IsTraversable(rrt[index], random_node))
                    {
                        min_distance = current_distance;
                        closest_index = index;
                    }
                }
                else
                {
                    min_distance = current_distance;
                    closest_index = index;
                }
            }
        }
        // DLOG(INFO) << "index is " << closest_index;
        if (closest_index == -1)
        {
            closest_node = FindClosestNode(rrt, SelectRandomNode(), flag);
        }
        else
        {
            closest_node = rrt[closest_index];
        }

        // DLOG(INFO) << "closest node is " << closest_node.GetX() << " " << closest_node.GetY() << " " << Utility::ConvertRadToDeg(closest_node.GetT()) << " random node is " << random_node.GetX() << " " << random_node.GetY() << " " << Utility::ConvertRadToDeg(random_node.GetT()) << " distance is " << min_distance;
        return closest_node;
    }

    float RRTPlanner::FindSteeringAngle(const Node3D &closest_node, const Node3D &direction_node)
    {
        float steering_angle, angle_between_two_nodes = Utility::RadNormalization(Utility::GetAngle(closest_node, direction_node));
        // 3. steering angle is the angle between closest node and random node, and when close to goal(distance to goal < some certain number), steering angle need to be goal.GetT()- closest node.GetT()
        // DLOG(INFO) << "angle between two nodes is " << Utility::ConvertRadToDeg(angle_between_two_nodes) << " goal angle is " << Utility::ConvertRadToDeg(random_node.GetT()) << " target angle is " << Utility::ConvertRadToDeg(target_angle);
        // DLOG(INFO) << "angle between two node is " << Utility::ConvertRadToDeg(angle_between_two_nodes);
        if (params_.consider_steering_angle_range)
        {
            steering_angle = SelectRandomSteeringAngle(Utility::ConvertDegToRad(30), closest_node);
        }
        else
        {
            if (params_.twoD_rrt)
            {
                steering_angle = angle_between_two_nodes;
            }
            else
            {
                steering_angle = -Utility::RadNormalization(Utility::RadNormalization(closest_node.GetT()) - angle_between_two_nodes);
            }
        }
        // DLOG_IF(INFO, (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < Utility::ConvertDegToRad(-30))) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
        return steering_angle;
    }

    float RRTPlanner::SelectRandomSteeringAngle(const float &max_steering_angle, const Node3D &current)
    {
        float steering_angle, random_number = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        // 1. if probability greater than parameter, then find a random node, otherwise use goal node
        if (random_number > params_.possibility_to_goal)
        {
            // DLOG(INFO) << "Randomly Steering!";
            float steering_angle_resolution;
            bool use_random = false;
            if (use_random)
            {
                steering_angle_resolution = rand() % 5;
            }
            else
            {
                steering_angle_resolution = params_.steering_angle_resolution;
            }

            int random_number = rand() % ((int)(std::round(Utility::ConvertRadToDeg(2 * max_steering_angle)) / steering_angle_resolution));
            steering_angle = -max_steering_angle + Utility::ConvertDegToRad(random_number * steering_angle_resolution);
            // steering_angle = -max_steering_angle + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * max_steering_angle)));
            // steering_angle = std::round(Utility::ConvertRadToDeg(-max_steering_angle + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * max_steering_angle)))));
            // steering_angle = Utility::ConvertDegToRad(steering_angle);
        }
        else
        {
            // DLOG(INFO) << "Towards Goal!";
            float angle_between_current_goal = Utility::RadNormalization(Utility::GetAngle(current, goal_));
            steering_angle = Utility::RadNormalization(angle_between_current_goal - Utility ::RadNormalization(current.GetT()));
            if (steering_angle > max_steering_angle)
            {
                steering_angle = max_steering_angle;
            }
            else if (steering_angle < -max_steering_angle)
            {
                steering_angle = -max_steering_angle;
            }
            if ((steering_angle > max_steering_angle) || (steering_angle < -max_steering_angle))
            {
                // DLOG(INFO) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " angle between current node and goal is " << Utility::ConvertRadToDeg(angle_between_current_goal) << " current node orientation is " << Utility ::ConvertRadToDeg(current.GetT());
            }
        }
        // DLOG_IF(INFO, (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < Utility::ConvertDegToRad(-30))) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
        return steering_angle;
    }

    std::pair<float, float> RRTPlanner::FindStepSizeAndSteeringAngle(const Node3D &closest_node, const Node3D &direction_node)
    {
        // DLOG(INFO) << "closest node is " << closest_node.GetX() << " " << closest_node.GetY() << " " << Utility::ConvertRadToDeg(closest_node.GetT()) << " direction_node is " << direction_node.GetX() << " " << direction_node.GetY() << " " << Utility::ConvertRadToDeg(direction_node.GetT());

        float steering_angle = FindSteeringAngle(closest_node, direction_node);
        float step_size = FindStepSize(closest_node, steering_angle, direction_node);

        // DLOG_IF(INFO, (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < Utility::ConvertDegToRad(-30)) || (step_size < params_.step_size)) << "step size is " << step_size << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
        // DLOG_IF(INFO, (step_size < params_.step_size)) << "step size is " << step_size << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);

        DLOG(INFO) << "step size is " << step_size << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
        return std::make_pair(step_size, steering_angle);
    }

    float RRTPlanner::FindStepSize(const Node3D &closest_node, const float &steering_angle, const Node3D &target)
    {
        float step_size = 0, distance_to_goal = Utility::GetDistance(closest_node, target), step_size_obstacle = 10000;

        bool consider_steering_angle_range = false;
        std::vector<std::pair<float, Utility::AngleRange>> available_angle_range_vec = configuration_space_ptr_->FindFreeAngleRangeAndObstacleAngleRange(closest_node, consider_steering_angle_range);
        float final_orientation = Utility::RadNormalization(steering_angle + Utility::RadNormalization(closest_node.GetT()));
        // find distance to obstacle in steering angle direction
        DLOG_IF(WARNING, available_angle_range_vec.size() == 0) << "available_angle_range_vec size is " << available_angle_range_vec.size();
        if (params_.use_AEB_rrt && params_.number_of_step_size != 0)
        {
            // DLOG(INFO) << "expand like AEB_RRT";
            if (Utility::GetDistance(target, goal_) < 0.1 || Utility::GetDistance(target, start_) < 0.1)
            {
                step_size = 2 * params_.step_size;
            }
            else
            {
                step_size = params_.step_size;
            }
        }
        else
        {
            for (const auto &pair : available_angle_range_vec)
            {
                if (Utility::IsAngleRangeInclude(pair.second, final_orientation))
                {
                    // DLOG(INFO) << "distance is " << pair.first << " angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " final orientation is " << Utility::ConvertRadToDeg(final_orientation);
                    step_size_obstacle = pair.first;
                    // DLOG_IF(INFO, step_size_obstacle == 0) << "distance is " << pair.first << " angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " final orientation is " << Utility::ConvertRadToDeg(final_orientation);
                    // DLOG(INFO) << "angle is included by angle range, set distance to step size obstacle";
                    break;
                }
                else
                {
                    // DLOG(INFO) << "distance is " << pair.first << " angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " final orientation is " << Utility::ConvertRadToDeg(final_orientation);
                }
            }
            float available_step_size_obstacle = ((step_size_obstacle - 0.5 * params_.collision_detection_params.vehicle_length) > 0) ? (step_size_obstacle - 0.5 * params_.collision_detection_params.vehicle_length) : 0;
            DLOG_IF(INFO, step_size_obstacle == 10000 || params_.collision_detection_params.vehicle_length != 2) << "step size obstacle is " << step_size_obstacle << " vehicle length is " << params_.collision_detection_params.vehicle_length;
            DLOG_IF(INFO, available_step_size_obstacle == 0) << "available_step_size_obstacle is " << available_step_size_obstacle << " vehicle length is " << params_.collision_detection_params.vehicle_length << " step_size_obstacle is " << step_size_obstacle;
            // original step size is purely determined by obstacle density.
            step_size = FindOriginalStepSize(closest_node, distance_to_goal, available_step_size_obstacle);
            if (available_step_size_obstacle > params_.step_size)
            {
                DLOG_IF(INFO, step_size < params_.step_size) << "step_size is " << step_size << " available step size obstacle is " << available_step_size_obstacle;
                if (step_size < params_.step_size)
                {
                    step_size = params_.step_size;
                    DLOG(INFO) << "step size (" << step_size << ") is smaller than  predefined min distance, make it to available step size: " << params_.step_size << " !!";
                }
            }
            else
            {
                step_size = 0;
                DLOG(INFO) << "min distance to obstacle is " << step_size_obstacle << " available step size obstacle is " << available_step_size_obstacle;
                // DLOG(WARNING) << "WARNING: step size is zero!!!";
            }
        }

        if (step_size > distance_to_goal)
        {
            DLOG(INFO) << "step size " << step_size << " larger than distance to goal " << distance_to_goal;
            step_size = distance_to_goal;
        }
        // DLOG(INFO) << "FindStepSize: step size is " << step_size;
        DLOG_IF(INFO, step_size < params_.step_size) << "step size is " << step_size;
        return step_size;
    }

    Utility::Path3D RRTPlanner::ShortCut(const Utility::Path3D &path, bool consider_steering_angle_limit)
    {
        Utility::Path3D out, input = path;
        // start from last point of path, check if segment between end point and path[n-1] is in collision, if not, then check end point and path[n-2] is in collision
        while (1)
        {
            Node3D current_point, previous_point;
            previous_point = input[input.size() - 2];
            for (uint index1 = input.size() - 1; index1 >= 0;)
            {
                // DLOG(INFO) << "index1 is " << index1;
                current_point = input[index1];
                out.emplace_back(current_point);
                for (uint index2 = index1 - 1; index2 >= 0; index2--)
                {
                    // DLOG(INFO) << "index2 is " << index2;
                    if (configuration_space_ptr_->IsTraversable(current_point, input[index2]))
                    {
                        previous_point = input[index2];
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
            if (input == out)
            {
                break;
            }
            else
            {
                input = out;
                out.clear();
            }
        }

        std::reverse(out.begin(), out.end());
        return out;
    }

    Utility::Path3D RRTPlanner::PiecewiseCubicBezier(const Utility::Path3D &path)
    { // DLOG(INFO) << "ConvertToPiecewiseCubicBezierPath in:";
        std::vector<Eigen::Vector3f> anchor_points_vec;
        Utility::Path3D out;
        for (uint index = 1; index < path.size() - 1; ++index)
        {
            // DLOG(INFO) << "path point is " << path_[index].GetX() << " " << path_[index].GetY() << " " << Utility::ConvertRadToDeg(path_[index].GetT());
            // not necessary to do this check, greater than 1

            anchor_points_vec.emplace_back(Utility::ConvertNode3DToVector3f(path[index]));
            // DLOG(INFO) << "anchor points is " << path_[index].GetX() << " " << path_[index].GetY() << " " << Utility::ConvertRadToDeg(path_[index].GetT());
        }
        // DLOG(INFO) << "end point is " << end_point.GetX() << " " << end_point.GetY() << " " << Utility::ConvertRadToDeg(end_point.GetT());
        HybridAStar::PiecewiseCubicBezier pwcb(Utility::ConvertNode3DToVector3f(start_), Utility::ConvertNode3DToVector3f(goal_));
        pwcb.SetAnchorPoints(anchor_points_vec);
        // for (const auto &point : anchor_points_vec)
        // {
        //   DLOG(INFO) << "anchor point is " << point.x() << " " << point.y() << " " << Utility::ConvertRadToDeg(point.z());
        // }
        std::vector<Eigen::Vector3f> points_vec = pwcb.GetPointsVec();
        // for (const auto &point : points_vec)
        // {
        //   DLOG(INFO) << "all point is " << point.x() << " " << point.y() << " " << Utility::ConvertRadToDeg(point.z());
        // }
        std::vector<Eigen::Vector3f> path_vec = pwcb.ConvertPiecewiseCubicBezierToVector3f(10);
        for (const auto &vector : path_vec)
        {
            // DLOG(INFO) << "vector is " << vector.x() << " " << vector.y() << " " << Utility::ConvertRadToDeg(vector.z());
            Node3D node3d = Utility::ConvertVector3fToNode3D(vector);
            out.emplace_back(node3d);
            // DLOG(INFO) << "point is " << node3d.GetX() << " " << node3d.GetY() << " " << Utility::ConvertRadToDeg(node3d.GetT());
        }
        // DLOG(INFO) << "ConvertToPiecewiseCubicBezierPath out.";
        return out;
    }

    bool RRTPlanner::AnalyticExpansion(RRT &rrt, const Node3D &start, Node3D &goal)
    {
        Utility::Path3D path_vec;
        int i = 0;
        float x = 0.f;
        Eigen::Vector3f vector3d_start = Utility::ConvertNode3DToVector3f(start);

        Eigen::Vector3f vector3d_goal = Utility::ConvertNode3DToVector3f(goal);

        CubicBezier::CubicBezier cubic_bezier(vector3d_start, vector3d_goal, map_width_, map_height_);
        float length = cubic_bezier.GetLength();
        path_vec.clear();
        Node3D node3d;
        std::shared_ptr<Node3D> pred_ptr = std::make_shared<Node3D>(start);
        while (x < length)
        {
            if (x == 0)
            {
                node3d = start;
            }
            else
            {
                node3d = Utility::ConvertVector3fToNode3D(cubic_bezier.GetValueAt(x / length));
                node3d.SetPred(pred_ptr);
            }
            // DLOG(INFO) << i << "th iteration";
            float curvature = cubic_bezier.GetCurvatureAt(x / length);
            node3d.setT(cubic_bezier.GetAngleAt(x / length));
            // DLOG(INFO) << "current node is " << node3d.GetX() << " " << node3d.GetY();
            // collision check
            if (configuration_space_ptr_->IsTraversable(node3d))
            {
                if (curvature <= 1 / params_.collision_detection_params.min_turning_radius)
                {
                    path_vec.emplace_back(node3d);
                    x += params_.curve_step_size;
                    i++;
                    pred_ptr = std::make_shared<Node3D>(node3d);
                }
                else
                {
                    // DLOG(INFO) << "cubic bezier curvature greater than 1/min_turning_radius, discarding the path";
                    path_vec.clear();
                    break;
                }
            }
            else
            {
                // DLOG(INFO) << "cubic bezier collided, discarding the path";
                path_vec.clear();
                break;
                // return nullptr;
            }
            // DLOG(INFO) << "goal point is " << vector3d_goal.x() << " " << vector3d_goal.y();
        }
        // Some kind of collision detection for all curves
        if (path_vec.size() != 0)
        {
            goal.SetPred(pred_ptr);
            path_vec.emplace_back(goal);
            // DLOG(INFO) << "start point is " << vector3d_start.x() << " " << vector3d_start.y();
            DLOG(INFO) << "Analytical expansion connected, returning path";
            AddNodeToRRT(rrt, path_vec);
            return true;
        }
        else
        {
            return false;
        }
    }
    void RRTPlanner::AddNodeToRRT(RRT &rrt, Utility::Path3D &current)
    {
        DLOG(INFO) << "In AddNodeToRRT!!!";
        for (auto &item : current)
        {
            AddNodeToRRT(rrt, item);
        }
    }

    void RRTPlanner::AddNodeToRRT(RRT &rrt, Node3D &current)
    {
        // DLOG(INFO) << "In AddNodeToRRT!!!";
        if (CheckNodeOnRRT(rrt, current))
        {
            DLOG(INFO) << "node already on rrt.";
            return;
        }
        else
        {
            if (params_.rewire || params_.use_AEB_rrt)
            {
                DLOG(INFO) << "rewiring";
                Rewire(rrt, current, params_.neighbor_detection_radius);
            }
            else
            {
                rrt.emplace_back(current);
                // DLOG(INFO) << "add node " << current.GetX() << " " << current.GetY() << " " << Utility::ConvertRadToDeg(current.GetT());
            }
        }
    }

    float RRTPlanner::GetPossibilityToGoal(const int &failure_counts)
    {
        // DLOG(INFO) << "in GetPossibilityToGoal, failure counts is " << failure_counts;
        float possibility_to_goal = 0;
        float p_min = 0.1, p_max = 1;
        if (params_.adaptive_possibility_to_goal || params_.use_AEB_rrt)
        {
            possibility_to_goal = p_min + (p_max - p_min) * std::exp(-9 / std::pow(failure_counts + 1, 3));
            // DLOG(INFO) << "failure counts is " << failure_counts << " possibility to goal is " << possibility_to_goal;
            // DLOG(INFO) << "std::exp(-9 / std::pow(failure_counts + 1, 3)) is " << std::exp(-9 / std::pow(failure_counts + 1, 3));
        }
        else
        {
            possibility_to_goal = params_.possibility_to_goal;
        }
        // DLOG_IF(INFO, possibility_to_goal != params_.possibility_to_goal) << "failure counts is " << failure_counts << " possibility to goal is " << possibility_to_goal;
        // DLOG(INFO) << "failure counts is " << failure_counts << " possibility to goal is " << possibility_to_goal;
        return possibility_to_goal;
    }

    float RRTPlanner::FindOriginalStepSize(const Node3D &closest_node, const float &distance_to_goal, const float &available_step_size_obstacle)
    {
        float step_size = 0;
        float obstacle_detection_range = configuration_space_ptr_->GetObstacleDetectionRange();
        float obstacle_density = configuration_space_ptr_->GetNormalizedObstacleDensity(closest_node);
        if (params_.number_of_step_size == 0)
        {
            // float distance_start_to_goal = Utility::GetDistance(start_, goal_);
            float weight_step_size = -0.8 * obstacle_density + 0.9;
            // step_size = distance_to_goal / distance_start_to_goal * weight_step_size * available_step_size_obstacle;
            step_size = weight_step_size * available_step_size_obstacle;
            DLOG_IF(INFO, step_size == 0) << "step_size is " << step_size << " weight is " << weight_step_size << " available step size obstacle is " << available_step_size_obstacle;
            // DLOG(INFO) << "available_step_size_obstacle is " << available_step_size_obstacle;
            // DLOG_IF(INFO, step_size < params_.step_size) << "step_size is " << step_size << " weight is " << weight_step_size << " available step size obstacle is " << available_step_size_obstacle;
            // DLOG(INFO) << "step_size is " << step_size << " distance weight is" << distance_to_goal / distance_start_to_goal << " weight is " << weight_step_size << " available step size obstacle is " << available_step_size_obstacle;
        }
        else if (params_.number_of_step_size == 1)
        {
            step_size = params_.step_size;
        }
        // for other discrete step size, divide obstacle density into number of step_size ranges, and make max step size the obstacle detection range
        else
        {
            int coefficient;
            if (obstacle_density == 0)
            {
                coefficient = (1 - obstacle_density) * params_.number_of_step_size;
            }
            else
            {
                coefficient = (1 - obstacle_density) * params_.number_of_step_size + 1;
            }
            // DLOG(INFO) << "obstacle density is:" << obstacle_density << " coefficient is " << coefficient;
            // use obstacle detection range/ number of step size is not a good idea
            //  float basic_step_size = obstacle_detection_range / params_.number_of_step_size;
            float basic_step_size = params_.step_size;
            step_size = coefficient * basic_step_size;
        }
        // make sure max step size is  obstacle detection range
        if (step_size > obstacle_detection_range)
        {
            step_size = obstacle_detection_range;
        }
        // DLOG(INFO) << "FindOriginalStepSize: step size is " << step_size;
        DLOG_IF(INFO, step_size < params_.step_size) << "step size is " << step_size;
        return step_size;
    }

    void RRTPlanner::Rewire(RRT &rrt, Node3D &current, const float &radius)
    {
        if (CheckNodeOnRRT(rrt, current))
        {
            DLOG(INFO) << "current node is already on rrt. returning.";
            return;
        }
        std::vector<std::pair<Node3D, float>> neighbors_and_cost_so_far = FindNeighborsAndCostSoFar(rrt, current, radius);
        float min_cost = 100000, current_cost;
        std::pair<Node3D, float> best_parent_pair;
        bool flag = true;
        while (flag)
        {
            for (auto &element : neighbors_and_cost_so_far)
            {
                current_cost = element.second + Utility::GetDistance(element.first, current);
                if (current_cost < min_cost)
                {
                    min_cost = current_cost;
                    best_parent_pair = element;
                    // DLOG(INFO) << "set best parent pair.";
                }
            }
            std::shared_ptr<Node3D> parent_ptr = std::make_shared<Node3D>(best_parent_pair.first);
            current.SetPred(parent_ptr);
            // after found min cost and new parent, a collision check is also needed.
            if (configuration_space_ptr_->IsTraversable(current))
            {
                rrt.emplace_back(current);
                flag = false;
                // DLOG(INFO) << "great, no collision.";
            }
            else
            {
                // reset min cost
                min_cost = 100000;
                // DLOG(INFO) << "best parent is " << best_parent_pair.first.GetX() << " " << best_parent_pair.first.GetY() << " " << best_parent_pair.first.GetT();
                int index = Utility::FindIndex(neighbors_and_cost_so_far, best_parent_pair);
                // for (const auto &element : neighbors_and_cost_so_far)
                // {
                //     DLOG(INFO) << "node is " << element.first.GetX() << " " << element.first.GetY() << " " << element.first.GetT() << " cost is " << element.second;
                // }

                // DLOG(INFO) << "index is " << index << " size of vector is " << neighbors_and_cost_so_far.size();
                if (index >= 0)
                {
                    neighbors_and_cost_so_far.erase(neighbors_and_cost_so_far.begin() + index);
                }
            }
        }
    }

    bool RRTPlanner::CheckNodeOnRRT(const RRT &rrt, const Node3D &current)
    {
        if (params_.twoD_rrt)
        {
            for (const auto &element : rrt)
            {
                if (Utility::GetDistance(element, current) < 0.1)
                {
                    return true;
                }
            }
        }

        if (std::find(rrt.begin(), rrt.end(), current) != rrt.end())
        {
            // DLOG(INFO) << "node already inside rrt, do not add element!!";
            // DLOG(INFO) << "current node is " << current.GetX() << " " << current.GetY() << " " << Utility::ConvertRadToDeg(current.GetT());
            // for (const auto &element : rrt)
            // {
            //     if (element == current)
            //     {
            //         // DLOG(INFO) << "element is equal to current node";
            //     }
            //     else
            //     {
            //         // DLOG(INFO) << "element is not equal to current node";
            //     }
            //     // DLOG(INFO) << "element is " << element.GetX() << " " << element.GetY() << " " << Utility::ConvertRadToDeg(element.GetT());
            // }

            return true;
        }
        return false;
    }

    float RRTPlanner::FindNodeCostSoFar(const Node3D &current)
    {
        float cost_so_far = 0;
        std::shared_ptr<Node3D> node3d_ptr = std::make_shared<Node3D>(current);
        while (node3d_ptr != nullptr && current.GetPred() != nullptr)
        {
            cost_so_far += Utility::GetDistance(current, *current.GetPred());
            if (*node3d_ptr == start_)
            {
                break;
            }
            // DLOG(INFO) << "current node is " << node3d_ptr->GetX() << " " << node3d_ptr->GetY() << " and its pred is " << node3d_ptr->GetPred()->GetX() << " " << node3d_ptr->GetPred()->GetY();
            node3d_ptr = node3d_ptr->GetPred();
        }
        return cost_so_far;
    }

    std::vector<Node3D> RRTPlanner::FindNeighbors(const RRT &rrt, const Node3D &current, const float &radius)
    {
        std::vector<Node3D> neighbor_vec;
        if (CheckNodeOnRRT(rrt, current))
        {
            DLOG(INFO) << "current node is already on rrt. returning.";
            return neighbor_vec;
        }
        for (const auto &element : rrt)
        {
            if (Utility::GetDistance(current, element) < radius)
            {
                neighbor_vec.emplace_back(element);
            }
        }
        return neighbor_vec;
    }

    std::vector<std::pair<Node3D, float>> RRTPlanner::FindNeighborsAndCostSoFar(const RRT &rrt, const Node3D &current, const float &radius)
    {
        std::vector<std::pair<Node3D, float>> neighbors_and_cost_so_far;
        std::vector<Node3D> neighbors = FindNeighbors(rrt, current, radius);
        for (const auto &element : neighbors)
        {
            neighbors_and_cost_so_far.emplace_back(element, FindNodeCostSoFar(element));
        }
        return neighbors_and_cost_so_far;
    }

    void RRTPlanner::RRTConnectPlanner()
    {
        DLOG(INFO) << "in rrt connect planner.";
        std::pair<RRT, RRTFlag> rrt_start;
        rrt_start.first = InitializeRRT(start_);
        rrt_start.second = RRTFlag::Start;
        std::pair<RRT, RRTFlag> rrt_goal;
        rrt_goal.second = RRTFlag::Goal;
        rrt_goal.first = InitializeRRT(TreatNode(goal_));
        Node3D direction_node;
        int failure_counts = 0;
        while (1)
        {
            direction_node = FindDirectionNode(failure_counts, rrt_start.first, rrt_start.second);
            DLOG_IF(INFO, rrt_start.second == RRTFlag::Start) << "extend start rrt tree. direction node is " << direction_node.GetX() << " " << direction_node.GetY() << " " << Utility::ConvertRadToDeg(direction_node.GetT());
            DLOG_IF(INFO, rrt_start.second == RRTFlag::Goal) << "extend goal rrt tree. direction node is " << direction_node.GetX() << " " << direction_node.GetY() << " " << Utility::ConvertRadToDeg(direction_node.GetT());
            std::pair<Status, Node3D> status_successor = Extend(rrt_start.first, direction_node);
            if (status_successor.first != Status::Trapped)
            {
                Node3D target;
                if (rrt_goal.second == RRTFlag::Goal)
                {
                    target = TreatNode(status_successor.second);
                }
                else
                {
                    target = status_successor.second;
                }
                std::pair<Status, Node3D> status_successor_connect = Connect(rrt_goal.first, target);

                if (status_successor_connect.first == Status::Reached)
                {
                    DLOG(INFO) << "two rrt connected!";
                    SetPath(rrt_start, rrt_goal, status_successor_connect.second);
                    DLOG(INFO) << "total number of nodes explored " << rrt_start.first.size() + rrt_goal.first.size();
                    break;
                }
            }
            else
            {
                failure_counts++;
                // DLOG(INFO) << "failure counts plus one.";
            }
            Swap(rrt_start, rrt_goal);
        }
    }

    std::pair<Status, Node3D> RRTPlanner::Extend(RRT &rrt, const Node3D &target, const bool &flag)
    {
        DLOG(INFO) << "Extend to " << target.GetX() << " " << target.GetY() << " " << Utility::ConvertRadToDeg(target.GetT());
        std::pair<Status, Node3D> status_and_node;
        Node3D successor, closest_node;
        std::pair<float, float> step_size_steering_angle_pair;
        // 1. find closet node on rrt to this random node
        closest_node = FindClosestNode(rrt, target, flag);
        // 2. find step size and steering angle using random node
        step_size_steering_angle_pair = FindStepSizeAndSteeringAngle(closest_node, target);
        if (step_size_steering_angle_pair.first != 0)
        {
            successor = GenerateSuccessor(closest_node, step_size_steering_angle_pair);
            float distance_to_target = Utility::GetDistance(successor, target);
            if (!configuration_space_ptr_->IsTraversable(successor))
            {

                status_and_node.first = Status::Trapped;
                // DLOG(INFO) << "Trapped.";
            }
            else if (distance_to_target < 0.1)
            {
                status_and_node.first = Status::Reached;
                // DLOG(INFO) << "Reached.";
            }
            else
            {
                status_and_node.first = Status::Advanced;
                // DLOG(INFO) << "Advanced.";
            }
            if (status_and_node.first != Status::Trapped)
            {
                AddNodeToRRT(rrt, successor);
                if (params_.visualization)
                {
                    // VISUALIZATION DELAY
                    ros::Duration delay(0.03);
                    visualization_ptr_->publishNode3DPoses(successor);
                    visualization_ptr_->publishNode3DPose(successor);
                    delay.sleep();
                }
            }
        }
        else
        {
            // DLOG(INFO) << "step size is zero.";
            status_and_node.first = Status::Trapped;
            // DLOG(INFO) << "Trapped.";
        }
        // DLOG(INFO) << "status is " << status_and_node.first << " closest node is " << closest_node.GetX() << " " << closest_node.GetY() << " " << Utility::ConvertRadToDeg(closest_node.GetT()) << " step size is " << step_size_steering_angle_pair.first << " steering angle is " << Utility::ConvertRadToDeg(step_size_steering_angle_pair.second) << " successor is " << successor.GetX() << " " << successor.GetY() << " " << Utility::ConvertRadToDeg(successor.GetT());
        // DLOG_IF(INFO, (step_size_steering_angle_pair.second > Utility::ConvertDegToRad(30)) || (step_size_steering_angle_pair.second < Utility::ConvertDegToRad(-30)) || (step_size_steering_angle_pair.first < params_.step_size)) << "closest node is " << closest_node.GetX() << " " << closest_node.GetY() << " " << Utility::ConvertRadToDeg(closest_node.GetT()) << " step size is " << step_size_steering_angle_pair.first << " steering angle is " << Utility::ConvertRadToDeg(step_size_steering_angle_pair.second) << " successor is " << successor.GetX() << " " << successor.GetY() << " " << Utility::ConvertRadToDeg(successor.GetT());
        // DLOG_IF(INFO, status_and_node.first == Status::Trapped) << "status is in collision or step size is zero.";
        // DLOG_IF(INFO, status_and_node.first == Status::Reached) << "status is reached, target node reached.";
        // DLOG_IF(INFO, status_and_node.first == Status::Advanced) << "status is advanced, not reached and not in collision.";
        status_and_node.second = successor;
        return status_and_node;
    }

    std::pair<Status, Node3D> RRTPlanner::Connect(RRT &rrt, const Node3D &target)
    {
        DLOG(INFO) << "in connect. target is " << target.GetX() << " " << target.GetY() << " " << Utility::ConvertRadToDeg(target.GetT());
        std::pair<Status, Node3D> current_status_and_successor;
        while (1)
        {
            // DLOG(INFO) << "extending.";
            current_status_and_successor = Extend(rrt, target, true);
            if (current_status_and_successor.first != Status::Advanced)
            {
                break;
            }
        }
        return current_status_and_successor;
    }

    void RRTPlanner::Swap(std::pair<RRT, RRTFlag> &rrt_1, std::pair<RRT, RRTFlag> &rrt_2)
    {
        // DLOG(INFO) << "Swapping two rrt.";
        std::pair<RRT, RRTFlag> temp = rrt_2;
        rrt_2 = rrt_1;
        rrt_1 = temp;
    }

    RRT RRTPlanner::InitializeRRT(const Node3D &root)
    {
        RRT rrt;
        rrt.emplace_back(root);
        return rrt;
    }

    void RRTPlanner::SetPath(const std::pair<RRT, RRTFlag> &rrt_1, const std::pair<RRT, RRTFlag> &rrt_2, const Node3D &connect_point)
    {
        RRT rrt_start, rrt_goal;
        if (rrt_1.second == RRTFlag::Start)
        {
            rrt_start = rrt_1.first;
            rrt_goal = rrt_2.first;
        }
        else
        {
            rrt_start = rrt_2.first;
            rrt_goal = rrt_1.first;
        }
        Utility::Path3D path_to_start = TracePath(rrt_start, connect_point);
        Utility::Path3D path_to_goal = TracePath(rrt_goal, connect_point);
        // for (const auto &element : path_to_start)
        // {
        //     // DLOG(INFO) << "element in path_to_start is " << element.GetX() << " " << element.GetY();
        // }
        // for (const auto &element : path_to_goal)
        // {
        //     // DLOG(INFO) << "element in path_to_goal is " << element.GetX() << " " << element.GetY();
        // }
        // reverse path to start
        std::reverse(path_to_goal.begin(), path_to_goal.end());
        // DLOG(INFO) << "path size is " << path_.size();
        for (const auto &element : path_to_start)
        {
            path_.emplace_back(element);
        }
        for (const auto &element : path_to_goal)
        {
            path_.emplace_back(TreatNode(element));
        }

        // for (const auto &element : path_)
        // {
        //     DLOG(INFO) << "element in path is " << element.GetX() << " " << element.GetY();
        // }
    }

    Utility::Path3D RRTPlanner::TracePath(const RRT &rrt, const Node3D &current)
    {
        Utility::Path3D path;
        int current_index = Utility::FindIndex(rrt, current);
        // DLOG(INFO) << "index is " << current_index;
        path = TracePath(rrt, current_index);
        return path;
    }

    Node3D RRTPlanner::TreatNode(const Node3D &goal)
    {
        Node3D out = goal;
        out.setT(Utility::RadToZeroTo2P(out.GetT() + M_PI));
        return out;
    }
}