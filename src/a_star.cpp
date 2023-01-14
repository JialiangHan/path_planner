#include "a_star.h"

namespace HybridAStar
{

    void AStar::Initialize(nav_msgs::OccupancyGrid::Ptr map)
    {
        // update the configuration space with the current map
        // LOG(INFO) << "a star initializing";
        configuration_space_ptr_->UpdateGrid(map);

        // LOG(INFO) << " a star initialized done.   ";
    }
    struct CompareNodes
    {
        /// Sorting 2D nodes by increasing C value - the total estimated cost
        bool operator()(const std::shared_ptr<Node2D> lhs, const std::shared_ptr<Node2D> rhs) const
        {
            return lhs->getTotalCost() > rhs->getTotalCost();
        }
        // Sorting 2D nodes by increasing C value - the total estimated cost
        bool operator()(const Node2D *lhs, const Node2D *rhs) const
        {
            return lhs->getTotalCost() > rhs->getTotalCost();
        }
    };

    //###################################################
    //                                        2D A*
    //###################################################
    float AStar::GetAStarCost(Node2D *nodes2D, const Node2D &start, const Node2D &goal, const bool &in_hybrid_a)
    {
        start_ = start;
        goal_ = goal;
        // LOG(INFO) << "GetAStarCost in: start is " << start.getX() << " " << start.getY() << " goal is " << goal.getX() << " " << goal.getY();
        // PREDECESSOR AND SUCCESSOR INDEX
        int iPred, iSucc;
        float newG;
        int width = configuration_space_ptr_->GetMap()->info.width;
        int height = configuration_space_ptr_->GetMap()->info.height;
        // number of nodes explored
        int number_nodes_explored = 0;
        // reset the open and closed list
        for (int i = 0; i < width * height; ++i)
        {
            nodes2D[i].reset();
        }
        // VISUALIZATION DELAY
        ros::Duration d(0.001);
        boost::heap::binomial_heap<std::shared_ptr<Node2D>, boost::heap::compare<CompareNodes>> openlist;
        // update h value
        UpdateHeuristic(start_);
        // mark start as open
        start_.setOpenSet();
        // push on priority queue
        std::shared_ptr<Node2D> start_ptr = std::make_shared<Node2D>(start_);
        openlist.push(start_ptr);
        iPred = start_.setIdx(width, height, resolution_, origin_x_, origin_y_);
        nodes2D[iPred] = start_;
        // NODE POINTER
        std::shared_ptr<Node2D> nPred;
        std::shared_ptr<Node2D> nSucc;

        // continue until O empty
        while (!openlist.empty())
        {
            // pop node with lowest cost from priority queue
            nPred = openlist.top();
            // LOG(INFO) << "current node  is " << nPred->getX() << " " << nPred->getY();
            number_nodes_explored++;
            // set index
            iPred = nPred->setIdx(width, height, resolution_, origin_x_, origin_y_);
            // _____________________________
            // LAZY DELETION of rewired node
            // if there exists a pointer this node has already been expanded
            if (nodes2D[iPred].isClosedSet())
            {
                // pop node from the open list and start with a fresh node
                openlist.pop();
                continue;
            }
            // _________________
            // EXPANSION OF NODE
            else if (nodes2D[iPred].isOpenSet())
            {
                // add node to closed list
                nodes2D[iPred].setClosedSet();
                nodes2D[iPred].discover();

                // RViz visualization_ptr_
                if (visualization2D_ && !in_hybrid_a)
                {
                    // LOG(INFO) << "in publishing";
                    visualization_ptr_->publishNode2DPoses((*nPred));
                    visualization_ptr_->publishNode2DPose((*nPred));
                    d.sleep();
                }
                // remove node from open list
                openlist.pop();
                // _________
                // GOAL TEST

                if (Utility::IsCloseEnough(*nPred, goal_, params_.goal_range))
                {
                    // DLOG(INFO) << "goal reached, return cost so far.";
                    // DLOG(INFO) << "GetAStarCost out.";
                    if (in_hybrid_a)
                    {
                        return nPred->getCostSofar();
                    }

                    TracePath(nPred);
                    // LOG(INFO) << "number of nodes explored is " << number_nodes_explored;
                    return nPred->getCostSofar();
                }
                // ____________________
                // CONTINUE WITH SEARCH
                else
                {
                    // CREATE POSSIBLE SUCCESSOR NODES
                    std::vector<std::shared_ptr<Node2D>> successor_vec = CreateSuccessor(*nPred, params_.possible_direction);
                    for (uint i = 0; i < successor_vec.size(); ++i)
                    {
                        // LOG(INFO) << "current successor is " << successor_vec[i]->getX() << " " << successor_vec[i]->getY();
                        // create possible successor
                        nSucc = successor_vec[i];
                        // set index of the successor
                        iSucc = nSucc->setIdx(width, height, resolution_, origin_x_, origin_y_);
                        // ensure successor is on grid ROW MAJOR
                        // ensure successor is not blocked by obstacle
                        // ensure successor is not on closed list
                        if (configuration_space_ptr_->IsTraversable(nSucc) && !nodes2D[iSucc].isClosedSet())
                        {
                            // LOG(INFO) << "not in collision.";
                            // calculate new G value
                            UpdateCostSoFar(*nSucc);
                            // LOG(INFO) << "update cost so far done.";
                            newG = nSucc->getCostSofar();
                            // LOG(INFO) << "getCostSofar done.";
                            // if successor not on open list or g value lower than before put it on open list
                            if (!nodes2D[iSucc].isOpenSet() || newG < nodes2D[iSucc].getCostSofar())
                            {
                                // calculate the H value
                                UpdateHeuristic(*nSucc);
                                // LOG(INFO) << "UpdateHeuristic done.";
                                // put successor on open list
                                nSucc->setOpenSet();
                                // LOG(INFO) << "setOpenSet done.";
                                // LOG(INFO) << "iSucc is " << iSucc;
                                nodes2D[iSucc] = *nSucc;
                                // LOG(INFO) << "iSucc is " << iSucc;
                                std::shared_ptr<Node2D> nSucc_ptr = std::make_shared<Node2D>(nodes2D[iSucc]);
                                openlist.push(nSucc_ptr);
                                // LOG(INFO) << "current successor not in open list and new cost so far is smaller than old one, put in into openlist.";
                            }
                        }
                        else
                        {
                            // LOG(INFO) << "current successor is in collision or is in close set.";
                        }
                    }
                }
            }
        }
        // return large number to guide search away
        // DLOG(INFO) << "GetAStarCost out.";
        // LOG(INFO) << "GetAStarCost in: start is " << start.getX() << " " << start.getY() << " goal is " << goal.getX() << " " << goal.getY() << ". open list is empty, end a star. number of nodes explored is " << number_nodes_explored;
        return 1000;
    }

    std::vector<std::shared_ptr<Node2D>> AStar::CreateSuccessor(const Node2D &pred, const uint &possible_dir)
    {
        // DLOG(INFO) << "CreateSuccessor in:";
        std::vector<std::shared_ptr<Node2D>> successor_vec;
        float step_size = FindStepSize(pred);
        successor_vec = CreateSuccessor(pred, possible_dir, step_size);
        return successor_vec;
    }

    std::vector<std::shared_ptr<Node2D>> AStar::CreateSuccessor(const Node2D &pred, const uint &possible_dir, const float &step_size)
    {
        // LOG(INFO) << "CreateSuccessor in:";
        std::vector<std::shared_ptr<Node2D>> successor_vec;
        std::shared_ptr<Node2D> pred_ptr = std::make_shared<Node2D>(pred);
        float x_successor, y_successor;
        if (possible_dir == 4)
        {
            std::vector<float> delta = {-step_size, step_size};
            for (uint i = 0; i < delta.size(); ++i)
            {
                x_successor = pred.getX() + delta[i];
                if (!configuration_space_ptr_->IsOnGrid(x_successor, pred.getY()))
                {
                    continue;
                }
                std::shared_ptr<Node2D> temp = std::make_shared<Node2D>(Node2D(x_successor, pred.getY(), pred.getCostSofar(), 0, nullptr, pred_ptr));
                successor_vec.emplace_back(temp);
            }
            for (uint i = 0; i < delta.size(); ++i)
            {
                y_successor = pred.getY() + delta[i];
                if (!configuration_space_ptr_->IsOnGrid(pred.getX(), y_successor))
                {
                    continue;
                }
                std::shared_ptr<Node2D> temp = std::make_shared<Node2D>(Node2D(pred.getX(), y_successor, pred.getCostSofar(), 0, nullptr, pred_ptr));
                successor_vec.emplace_back(temp);
            }
        }
        else if (possible_dir == 8)
        {
            std::vector<float> delta = {-step_size, 0, step_size};
            for (uint i = 0; i < delta.size(); ++i)
            {
                for (uint j = 0; j < delta.size(); ++j)
                {
                    if (delta[i] == 0 && delta[j] == 0)
                    {
                        continue;
                    }
                    x_successor = pred.getX() + delta[i];
                    y_successor = pred.getY() + delta[j];
                    if (!configuration_space_ptr_->IsOnGrid(x_successor, y_successor))
                    {
                        // LOG(INFO) << "successor is not on grid!!!";
                        continue;
                    }
                    std::shared_ptr<Node2D> temp = std::make_shared<Node2D>(Node2D(x_successor, y_successor, pred.getCostSofar(), 0, nullptr, pred_ptr));
                    successor_vec.emplace_back(temp);
                }
            }
        }
        else
        {
            DLOG(WARNING) << "WARNING: Wrong possible_dir!!!";
        }
        // for (const auto &element : successor_vec)
        // {
        //     LOG(INFO) << "successor created is " << element->getX() << " " << element->getY();
        // }
        // LOG(INFO) << "CreateSuccessor out.";
        return successor_vec;
    }
    //###################################################
    //                                    update cost so far
    //###################################################
    void AStar::UpdateCostSoFar(Node2D &node)
    {
        float cost_so_far = node.getCostSofar() + Utility::GetDistance(node, *node.getSmartPtrPred());
        node.setCostSofar(cost_so_far);
    }
    // ###################################################
    //                                     update cost so far
    // ###################################################
    //  checked
    void AStar::UpdateHeuristic(Node2D &current)
    {
        float distance_to_goal = Utility::GetDistance(current, goal_);
        // LOG(INFO) << "current node is " << current.getX() << " " << current.getY() << " goal is " << goal_.getX() << " " << goal_.getY() << " distance to goal is " << distance_to_goal;

        current.setCostToGo(distance_to_goal);
    }
    //###################################################
    //                                   trace path
    //###################################################
    void AStar::TracePath(std::shared_ptr<Node2D> node2d_ptr)
    {
        path_.clear();
        while (node2d_ptr != nullptr)
        {
            path_.emplace_back(*node2d_ptr);
            if (*node2d_ptr == start_)
            {
                break;
            }
            // DLOG(INFO) << "current node is " << node->getX() << " " << node->getY() << " and its pred is " << node->getPred()->getX() << " " << node->getPred()->getY();
            node2d_ptr = node2d_ptr->getSmartPtrPred();
        }
        std::reverse(path_.begin(), path_.end());
    }

    Utility::Path3D AStar::GetPath(Node3D &start, Node3D &goal,
                                   Node2D *nodes2D)
    {
        Utility::Path3D path_3d;
        Node2D start_2d, goal_2d;
        Utility::TypeConversion(start, start_2d);
        Utility::TypeConversion(goal, goal_2d);
        GetAStarCost(nodes2D, start_2d, goal_2d);
        Utility::TypeConversion(path_, path_3d);
        return path_3d;
    }

    Utility::Path3D AStar::GetPath(Node2D &start, Node2D &goal,
                                   Node2D *nodes2D)
    {
        Utility::Path3D path_3d;
        GetAStarCost(nodes2D, start, goal);
        Utility::TypeConversion(path_, path_3d);
        return path_3d;
    }

    float AStar::FindStepSize(const Node2D &current_node)
    {
        float step_size;
        if (!params_.use_adaptive_step_size_in_a_star)
        {
            return resolution_;
        }

        float min_distance = 10000;
        HybridAStar::Node3D node_3d;
        Utility::TypeConversion(current_node, node_3d);
        std::vector<std::pair<float, Utility::AngleRange>> step_size_angle_range =
            configuration_space_ptr_->FindFreeAngleRangeAndObstacleAngleRange(node_3d, false);
        for (const auto &element : step_size_angle_range)
        {
            if (min_distance > element.first)
            {
                min_distance = element.first;
            }
        }
        step_size = min_distance;
        // DLOG(INFO) << "step size is " << step_size;
        return step_size;
    }

    bool AStar::calculatePath(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped> &plan,
                              ros::Publisher &pub, visualization_msgs::MarkerArray &pathNodes)
    {
        Utility::TypeConversion(start, start_);
        Utility::TypeConversion(goal, goal_);
        Node2D *nodes2D = new Node2D[cells_x * cells_y]();
        Utility::Path3D path3d = GetPath(start_, goal_, nodes2D);
        Utility::TypeConversion(path3d, plan);
        if (plan.size() != 0)
        {
            delete[] nodes2D;
            return true;
        }
        else
        {
            delete[] nodes2D;
            return false;
        }
    }
}