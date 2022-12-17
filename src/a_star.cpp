#include "a_star.h"

namespace HybridAStar
{

    void AStar::Initialize(nav_msgs::OccupancyGrid::Ptr map)
    {
        // update the configuration space with the current map
        //  DLOG(INFO) << "hybrid a star initializing";
        configuration_space_ptr_->UpdateGrid(map);
        // DLOG(INFO) << "hybrid a star initialized done.   ";
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
        // DLOG(INFO) << "GetAStarCost in:";
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
        // start.UpdateHeuristic(goal);
        UpdateHeuristic(start_);
        // mark start as open
        start_.setOpenSet();
        // push on priority queue
        std::shared_ptr<Node2D> start_ptr = std::make_shared<Node2D>(start_);
        openlist.push(start_ptr);
        iPred = start_.setIdx(width);
        nodes2D[iPred] = start_;
        // NODE POINTER
        std::shared_ptr<Node2D> nPred;
        std::shared_ptr<Node2D> nSucc;

        // continue until O empty
        while (!openlist.empty())
        {
            // pop node with lowest cost from priority queue
            nPred = openlist.top();
            number_nodes_explored++;
            // set index
            iPred = nPred->setIdx(width);
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
                    DLOG(INFO) << "in publishing";
                    visualization_ptr_->publishNode2DPoses((*nPred));
                    visualization_ptr_->publishNode2DPose((*nPred));
                    d.sleep();
                }
                // remove node from open list
                openlist.pop();
                // _________
                // GOAL TEST
                if (*nPred == goal_)
                {
                    // DLOG(INFO) << "goal reached, return cost so far.";
                    // DLOG(INFO) << "GetAStarCost out.";
                    if (in_hybrid_a)
                    {
                        return nPred->getCostSofar();
                    }

                    TracePath(nPred);
                    LOG(INFO) << "number of nodes explored is " << number_nodes_explored;
                    return nPred->getCostSofar();
                }
                // ____________________
                // CONTINUE WITH SEARCH
                else
                {
                    // _______________________________
                    // CREATE POSSIBLE SUCCESSOR NODES
                    std::vector<std::shared_ptr<Node2D>> successor_vec = CreateSuccessor(*nPred, params_.possible_direction);
                    for (uint i = 0; i < successor_vec.size(); ++i)
                    {
                        // create possible successor
                        nSucc = successor_vec[i];
                        // set index of the successor
                        iSucc = nSucc->setIdx(width);
                        // ensure successor is on grid ROW MAJOR
                        // ensure successor is not blocked by obstacle
                        // ensure successor is not on closed list
                        if (configuration_space_ptr_->IsTraversable(nSucc) && !nodes2D[iSucc].isClosedSet())
                        {
                            // calculate new G value
                            // nSucc->updateG();
                            UpdateCostSoFar(*nSucc);
                            newG = nSucc->getCostSofar();
                            // if successor not on open list or g value lower than before put it on open list
                            if (!nodes2D[iSucc].isOpenSet() || newG < nodes2D[iSucc].getCostSofar())
                            {
                                // calculate the H value
                                // nSucc->UpdateHeuristic(goal);
                                UpdateHeuristic(*nSucc);
                                // put successor on open list
                                nSucc->setOpenSet();
                                nodes2D[iSucc] = *nSucc;
                                std::shared_ptr<Node2D> nSucc_ptr = std::make_shared<Node2D>(nodes2D[iSucc]);
                                openlist.push(nSucc_ptr);
                            }
                        }
                    }
                }
            }
        }
        // return large number to guide search away
        // DLOG(INFO) << "GetAStarCost out.";
        DLOG(INFO) << "open list is empty, end a star. number of nodes explored is " << number_nodes_explored;
        return 1000;
    }

    std::vector<std::shared_ptr<Node2D>> AStar::CreateSuccessor(const Node2D &pred, const uint &possible_dir)
    {
        // DLOG(INFO) << "CreateSuccessor in:";
        std::vector<std::shared_ptr<Node2D>> successor_vec;
        int step_size = FindStepSize(pred);
        successor_vec = CreateSuccessor(pred, possible_dir, step_size);
        return successor_vec;
    }

    std::vector<std::shared_ptr<Node2D>> AStar::CreateSuccessor(const Node2D &pred, const uint &possible_dir, const int &step_size)
    {
        // DLOG(INFO) << "CreateSuccessor in:";
        std::vector<std::shared_ptr<Node2D>> successor_vec;
        std::shared_ptr<Node2D> pred_ptr = std::make_shared<Node2D>(pred);
        int x_successor, y_successor;
        if (possible_dir == 4)
        {
            std::vector<int> delta = {-step_size, step_size};
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
            std::vector<int> delta = {-step_size, 0, step_size};
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
        //     DLOG(INFO) << "successor created is " << element->getX() << " " << element->getY();
        // }
        // DLOG(INFO) << "CreateSuccessor out.";
        return successor_vec;
    }
    //###################################################
    //                                    update cost so far
    //###################################################
    void AStar::UpdateCostSoFar(Node2D &node)
    {
        float cost_so_far = 0;
        cost_so_far = node.getCostSofar() + Utility::GetDistance(node, *node.getSmartPtrPred());
        node.setCostSofar(cost_so_far);
    }
    //###################################################
    //                                    update cost so far
    //###################################################
    void AStar::UpdateHeuristic(Node2D &current)
    {
        float distance_to_goal = Utility::GetDistance(current, goal_);
        // DLOG(INFO) << "current node is " << current.getX() << " " << current.getY() << " distance to goal is " << distance_to_goal;

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

    int AStar::FindStepSize(const Node2D &current_node)
    {
        int step_size;
        if (!params_.use_adaptive_step_size_in_a_star)
        {
            return 1;
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
        step_size = std::round(min_distance);
        // DLOG(INFO) << "step size is " << step_size;
        return step_size;
    }

    bool AStar::calculatePath(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped> &plan,
                              ros::Publisher &pub, visualization_msgs::MarkerArray &pathNodes)
    {

        const unsigned char *charMap = costmap->getCharMap();

        int counter = 0; // 记录程序搜寻节点次数
        // float resolution = costmap->getResolution(); // 获取代价图的分辨率

        // 使用boost库中的二项堆，优化优先队列的性能
        boost::heap::binomial_heap<Node2D *, boost::heap::compare<CompareNodes>> openSet;
        unsigned int startx, starty, goalx, goaly;
        // 坐标转换，将世界坐标转换为costmap使用的绝对坐标
        costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty);
        costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly);
        Node2D *pathNode2D = new Node2D[cells_x * cells_y](); // 与代价图等大的2Dnode栅格节点

        // 设置开始节点与结束节点指针，指针指向其对应2D节点图中的点位
        Node2D *startPose = &pathNode2D[startx * cells_y + starty];
        Node2D *goalPose = &pathNode2D[goalx * cells_y + goaly];

        // 设置起始节点和结束节点的坐标，以便后续进行比较计算
        goalPose->setX(goalx);
        goalPose->setY(goaly);
        startPose->setX(startx);
        startPose->setY(starty);
        startPose->setCostSofar(0);
        openSet.push(startPose);
        startPose->setOpenSet();

        Node2D *tmpStart;
        while (openSet.size())
        {
            ++counter;
            tmpStart = openSet.top();
            openSet.pop();
            // 如果找到目标点则返回
            if (tmpStart->getX() == goalPose->getX() && tmpStart->getY() == goalPose->getY())
            {
                LOG(INFO) << "got a plan"  ;
                nodeToPlan(tmpStart, plan);
                LOG(INFO) << counter  ;
                delete[] pathNode2D;
                return true;
            }
            std::vector<Node2D *> adjacentNodes = getAdjacentPoints(cells_x, cells_y, charMap, pathNode2D, tmpStart);
            tmpStart->setClosedSet();

            // 下面正式开始A*算法的核心搜索部分
            for (std::vector<Node2D *>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it)
            {
                Node2D *point = *it;
                float g;
                if (!point->isClosedSet())
                {
                    g = point->updateCostSofar(tmpStart);
                    // 在此拓展点为初次被探索时，设置此点的G值，设置其父节点。或是以其他路径到达此点的G值更小时，重新设置此点的父节点
                    if (!point->isOpenSet() || (g < point->getCostSofar()))
                    {
                        point->setPred(tmpStart);
                        point->setCostSofar(g);
                        if (!point->isOpenSet())
                        {
                            point->updateHeuristic(goalPose); // 计算此点距离目标节点距离（作为启发值）
                            point->setOpenSet();              // 将此拓展点加入开集合中
                        }
                        openSet.push(point);
                    }
                }
            }
        }

        delete[] pathNode2D; // 删除产生的Node2D节点
        return false;        // 搜索失败
    }

    std::vector<Node2D *> AStar::getAdjacentPoints(int cells_x, int cells_y, const unsigned char *charMap, Node2D *pathNode2D, Node2D *point)
    {
        std::vector<Node2D *> adjacentNodes;
        for (int x = point->getX() - 1; x <= point->getX() + 1; ++x)
        {
            for (int y = point->getY() - 1; y <= point->getY() + 1; ++y)
            {
                if (charMap[x + y * cells_x] <= 1)
                {
                    pathNode2D[x * cells_y + y].setX(x);
                    pathNode2D[x * cells_y + y].setY(y);
                    adjacentNodes.push_back(&pathNode2D[x * cells_y + y]);
                }
            }
        } // end of for

        return adjacentNodes;
    }

    void AStar::nodeToPlan(Node2D *node, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        Node2D *tmpPtr = node;
        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header.stamp = ros::Time::now();
        // 参数后期处理，发布到RViz上进行可视化
        tmpPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        std::vector<geometry_msgs::PoseStamped> replan;
        while (tmpPtr != nullptr)
        {
            costmap->mapToWorld(tmpPtr->getX(), tmpPtr->getY(), tmpPose.pose.position.x, tmpPose.pose.position.y);
            tmpPose.header.frame_id = frame_id_;
            replan.push_back(tmpPose);
            tmpPtr = tmpPtr->getPred();
        }
        int size = replan.size();
        for (int i = 0; i < size; ++i)
        {
            plan.push_back(replan[size - i - 1]);
        }
    }
}