#include "a_star.h"

namespace HybridAStar
{
    AStar::AStar(const std::shared_ptr<CollisionDetection> &configuration_space_ptr, const std::shared_ptr<Visualize> &visualization_ptr, const int &possible_direction, const bool &visualization2D)
    {
        configuration_space_ptr_ = configuration_space_ptr;
        visualization_ptr_ = visualization_ptr;
        possible_direction_ = possible_direction;
        visualization2D_ = visualization2D;
    };
    void AStar::Initialize(const Node2D &start, const Node2D &goal)
    {
        start_ = start;
        goal_ = goal;
    }
    struct CompareNodes
    {
        /// Sorting 2D nodes by increasing C value - the total estimated cost
        bool operator()(const std::shared_ptr<Node2D> lhs, const std::shared_ptr<Node2D> rhs) const
        {
            return lhs->GetC() > rhs->GetC();
        }
    };
    //###################################################
    //                                        2D A*
    //###################################################
    float AStar::GetAStarCost(Node2D *nodes2D)
    {
        // PREDECESSOR AND SUCCESSOR INDEX
        int iPred, iSucc;
        float newG;
        int width = configuration_space_ptr_->GetMap()->info.width;
        int height = configuration_space_ptr_->GetMap()->info.height;
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
        start_.open();
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
            // set index
            iPred = nPred->setIdx(width);
            // _____________________________
            // LAZY DELETION of rewired node
            // if there exists a pointer this node has already been expanded
            if (nodes2D[iPred].isClosed())
            {
                // pop node from the open list and start with a fresh node
                openlist.pop();
                continue;
            }
            // _________________
            // EXPANSION OF NODE
            else if (nodes2D[iPred].isOpen())
            {
                // add node to closed list
                nodes2D[iPred].close();
                nodes2D[iPred].discover();

                // RViz visualization_ptr_
                if (visualization2D_)
                {
                    visualization_ptr_->publishNode2DPoses(*nPred);
                    visualization_ptr_->publishNode2DPose(*nPred);
                    //        d.sleep();
                }
                // remove node from open list
                openlist.pop();
                // _________
                // GOAL TEST
                if (*nPred == goal_)
                {
                    return nPred->GetG();
                }
                // ____________________
                // CONTINUE WITH SEARCH
                else
                {
                    // _______________________________
                    // CREATE POSSIBLE SUCCESSOR NODES
                    std::vector<std::shared_ptr<Node2D>> successor_vec = CreateSuccessor(*nPred, possible_direction_);
                    for (uint i = 0; i < successor_vec.size(); ++i)
                    {
                        // create possible successor
                        nSucc = successor_vec[i];
                        // set index of the successor
                        iSucc = nSucc->setIdx(width);
                        // ensure successor is on grid ROW MAJOR
                        // ensure successor is not blocked by obstacle
                        // ensure successor is not on closed list
                        if (configuration_space_ptr_->IsTraversable(nSucc) && !nodes2D[iSucc].isClosed())
                        {
                            // calculate new G value
                            // nSucc->updateG();
                            UpdateCostSoFar(*nSucc);
                            newG = nSucc->GetG();
                            // if successor not on open list or g value lower than before put it on open list
                            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].GetG())
                            {
                                // calculate the H value
                                // nSucc->UpdateHeuristic(goal);
                                UpdateHeuristic(*nSucc);
                                // put successor on open list
                                nSucc->open();
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
        return 1000;
    }

    std::vector<std::shared_ptr<Node2D>> AStar::CreateSuccessor(const Node2D &pred, const int &possible_dir)
    {
        std::vector<std::shared_ptr<Node2D>> successor_vec;
        std::shared_ptr<Node2D> pred_ptr = std::make_shared<Node2D>(pred);
        if (possible_dir == 4)
        {
            std::vector<int> delta = {-1, 1};
            for (uint i = 0; i < delta.size(); ++i)
            {
                int x_successor = pred.GetX() + delta[i];
                std::shared_ptr<Node2D> temp = std::make_shared<Node2D>(Node2D(x_successor, pred.GetY(), pred.GetG(), 0, pred_ptr));
                successor_vec.emplace_back(temp);
            }
            for (uint i = 0; i < delta.size(); ++i)
            {
                int y_successor = pred.GetY() + delta[i];
                std::shared_ptr<Node2D> temp = std::make_shared<Node2D>(Node2D(pred.GetX(), y_successor, pred.GetG(), 0, pred_ptr));
                successor_vec.emplace_back(temp);
            }
        }
        else if (possible_dir == 8)
        {
            std::vector<int> delta_x = {-1, 0, 1};
            std::vector<int> delta_y = {-1, 0, 1};
            for (uint i = 0; i < delta_x.size(); ++i)
            {
                for (uint j = 0; j < delta_y.size(); ++j)
                {
                    if (delta_x[i] == 0 && delta_y[j] == 0)
                    {
                        continue;
                    }
                    int x_successor = pred.GetX() + delta_x[i];
                    int y_successor = pred.GetY() + delta_y[j];
                    std::shared_ptr<Node2D> temp = std::make_shared<Node2D>(Node2D(x_successor, y_successor, pred.GetG(), 0, pred_ptr));
                    successor_vec.emplace_back(temp);
                }
            }
        }
        else
        {
            DLOG(WARNING) << "Wrong possible_dir!!!";
        }
        return successor_vec;
    }
    //###################################################
    //                                    update cost so far
    //###################################################
    void AStar::UpdateCostSoFar(Node2D &node)
    {
        float cost_so_far;
        cost_so_far += Utility::GetDistance(node, *node.GetPred());
        node.SetG(cost_so_far);
    }
    //###################################################
    //                                    update cost so far
    //###################################################
    void AStar::UpdateHeuristic(Node2D &current)
    {
        current.SetH(Utility::GetDistance(current, goal_));
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
            // DLOG(INFO) << "current node is " << node->GetX() << " " << node->GetY() << " and its pred is " << node->GetPred()->GetX() << " " << node->GetPred()->GetY();
            node2d_ptr = node2d_ptr->GetPred();
        }
        std::reverse(path_.begin(), path_.end());
    }
}