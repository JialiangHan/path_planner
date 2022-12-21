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
 *  Author:  dengpw
 *********************************************************************/
#include "node2d.h"
#include <iostream>
namespace HybridAStar
{

  float Node2D::updateCostSofar(Node2D const *parent)
  {
    // float g;
    // if ((abs(x - partent->x) + abs(y - partent->y)) == 2)
    // {
    //   g = 1.4142;
    // }
    // else
    //   g = 1;
    // return g + partent->getCostSofar() + 0.1 * cost;
    g = g + movementCost(*parent) + parent->getCostSofar() + 0.1 * map_cost_;
    d = true;
    return g;
  }

  std::unordered_map<int, std::shared_ptr<Node2D>> GridSearch::GenerateDpMap(
      const double goal_x, const double goal_y,
      costmap_2d::Costmap2D *costmap)
  {
    DLOG(INFO) << "in GridSearch::GenerateDpMap.";
    const unsigned char *charMap = costmap->getCharMap();
    int counter = 0;
    int cells_x, cells_y;
    cells_x = costmap->getSizeInCellsX();
    cells_y = costmap->getSizeInCellsY();
    // float resolution = costmap->getResolution();
    std::priority_queue<std::pair<int, double>,
                        std::vector<std::pair<int, double>>, cmp>
        open_pq;
    std::unordered_map<int, std::shared_ptr<Node2D>> open_set;
    std::unordered_map<int, std::shared_ptr<Node2D>> dp_map_;
    std::shared_ptr<Node2D> goal_node = std::make_shared<Node2D>(goal_x, goal_y);
    goal_node->setX(goal_x);
    goal_node->setY(goal_y);
    goal_node->setCostSofar(0);
    open_set.emplace(goal_node->setIdx(cells_x), goal_node);
    open_pq.emplace(goal_node->setIdx(cells_x), goal_node->getCostSofar());
    while (!open_pq.empty())
    {
      ++counter;
      int id = open_pq.top().first;
      open_pq.pop();
      std::shared_ptr<Node2D> current_node = open_set[id];
      dp_map_.emplace(current_node->setIdx(cells_x), current_node);
      DLOG(INFO) << "put current node " << current_node->getX() << " " << current_node->getY() << " "
                 << "index is " << current_node->setIdx(cells_x) << " into dpmap;";
      std::vector<std::shared_ptr<Node2D>> adjacent_nodes =
          getAdjacentPoints(cells_x, cells_y, charMap, current_node);
      for (std::vector<std::shared_ptr<Node2D>>::iterator
               it = adjacent_nodes.begin();
           it != adjacent_nodes.end(); ++it)
      {
        std::shared_ptr<Node2D> next_node = *it;
        if (dp_map_.find(next_node->setIdx(cells_x)) != dp_map_.end())
        {
          continue;
        }
        if (open_set.find(next_node->setIdx(cells_x)) != open_set.end())
        {
          if (open_set[next_node->setIdx(cells_x)]->getCostSofar() > next_node->getCostSofar())
          {
            open_set[next_node->setIdx(cells_x)]->setMapCost(next_node->getCostSofar());
            open_set[next_node->setIdx(cells_x)]->setSmartPtrPred(current_node);
          }
        }
        else
        {
          // ++counter;
          next_node->setSmartPtrPred(current_node);
          open_set.emplace(next_node->setIdx(cells_x), next_node);
          open_pq.emplace(next_node->setIdx(cells_x), next_node->getCostSofar());
        }
      }
    }
    // printf("X %d y %d  explored node num is : %d \n", cells_x, cells_y, counter);
    return dp_map_;
  }

  std::vector<std::shared_ptr<Node2D>> GridSearch::getAdjacentPoints(int cells_x,
                                                                     int cells_y, const unsigned char *charMap, std::shared_ptr<Node2D> point)
  {
    std::vector<std::shared_ptr<Node2D>> adjacentNodes;
    float g = 0;

    for (int x = point->getX() - 1; x <= point->getX() + 1; ++x)
    {
      for (int y = point->getY() - 1; y <= point->getY() + 1; ++y)
      {
        if (charMap[x + y * cells_x] < 250 && x < cells_x && y < cells_y && x > 0 && y > 0)
        {
          std::shared_ptr<Node2D> node = std::make_shared<Node2D>(x, y);
          if ((abs(x - point->getX()) + abs(y - point->getY())) == 2)
          {
            g = 1.4142;
          }
          else
          {
            g = 1;
          }
          node->setCostSofar(point->getCostSofar() + 0.1 * charMap[x + y * cells_x] + g);
          node->setX(x);
          node->setY(y);
          adjacentNodes.emplace_back(node);
        }
      }
    } // end of for
    return adjacentNodes;
  }

} // namespace hybrid_astar_planner
