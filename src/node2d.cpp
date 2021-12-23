#include "node2d.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
using namespace HybridAStar;

//###################################################
//                                         IS ON GRID
//###################################################
bool Node2D::isOnGrid(const int width, const int height) const {
  return  x >= 0 && x < width && y >= 0 && y < height;
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################

std::vector<Node2D *> Node2D::CreateSuccessor(const int &possible_dir)
{
  std::vector<Node2D *> successor_vec;
  if (possible_dir == 4)
  {
    std::vector<int> delta = {-1, 1};
    for (uint i = 0; i < delta.size(); ++i)
    {
      int x_successor = x + delta[i];
      successor_vec.emplace_back(new Node2D(x_successor, y, g, 0, this));
    }
    for (uint i = 0; i < delta.size(); ++i)
    {
      int y_successor = y + delta[i];
      successor_vec.emplace_back(new Node2D(x, y_successor, g, 0, this));
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
        int x_successor = x + delta_x[i];
        int y_successor = y + delta_y[j];
        successor_vec.emplace_back(new Node2D(x_successor, y_successor, g, 0, this));
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
//                                 2D NODE COMPARISON
//###################################################
bool Node2D::operator == (const Node2D& rhs) const {
  return x == rhs.x && y == rhs.y;
}
