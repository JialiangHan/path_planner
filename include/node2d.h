#pragma once

#include <cmath>
#include <unordered_map>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include "glog/logging.h"
#include "gflags/gflags.h"
#include <iomanip>
namespace HybridAStar {

/*!
   \brief A two dimensional node class used for the holonomic with obstacles heuristic.

   Each node has a unique discrete position (x,y).
*/
class Node2D {
 public:
  /// The default constructor for 2D array initialization.
   Node2D() : Node2D(0, 0, 0, 0, nullptr) {}
   /// Constructor for a node with the given arguments
   Node2D(float _x, float _y, float _g = 0, float _h = 0, Node2D *_pred_ptr = nullptr, std::shared_ptr<Node2D> _pred_shared_ptr = nullptr) : x(_x), y(_y), g(_g), h(_h), idx(-1), o(false), c(false), d(false), pred_ptr_(_pred_ptr), pred_shared_ptr_(_pred_shared_ptr)
   {
      }
   // GETTER METHODS
   /// get the x position
   float getX() const { return x; }
   /// get the y position
   float getY() const { return y; }
   /// get the cost-so-far (real value)
   float getCostSofar() const { return g; }
   /// get the cost-to-come (heuristic value)
   float getCostToGo() const { return h; }
   /// get the total estimated cost
   float getTotalCost() const { return g + h; }

   /// get the index of the node in the 2D array
   int getIdx() const { return idx; }
   /// determine whether the node is open
   bool isOpenSet() const { return o; }
   /// determine whether the node is closed
   bool isClosedSet() const { return c; }
   /// determine whether the node is discovered
   bool isDiscovered() const { return d; }
   /// get a pointer to the predecessor
   Node2D *getPred() const { return pred_ptr_; }

   // SETTER METHODS
   /// set the x position
   void setX(const float &x) { this->x = x; }
   /// set the y position
   void setY(const float &y) { this->y = y; }
   /// set the cost-so-far (real value)
   void setCostSofar(const float &g) { this->g = g; }
   /// set the cost-to-come (heuristic value)
   void setCostToGo(const float &h) { this->h = h; }

   /// set and get the index of the node in the 2D array
   int setIdx(int width, int height, const float &resolution, float origin_x, float origin_y)
   {
     this->idx = std::floor(std::round(((y - origin_y) / resolution) * 1000) / 1000) * width + std::floor(std::round(((x - origin_x) / resolution) * 1000) / 1000);
     LOG_IF(FATAL, idx >= width * height) << "idx larger than width*height!!! idx is " << idx << " width is " << width << " height is " << height << " resolution is " << resolution << " origin x is " << origin_x << " origin y is " << origin_y;
     LOG_IF(FATAL, idx < 0) << "idx smaller than zero!!! idx is " << idx << " width is " << width << " height is " << height << " resolution is " << resolution << " origin x is " << origin_x << " origin y is " << origin_y;
     //  LOG(INFO) << "idx is " << idx << " width is " << width << " height is " << height << " resolution is " << resolution << " origin x is " << origin_x << " origin y is " << origin_y << " x is " << x << " y is " << y << " std::floor(std::round(((y - origin_y) / resolution) * 1000) / 1000)   is " << std::floor(std::round(((y - origin_y) / resolution) * 1000) / 1000) << " std::floor(std::round(((x - origin_x) / resolution) * 1000) / 1000) is " << std::floor(std::round(((x - origin_x) / resolution) * 1000) / 1000) << " ((y - origin_y) / resolution)  is " << std::fixed << std::setprecision(7) << ((y - origin_y) / resolution) << " ((x - origin_x) / resolution) is " << std::fixed << std::setprecision(7) << ((x - origin_x) / resolution);
     return idx;
   }
   /// open the node
   void setOpenSet()
   {
     o = true;
     c = false;
   }
   /// close the node
   void setClosedSet()
   {
     c = true;
     o = false;
   }
   /// set the node neither open nor closed
   void reset()
   {
     c = false;
     o = false;
   }
   /// discover the node
   void discover() { d = true; }
   /// set a pointer to the predecessor of the node
   void setPred(Node2D *pred_ptr) { this->pred_ptr_ = pred_ptr; }

   void setSmartPtrPred(std::shared_ptr<Node2D> _pred) { pred_shared_ptr_ = _pred; }
   std::shared_ptr<Node2D> getSmartPtrPred() { return pred_shared_ptr_; }

   // UPDATE METHODS
   /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
   float updateCostSofar(Node2D const *parent);
   /// Updates the cost-to-go for the node x' to the goal node.
   void updateHeuristic(Node2D const *goal) { h = movementCost(*goal); }
   /// The heuristic as well as the cost measure.
   float movementCost(const Node2D &pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }

   // CUSTOM OPERATORS
   /// Custom operator to compare nodes. Nodes are equal if their x and y position is the same.
   bool operator==(const Node2D &rhs) const
   {
     return x == rhs.x && y == rhs.y;
   };

 private:
   /// the x position
   float x;
   /// the y position
   float y;
   /// the cost-so-far
   float g;
   /// the cost-to-go
   float h;
   /// the index of the node in the 2D array
   int idx;
   /// the open value
   bool o;
   /// the closed value
   bool c;
   /// the discovered value
   bool d;
   /// the predecessor pointer
   Node2D *pred_ptr_;
   std::shared_ptr<Node2D> pred_shared_ptr_ = nullptr;
   // cost from cost_map
   unsigned int map_cost_ = 0;
};
}
