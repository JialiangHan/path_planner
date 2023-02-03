#pragma once

#include <cmath>
#include <memory>
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "constants.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y, theta) in the configuration space C.
*/
class Node3D {
 public:
   /// The default constructor for 3D array initialization
   Node3D() : Node3D(0, 0, 0, 999, 0, false, nullptr) {}
   /// Constructor for a node with the given arguments
   Node3D(float _x, float _y, float _t, float _g = 999, float _h = 0, bool _reverse = false, const std::shared_ptr<Node3D> &_pred_smart_ptr = nullptr, int prim = 0) : x(_x), y(_y), t(_t), g(_g), h(_h), map_cost_(0), idx(-1), o(false), c(false), reverse(_reverse), pred_smart_ptr_(_pred_smart_ptr)
   {
     this->prim = prim;
   }

   // GETTER METHODS
   /// get the x position
   float getX() const { return x; }
   /// get the y position
   float getY() const { return y; }
   /// get the heading theta
   float getT() const { return t; }
   /// get the cost-so-far (real value)
   float getCostSofar() const { return g; }
   /// get the cost-to-come (heuristic value)
   float getCostToGo() const { return h; }
   /// get the total estimated cost
   float getTotalCost() const { return g + h; }

   /// get the index of the node in the 3D array
   int getIdx() const { return idx; }
   /// get the number associated with the motion primitive of the node
   int getPrim() const { return prim; }
   /// determine whether the node is open
   bool isOpenSet() const { return o; }
   /// determine whether the node is closed
   bool isClosedSet() const { return c; }
   bool isReverse() { return reverse; }
   /// determine whether the node is open
   std::shared_ptr<Node3D> getSmartPtrPred() const { return pred_smart_ptr_; }
   // SetTER METHODS
   /// set the x position
   void setX(const float &x) { this->x = x; }
   /// set the y position
   void setY(const float &y) { this->y = y; }
   /// set the heading theta
   void setT(const float &t) { this->t = t; };
   /// set the cost-so-far (real value)
   void setCostSofar(const float &g)
   {
     this->g = g;
   }
   /// set the cost-to-come (heuristic value)
   void setCostToGo(const float &h) { this->h = h; }

   /// set and get the index of the node in the 3D grid
   int setIdx(int width, int height, const float &delta_heading_in_rad, const float &resolution, float origin_x, float origin_y)
   {
     this->idx = std::floor(std::round(t / delta_heading_in_rad * 1000) / 1000) * width * height + std::floor(std::round(((y - origin_y) / resolution) * 1000) / 1000) * width + std::floor(std::round(((x - origin_x) / resolution) * 1000) / 1000);
     //  LOG_IF(FATAL, idx >= width * height) << "idx larger than width*height!!! idx is " << idx << " width is " << width << " height is " << height << " resolution is " << resolution << " origin x is " << origin_x << " origin y is " << origin_y;
     LOG_IF(FATAL, idx < 0) << "idx smaller than zero!!! idx is " << idx << " width is " << width << " height is " << height << " delta_heading_in_rad is " << delta_heading_in_rad << " resolution is " << resolution << " origin x is " << origin_x << " origin y is " << origin_y << " x is " << x << " y is " << y << " t is " << t;
     LOG_IF(INFO, t >= 2 * M_PI) << "idx is " << idx << " width is " << width << " height is " << height << " delta_heading_in_rad is " << delta_heading_in_rad << " resolution is " << resolution << " origin x is " << origin_x << " origin y is " << origin_y << " x is " << x << " y is " << y << " t is " << t;
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
   void setSmartPtrPred(const std::shared_ptr<Node3D> &pred_ptr) { this->pred_smart_ptr_ = pred_ptr; }

   // CUSTOM OPERATORS
   /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
   bool operator==(const Node3D &rhs) const
   {
     if ((std::abs(x - rhs.x) < 1e-2) &&
         (std::abs(y - rhs.y) < 1e-2) &&
         (std::abs(t - rhs.t) < 1e-2))
     {
       return true;
     }
     return false;
   };

 private:
  /// the x position
  float x;
  /// the y position
  float y;
  /// the heading theta
  float t;
  /// the cost-so-far
  float g;
  /// the cost-to-go
  float h;
  /// the cost of this node in costmap
  unsigned int map_cost_;
  /// the index of the node in the 3D array
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the flag of node,Indicates whether the vehicle is in reverse
  bool reverse;
  /// the motion primitive of the node
  int prim;
  /// the predecessor pointer
  std::shared_ptr<Node3D> pred_smart_ptr_;
};
}
