#pragma once

#include <cmath>
#include <memory>
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y, theta) in the configuration space C.
*/
class Node3D {
 public:
   /// The default constructor for 3D array initialization
   Node3D() : Node3D(0, 0, 0, 0, 0, nullptr) {}
   /// Constructor for a node with the given arguments
   Node3D(float x, float y, float t, float g, float h, const std::shared_ptr<Node3D> &pred_ptr, int prim = 0)
   {
     this->x = x;
     this->y = y;
     this->t = t;
     this->g = g;
     this->h = h;
     this->pred_ptr_ = pred_ptr;
     this->o = false;
     this->c = false;
     this->idx = -1;
     this->prim = prim;
   }

   // GETTER METHODS
   /// get the x position
   float GetX() const { return x; }
   /// get the y position
   float GetY() const { return y; }
   /// get the heading theta
   float GetT() const { return t; }
   /// get the cost-so-far (real value)
   float GetCostSofar() const { return g; }
   /// get the cost-to-come (heuristic value)
   float GetCostToGo() const { return h; }
   /// get the total estimated cost
   float GetTotalCost() const { return g + h; }
   /// get the index of the node in the 3D array
   int GetIdx() const { return idx; }
   /// get the number associated with the motion primitive of the node
   int GetPrim() const { return prim; }
   /// determine whether the node is open
   bool isOpen() const { return o; }
   /// determine whether the node is closed
   bool isClosed() const { return c; }
   /// determine whether the node is open
   std::shared_ptr<Node3D> GetPred() const { return pred_ptr_; }

   // SETTER METHODS
   /// set the x position
   void setX(const float &x) { this->x = x; }
   /// set the y position
   void setY(const float &y) { this->y = y; }
   /// set the heading theta
   void setT(const float &t) { this->t = t; }
   /// set the cost-so-far (real value)
   void SetG(const float &g) { this->g = g; }
   /// set the cost-to-come (heuristic value)
   void SetH(const float &h) { this->h = h; }
   /// set and get the index of the node in the 3D grid
   int setIdx(int width, int height, const float &delta_heading_in_rad)
   {
     this->idx = (int)(t / delta_heading_in_rad) * width * height + (int)(y)*width + (int)(x);
     return idx;
   }
   /// open the node
   void open()
   {
     o = true;
     c = false;
   }
   /// close the node
   void close()
   {
     c = true;
     o = false;
   }
   /// set a pointer to the predecessor of the node

   void SetPred(const std::shared_ptr<Node3D> &pred_ptr) { this->pred_ptr_ = pred_ptr; }

   // CUSTOM OPERATORS
   /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
   bool operator==(const Node3D &rhs) const
   {
     return (int)x == (int)rhs.x &&
            (int)y == (int)rhs.y &&
            (t == rhs.t);
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
  /// the index of the node in the 3D array
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the motion primitive of the node
  int prim;
  /// the predecessor pointer
  std::shared_ptr<Node3D> pred_ptr_;
};
}
