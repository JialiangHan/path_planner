#ifndef NODE2D_H
#define NODE2D_H

#include <cmath>

#include <vector>
namespace HybridAStar {

/*!
   \brief A two dimensional node class used for the holonomic with obstacles heuristic.

   Each node has a unique discrete position (x,y).
*/
class Node2D {
 public:
  /// The default constructor for 2D array initialization.
  Node2D(): Node2D(0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node2D(int x, int y, float g, float h, const std::shared_ptr<Node2D> &pred_ptr)
  {
    this->x = x;
    this->y = y;
    this->g = g;
    this->h = h;
    this->pred_ptr_ = pred_ptr;
    this->o = false;
    this->c = false;
    this->d = false;
    this->idx = -1;
  }
  // GETTER METHODS
  /// get the x position
  int GetX() const { return x; }
  /// get the y position
  int GetY() const { return y; }
  /// get the cost-so-far (real value)
  float GetG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float GetH() const { return h; }
  /// get the total estimated cost
  float GetC() const { return g + h; }
  /// get the index of the node in the 2D array
  int GetIdx() const { return idx; }
  /// determine whether the node is open
  bool  isOpen() const { return o; }
  /// determine whether the node is closed
  bool  isClosed() const { return c; }
  /// determine whether the node is discovered
  bool  isDiscovered() const { return d; }
  /// get a pointer to the predecessor
  std::shared_ptr<Node2D> GetPred() const { return pred_ptr_; }

  // SETTER METHODS
  /// set the x position
  void setX(const int& x) { this->x = x; }
  /// set the y position
  void setY(const int& y) { this->y = y; }
  /// set the cost-so-far (real value)
  void SetG(const float &g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void SetH(const float &h) { this->h = h; }
  /// set and get the index of the node in the 2D array
  int setIdx(int width) { this->idx = y * width + x; return idx;}
  /// open the node
  void open() { o = true; c = false; }
  /// close the node
  void close() { c = true; o = false; }
  /// set the node neither open nor closed
  void reset() { c = false; o = false; }
  /// discover the node
  void discover() { d = true; }
  /// set a pointer to the predecessor of the node
  void SetPred(const std::shared_ptr<Node2D> &pred_ptr) { this->pred_ptr_ = pred_ptr; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG()
  {
    g += movementCost(*pred_ptr_);
    d = true;
  }
  /// Updates the cost-to-go for the node x' to the goal node.
  void UpdateHeuristic(const Node2D &goal) { h = movementCost(goal); }
  /// The heuristic as well as the cost measure.
  float movementCost(const Node2D& pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position is the same.
  bool operator==(const Node2D &rhs) const
  {
    return x == rhs.x && y == rhs.y;
  };

private:
  /// the x position
  int x;
  /// the y position
  int y;
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
  std::shared_ptr<Node2D> pred_ptr_;
};
}
#endif // NODE2D_H
