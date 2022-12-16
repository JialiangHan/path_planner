#pragma once

#include <cmath>
#include <unordered_map>
#include <costmap_2d/costmap_2d.h>
#include <vector>
namespace HybridAStar {

/*!
   \brief A two dimensional node class used for the holonomic with obstacles heuristic.

   Each node has a unique discrete position (x,y).
*/
class Node2D {
 public:
  /// The default constructor for 2D array initialization.
   Node2D() : Node2D(0, 0, 999, 0, nullptr) {}
   /// Constructor for a node with the given arguments
   Node2D(int _x, int _y, float _g = 999, float _h = 0, Node2D *_pred_ptr = nullptr, std::shared_ptr<Node2D> _pred_shared_ptr = nullptr) : x(_x), y(_y), g(_g), h(_h), idx(-1), o(false), c(false), d(false), pred_ptr_(_pred_ptr), pred_shared_ptr_(_pred_shared_ptr)
   {
     this->map_cost_ = -1;
   }
   // GETTER METHODS
   /// get the x position
   int getX() const { return x; }
   /// get the y position
   int getY() const { return y; }
   /// get the cost-so-far (real value)
   float getCostSofar() const { return g; }
   /// get the cost-to-come (heuristic value)
   float getCostToGo() const { return h; }
   /// get the total estimated cost
   float getTotalCost() const { return g + h; }
   /* this cost is the cost from costmap
    */
   float getMapCost() { return map_cost_; }
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
   void setX(const int &x) { this->x = x; }
   /// set the y position
   void setY(const int &y) { this->y = y; }
   /// set the cost-so-far (real value)
   void setCostSofar(const float &g) { this->g = g; }
   /// set the cost-to-come (heuristic value)
   void setCostToGo(const float &h) { this->h = h; }
   /* this cost is the cost from costmap
    */
   void setMapCost(unsigned int C) { map_cost_ = C; }
   /// set and get the index of the node in the 2D array
   int setIdx(int width)
   {
     this->idx = y * width + x;
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
   Node2D *pred_ptr_;
   std::shared_ptr<Node2D> pred_shared_ptr_ = nullptr;
   // cost from cost_map
   unsigned int map_cost_;
};

class GridSearch
{
 public:
   GridSearch()
   {
   }
   ~GridSearch()
   {
   }
   // used in GenerateDpmap
   std::vector<std::shared_ptr<Node2D>> getAdjacentPoints(int cells_x, int cells_y, const unsigned char *charMap, std::shared_ptr<Node2D> point);
   /**
    * @brief
    *
    * @param goal_x
    * @param goal_y
    * @param costmap
    * @return std::unordered_map<int, std::shared_ptr<Node2D>>
    */
   std::unordered_map<int, std::shared_ptr<Node2D>> GenerateDpMap(
       const double goal_x, const double goal_y,
       costmap_2d::Costmap2D *costmap);
   // std::unordered_map<int, std::shared_ptr<Node2D>> dp_map_;
 private:
   // std::unordered_map<int, std::shared_ptr<Node2D>> dp_map_;
   struct cmp
   {
     // Sorting 3D nodes by increasing C value - the total estimated cost
     bool operator()(const std::pair<int, double> &left,
                     const std::pair<int, double> &right) const
     {
       return left.second >= right.second;
     }
   };
};
}
