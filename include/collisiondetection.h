#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>
#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"
#include "parameter_manager.h"
namespace HybridAStar
{

  /*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
  class CollisionDetection
  {
  public:
    /// Constructor
    CollisionDetection(){};

    CollisionDetection(const ParameterCollisionDetection &params)
    {
      params_ = params;
      this->grid_ptr_ = nullptr;
      Lookup::collisionLookup(collisionLookup);
    };

    bool IsTraversable(const std::shared_ptr<Node2D> &nod2d_ptr) const;
    /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
    template <typename T>
    bool IsTraversable(const T *node) const
    {
      if (!IsOnGrid(node))
      {
        return false;
      }
      /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
      float cost = 0;
      float x;
      float y;
      float t;
      // assign values to the configuration
      getConfiguration(node, x, y, t);

      // 2D collision test
      if (t == 99)
      {
        return !grid_ptr_->data[node->GetIdx()];
      }

      if (true)
      {
        cost = configurationTest(x, y, t) ? 0 : 1;
      }
      else
      {
        cost = configurationCost(x, y, t);
      }

      return cost <= 0;
    };

    /*!
     \brief updates the grid with the world map
  */
    void UpdateGrid(nav_msgs::OccupancyGrid::Ptr map) { grid_ptr_ = map; }

    bool IsOnGrid(const Node3D &node3d) const;

    bool IsOnGrid(const Node3D *node3d_ptr) const;

    bool IsOnGrid(const Node2D &node2d) const;

    bool IsOnGrid(const Node2D *node2d_ptr) const;
    bool IsOnGrid(const std::shared_ptr<Node2D> node2d_ptr) const;
    /// The occupancy grid
    nav_msgs::OccupancyGrid::Ptr grid_ptr_;

  private:
    /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
    float configurationCost(float x, float y, float t) const { return 0; }
    /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
    bool configurationTest(float x, float y, float t) const;
    void getConfiguration(const Node2D *node, float &x, float &y, float &t) const;

    void getConfiguration(const std::shared_ptr<Node2D> &node2d_ptr, float &x, float &y, float &t) const;

    void getConfiguration(const Node3D *node, float &x, float &y, float &t) const;

  private:
    /// The collision lookup table
    Constants::config collisionLookup[Constants::headings * Constants::positions];
    ParameterCollisionDetection params_;
  };
}
#endif // COLLISIONDETECTION_H
