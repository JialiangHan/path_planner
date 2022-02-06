#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"
#include "parameter_manager.h"
#include "utility.h"
#include <unordered_map>
#include <algorithm>
#include <cmath>
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

    bool IsTraversable(const std::shared_ptr<Node2D> &nod2d_ptr);
    bool IsTraversable(const std::shared_ptr<Node3D> &nod3d_ptr);
    bool IsTraversable(const Node3D &nod3d_ptr);

    /*!
     \brief updates the grid with the world map
  */
    void UpdateGrid(const nav_msgs::OccupancyGrid::Ptr &map)
    {
      grid_ptr_ = map;
      map_height_ = map->info.height;
      map_width_ = map->info.width;
      SetObstacleVec();
      CombineInNeighborObstacles();
      float obstacle_detection_range = 6 * sqrt(params_.vehicle_width * 0.5 * params_.vehicle_width * 0.5 + params_.vehicle_length * 0.5 * params_.vehicle_length * 0.5);
      DLOG(INFO) << "obstacle_detection_range is " << obstacle_detection_range;
      SetInRangeObstacle(obstacle_detection_range);
      SetDistanceAngleRangeMap();
      // BuildCollisionLookupTable();
    }
    /**
     * @brief find a list of angle range which has no obstacle in a certain radius, note for forward and backward, steering angle is the same
     * 
     * @param node3d 
     * @return Utility::AngleRangeVec ,pair: first is min angle, second is max angle ,all in rads
     */
    Utility::AngleRangeVec FindFreeAngleRange(const Node3D &node3d);
    /**
     * @brief find a list of angle range which has no obstacle in a certain radius, note for forward and backward, steering angle is the same, and its step size
     * 
     * @param node3d 
     * @return std::vector<std::pair<float,AngleRange>> first is step size 
     */
    std::vector<std::pair<float, Utility::AngleRange>> FindFreeAngleRangeAndStepSize(const Node3D &node3d);
    nav_msgs::OccupancyGrid::Ptr GetMap() const { return grid_ptr_; };

    std::vector<std::pair<float, Utility::AngleRange>> GetDistanceAngleRangeVec(const HybridAStar::Node3D &current_node);

  private:
    bool IsOnGrid(const Node3D &node3d) const;
    bool IsOnGrid(const Node2D &node2d) const;
    bool IsOnGrid(const std::shared_ptr<Node2D> &node2d_ptr) const;
    bool IsOnGrid(const std::shared_ptr<Node3D> &node3d_ptr) const;
    bool IsOnGrid(const float &x, const float &y) const;
    bool IsOnGrid(const Eigen::Vector2f &point) const;

    /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
    bool configurationTest(const float &x, const float &y, const float &t);
    /**
     * @brief Tests whether the segment start to end of the robot is in C_free
     * 
     * @param start 
     * @param end 
     * @return true free
     * @return false collision
     */
    bool configurationTest(const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    bool configurationTest(const Node3D &start, const Node3D &end);

    void getConfiguration(const std::shared_ptr<Node2D> &node2d_ptr, float &x, float &y, float &t) const;

    void getConfiguration(const std::shared_ptr<Node3D> &node3d_ptr, float &x, float &y, float &t) const;

    void getConfiguration(const Node3D &node, float &x, float &y, float &t) const;

    void SetObstacleVec();

    void SetInRangeObstacle(const float &range);

    uint GetNode3DIndexOnGridMap(const Node3D &node3d);

    uint GetNode3DIndexOnGridMap(const float &x, const float &y);
    /**
     * @brief this function is only for collision detection
     * 
     * @param x 
     * @param y 
     * @param theta 
     * @return uint 
     */
    // uint CalculateNode3DIndex(const float &x, const float &y, const float &theta) const;

    /**
     * @brief Get the Node 3 D Available Angle Range object
     * 
     * @param node3d 
     * @return Utility::AngleRange 
     */
    Utility::AngleRange GetNode3DAvailableAngleRange(const Node3D &node3d);

    void SetDistanceAngleRangeMap();

    void BuildCollisionLookupTable();
    /**
     * @brief check if this polygon is intersect with obstacle
     * 
     * @param polygon 
     * @return true free
     * @return false collsion 
     */
    bool CollsionCheck(const Utility::Polygon &polygon);
    /**
     * @brief check if this segment is intersect with obstacle
     * 
     * @param start 
     * @param end 
     * @return true 
     * @return false 
     */
    bool CollsionCheck(const Eigen::Vector2f &start, const Eigen::Vector2f &end);

    uint CalculateFineIndex(const float &x, const float &y, const float &t);

    void CombineInNeighborObstacles();

  private:
    ParameterCollisionDetection params_;
    /// The occupancy grid
    nav_msgs::OccupancyGrid::Ptr grid_ptr_;
    uint map_width_;
    uint map_height_;
    /// The collision lookup table
    Constants::config collisionLookup[Constants::headings * Constants::positions];
    //collision lookup table, use array instead of vector, temporally use vector
    /**
     * @brief key is location index(y*width+x) in the map, value is unordered map for fine index, key is fine index related to params.position resolution, value is collision or not, true for free, false for collision.
     * 
     */
    std::unordered_map<uint, std::unordered_map<uint, bool>> collision_lookup_;

    /**
     * @brief all obstacle, in a vector format
     * 
     */
    std::vector<Utility::Polygon> obstacle_vec_;
    /**
     * @brief key is location index, value is vector of polygon in a certain range
     * 
     */
    std::unordered_map<uint, std::vector<Utility::Polygon>> in_range_obstacle_map_;
    /**
     * @brief key int: is the location index ,value is a vector of pair<distance to obstacle, and its angle range
     * 
     */
    std::unordered_map<uint, std::vector<std::pair<float, Utility::AngleRange>>> distance_angle_range_map_;
  };
}
