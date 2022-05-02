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
#include "computational_geometry.h"
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

    // TODO need a function to convert occupancygrid to a configuration space map
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
      obstacle_detection_range_ = 6 * sqrt(params_.vehicle_width * 0.5 * params_.vehicle_width * 0.5 + params_.vehicle_length * 0.5 * params_.vehicle_length * 0.5);
      // DLOG(INFO) << "obstacle_detection_range is " << obstacle_detection_range;
      SetInRangeObstacle(obstacle_detection_range_);
      SetDistanceAngleRangeMap();
      BuildObstacleDensityMap(1.3 * obstacle_detection_range_);
      BuildNormalizedObstacleDensityMap();
      // BuildCollisionLookupTable();
    }
    nav_msgs::OccupancyGrid::Ptr GetMap() const { return grid_ptr_; };
    /**
     * @brief find free angle range and obstacle angle range from a vector of <distance,angle> pair
     *
     * @param node3d
     * @return std::vector<std::pair<float, Utility::AngleRange>> pair first is min distance, pair second is angle range
     */
    std::vector<std::pair<float, Utility::AngleRange>> FindFreeAngleRangeAndObstacleAngleRange(const Node3D &node3d);
    /**
     * @brief select step size and steering angle according to available angle range vec and current node
     *
     * @param available_angle_range_vec
     * @param pred
     * @return std::vector<std::pair<float, float>> first is step size, second is steering angle
     */
    std::vector<std::pair<float, float>> SelectStepSizeAndSteeringAngle(const std::vector<std::pair<float, Utility::AngleRange>> &available_angle_range_vec, const Node3D &pred,const int& number_of_successor);

    float GetObstacleDetectionRange() const { return obstacle_detection_range_; };

    // float GetObstacleDensity(const Node3D &node3d);

    float GetNormalizedObstacleDensity(const Node3D &node3d);

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
    /**
     * @brief Set the In Range Obstacle object and build obstacle density map
     *
     * @param range
     */
    void SetInRangeObstacle(const float &range);
    /**
     * @brief Get the Node3D Index On Grid Map, index=y*map width+x;
     *
     * @param node3d
     * @return uint index, this index is a 2d index
     */
    uint GetNode3DIndexOnGridMap(const Node3D &node3d);
    /**
     * @brief Get the Node 3 D Fine Index on grid map, index is calculated by CalculateFineIndex
     *
     * @param node3d
     * @return uint
     */
    uint GetNode3DFineIndex(const Node3D &node3d);
    /**
     * @brief Get the Node3D Index On Grid Map, index=y*map width+x;
     *
     * @param node3d
     * @return uint index
     */
    uint GetNode3DIndexOnGridMap(const float &x, const float &y);
    /**
     * @brief get node3d 3d index on grid map, index=theta*(angle resolution)*map width+y*map width+x;
     *
     * @param x
     * @param y
     * @param theta
     * @return uint
     */
    uint Get3DIndexOnGridMap(const float &x, const float &y, const float &theta);
    /**
     * @brief get node3d 3d index
     *
     * @param node3d
     * @return uint
     */
    uint Get3DIndexOnGridMap(const Node3D &node3d);

    /**
     * @brief Get the Node 3D Available steering Angle Range, current orientation +-30deg
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
    bool CollisionCheck(const Utility::Polygon &polygon);
    /**
     * @brief check if this segment is intersect with obstacle
     *
     * @param start
     * @param end
     * @return true
     * @return false
     */
    bool CollisionCheck(const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    /**
     * @brief check if this segment is intersect with obstacle
     *
     * @param segment
     * @return true
     * @return false
     */
    bool CollisionCheck(const ComputationalGeometry::Segment &segment);
    /**
     * @brief calculate fine index according its coordinate within one cell.
     *
     * @param x
     * @param y
     * @param t in rad
     * @return uint
     */
    uint
    CalculateFineIndex(const float &x, const float &y, const float &t);

    void CombineInNeighborObstacles();

    std::vector<std::pair<float, Utility::AngleRange>> GetObstacleInAvailableSteeringAngleRangle(const Node3D &node3d);
    /**
     * @brief sweep from steering angle range at node3d, get angle and their distance which has no collision
     *
     * @param node3d
     * @param radius
     * @return std::vector<std::pair<float, float>>, first is angle, second is distance.
     */
    std::vector<std::pair<float, float>> SweepDistanceAndAngle(const Node3D &node3d, const float &radius);
    /**
     * @brief   find max distance which has no collision on current map in certain range, range is equal to in range obstacle map
     *
     * @param node3d
     * @param radius in this range find max distance
     * @param angle in rad, start at 3 o`clock, CCW is positive
     * @return float
     */
    float FindNoCollisionDistance(const Node3D &node3d, const float &radius, const float &angle);
    /**
     * @brief find free angle range(no obstacle in the range) in node3d steering angle range.
     *
     * @param node3d
     * @return Utility::AngleRangeVec,angle range is pair of start angle and angle range
     */
    Utility::AngleRangeVec FindFreeAngleRange(const Node3D &node3d);
    /**
     * @brief build in range obstacle density map according to map info
     *
     * @param range
     */
    void BuildObstacleDensityMap(const float &range);
    /**
     * @brief this function is building a normalized obstacle density map from in_range_obstacle_density_map_
     *
     */
    void BuildNormalizedObstacleDensityMap();
    /**
     * @brief Get the weight_step_size from normalized obstacle density
     *
     * @param normalizied_obstacle_density this is from [0,1]
     * @return float output should be somehow in a (0,1) range
     */
    float GetStepSizeWeight(const float &normalizied_obstacle_density);

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

    float obstacle_detection_range_;
    /**
     * @brief key is location index, value is in range obstacle density defined by (number of obstacle in range ), definition need further consideration., this value define the step size for  adaptive step size
     *
     */
    std::unordered_map<uint, float> in_range_obstacle_density_map_;
    /**
     * @brief this is a normalized density map, which is calculated from in range obstacle density map, the range of second is from 0-1, 0 is not a good idea, need to convert to some number > 0
     *
     */
    std::unordered_map<uint, float> normalized_obstacle_density_map_;
  };
}
