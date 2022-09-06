#include "collisiondetection.h"

using namespace HybridAStar;
//**********************constructor*******************
CollisionDetection::CollisionDetection(const ParameterCollisionDetection &params)
{
  params_ = params;
  this->grid_ptr_ = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

void CollisionDetection::UpdateGrid(const nav_msgs::OccupancyGrid::Ptr &map)
{
  grid_ptr_ = map;

  SetObstacleVec();
  // CombineInNeighborObstacles();
  obstacle_detection_range_ = 6 * sqrt(params_.vehicle_width * 0.5 * params_.vehicle_width * 0.5 + params_.vehicle_length * 0.5 * params_.vehicle_length * 0.5);
  // DLOG(INFO) << "obstacle_detection_range is " << obstacle_detection_range_;
  SetInRangeObstacle(obstacle_detection_range_);
  SetDistanceAngleRangeMap();
  BuildObstacleDensityMap(1.3 * obstacle_detection_range_);
  BuildNormalizedObstacleDensityMap();
  // BuildCollisionLookupTable();
}

bool CollisionDetection::IsTraversable(const std::shared_ptr<Node2D> &node2d_ptr)
{
  // DLOG(INFO) << "IsTraversable in:";
  if (!IsOnGrid(node2d_ptr))
  {
    // DLOG(INFO) << "IsTraversable out.";
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
  getConfiguration(node2d_ptr, x, y, t);
  // 2D collision test
  if (t == 99)
  {
    // DLOG(INFO) << "IsTraversable out.";
    return !grid_ptr_->data[node2d_ptr->GetIdx()];
  }
  if (true)
  {
    cost = configurationTest(x, y, t) ? 0 : 1;
  }
  // DLOG(INFO) << "IsTraversable out.";
  return cost <= 0;
};
bool CollisionDetection::IsTraversable(const std::shared_ptr<Node3D> &node3d_ptr)
{
  return IsTraversable(*node3d_ptr);
};

bool CollisionDetection::IsTraversable(const Node3D &node3d)
{
  // DLOG(INFO) << "IsTraversable in: current node is x " << node3d.GetX() << " y " << node3d.GetY() << " angle is " << Utility::ConvertRadToDeg(node3d.GetT());
  if (!IsOnGrid(node3d))
  {
    // DLOG(INFO) << "IsTraversable out.";
    return false;
  }
  /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */

  float x, y, t;
  // assign values to the configuration
  getConfiguration(node3d, x, y, t);
  // 2D collision test
  if (t == 99)
  {
    // DLOG(INFO) << "IsTraversable out.";
    return !grid_ptr_->data[node3d.GetIdx()];
  }
  if (node3d.GetPred() != nullptr)
  {
    // if (configurationTest(x, y, t))
    if (configurationTest(x, y, t) && configurationTest(node3d, *node3d.GetPred()))
    {
      // DLOG(INFO) << "IsTraversable out.";
      return true;
    }
  }
  else
  {
    if (configurationTest(x, y, t))
    {
      // DLOG(INFO) << "IsTraversable out.";
      return true;
    }
  }
  // DLOG(INFO) << "IsTraversable out.";
  return false;
}

bool CollisionDetection::IsTraversable(const Node3D &current_node, const Node3D &previous_node)
{ // DLOG(INFO) << "IsTraversable in: current node is x " << node3d.GetX() << " y " << node3d.GetY() << " angle is " << Utility::ConvertRadToDeg(node3d.GetT());
  if (!IsOnGrid(current_node) || !IsOnGrid(previous_node))
  {
    // DLOG(INFO) << "IsTraversable out.";
    return false;
  }

  // if (configurationTest(x, y, t))
  if (configurationTest(current_node.GetX(), current_node.GetY(), current_node.GetT()) && configurationTest(current_node, previous_node) && configurationTest(previous_node.GetX(), previous_node.GetY(), previous_node.GetT()))
  {
    // DLOG(INFO) << "IsTraversable out.";
    return true;
  }
  // DLOG(INFO) << "IsTraversable out.";
  return false;
}
bool CollisionDetection::IsOnGrid(const Eigen::Vector2f &point) const
{
  return IsOnGrid(point.x(), point.y());
}
bool CollisionDetection::IsOnGrid(const Node3D &node3d) const
{
  return IsOnGrid(node3d.GetX(), node3d.GetY());
}

bool CollisionDetection::IsOnGrid(const Node2D &node2d) const
{
  return IsOnGrid(node2d.GetX(), node2d.GetY());
}

bool CollisionDetection::IsOnGrid(const std::shared_ptr<Node2D> &node2d_ptr) const
{
  return IsOnGrid(node2d_ptr->GetX(), node2d_ptr->GetY());
}

bool CollisionDetection::IsOnGrid(const std::shared_ptr<Node3D> &node3d_ptr) const
{
  return IsOnGrid(node3d_ptr->GetX(), node3d_ptr->GetY());
}

bool CollisionDetection::IsOnGrid(const float &x, const float &y) const
{
  if (x >= 0 && x < (int)grid_ptr_->info.width &&
      y >= 0 && y < (int)grid_ptr_->info.height)
  {
    return true;
  }
  else
  {
    // DLOG(WARNING) << "WARNING: current node " << x << " " << y << " is not on grid!!!";
    return false;
  }
}
bool CollisionDetection::configurationTest(const float &x, const float &y, const float &t)
{
  if (params_.enable_collision_lookup)
  {
    // 1. find index in the collision lookup map
    uint index = GetNode3DIndexOnGridMap(x, y);
    // 2. check collision lookup table
    if (collision_lookup_.find(index) != collision_lookup_.end())
    {
      uint fine_index = CalculateFineIndex(x, y, t);
      // DLOG(INFO) << "index is " << index << " fine index is " << fine_index;
      Utility::Polygon polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y), params_.vehicle_length, params_.vehicle_width, t);
      bool collision_result;
      collision_result = CollisionCheck(polygon);
      return collision_result;
      if (collision_lookup_[index].find(fine_index) != collision_lookup_[index].end())
      {
        // DLOG(INFO) << "collision result is " << collision_lookup_[index][fine_index];
        return collision_lookup_[index][fine_index];
      }
      // DLOG(WARNING) << "fine index can`t be found in the collision lookup map, coordinate is " << x << " " << y << " " << Utility::ConvertRadToDeg(t);
    }
    else
    {
      DLOG(WARNING) << "WARNING: index can`t be found in the collision lookup map, coordinate is " << x << " " << y << " " << Utility::ConvertRadToDeg(t);
    }
  }
  else
  {
    Utility::Polygon polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y), params_.vehicle_length, params_.vehicle_width, t);
    bool collision_result;
    collision_result = CollisionCheck(polygon);
    if (!collision_result)
    {
      // DLOG(INFO) << "in collision, coordinate is " << x << " " << y << " " << t;
    }
    return collision_result;
  }
  return false;
}
bool CollisionDetection::configurationTest(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
{
  bool collision_result;
  collision_result = CollisionCheck(start, end);
  if (!collision_result)
  {
    // DLOG(INFO) << "in collision, segment start at " << start.x() << " " << start.y() << " end at " << end.x() << " " << end.y();
  }
  return collision_result;
}
bool CollisionDetection::configurationTest(const Node3D &start, const Node3D &end)
{

  return configurationTest(Utility::ConvertNod3DToVector2f(start), Utility::ConvertNod3DToVector2f(end));
}

void CollisionDetection::getConfiguration(const Node3D &node, float &x, float &y, float &t) const
{
  x = node.GetX();
  y = node.GetY();
  t = node.GetT();
}
void CollisionDetection::getConfiguration(const std::shared_ptr<Node2D> &node2d_ptr, float &x, float &y, float &t) const
{
  x = node2d_ptr->GetX();
  y = node2d_ptr->GetY();
  // avoid 2D collision checking
  t = 99;
}
void CollisionDetection::getConfiguration(const std::shared_ptr<Node3D> &node3d_ptr, float &x, float &y, float &t) const
{
  x = node3d_ptr->GetX();
  y = node3d_ptr->GetY();
  t = node3d_ptr->GetT();
}

// checked
void CollisionDetection::SetObstacleVec()
{
  // DLOG(INFO) << "SetObstacleVec in:";
  Utility::Polygon current_polygon;
  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      if (grid_ptr_->data[GetNode3DIndexOnGridMap(x, y)])
      {
        current_polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y));
        if (obstacle_vec_.size() != 0)
        {
          if (Utility::IsPolygonInNeighbor(obstacle_vec_.back(), current_polygon))
          {
            Utility::Polygon combined_polygon = Utility::CombinePolygon(current_polygon, obstacle_vec_.back());
            obstacle_vec_.pop_back();
            obstacle_vec_.emplace_back(combined_polygon);
          }
          else
          {
            obstacle_vec_.emplace_back(current_polygon);
          }
        }
        else
        {
          obstacle_vec_.emplace_back(current_polygon);
        }

        // DLOG(INFO) << "obstacle origin is " << x << " " << y;
      }
    }
  }
  if (params_.map_boundary_obstacle)
  {
    AddMapBoundaryAsObstacle(obstacle_vec_);
  }
  // for (const auto &polygon : obstacle_vec_)
  // {
  //   DLOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y();
  // }
  // DLOG(INFO) << "SetObstacleVec out.";
}
// checked
void CollisionDetection::SetInRangeObstacle(const float &range)
{
  std::vector<Utility::Polygon> obstacle_vec;

  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      uint current_point_index = GetNode3DIndexOnGridMap(x, y);
      Eigen::Vector2f current_point(x, y);
      obstacle_vec.clear();
      for (const auto &polygon : obstacle_vec_)
      {
        float distance = Utility::GetDistanceFromPolygonToPoint(polygon, current_point);

        if (distance < range)
        {
          obstacle_vec.emplace_back(polygon);
          // DLOG_IF(INFO, distance < 0) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << "  and its in the range. current point is " << x << " " << y << " distance is " << distance;
        }
      }
      in_range_obstacle_map_.emplace(current_point_index, obstacle_vec);
    }
  }
}
// checked,it`s correct.
uint CollisionDetection::GetNode3DIndexOnGridMap(const Node3D &node3d)
{

  return GetNode3DIndexOnGridMap(node3d.GetX(), node3d.GetY());
}
// checked,it`s correct.
uint CollisionDetection::GetNode3DIndexOnGridMap(const float &x, const float &y)
{
  // DLOG(INFO) << "coordinate is " << x << " " << y << " "
  //  << " final index is " << (uint)y * grid_ptr_->info.width + (uint)x;
  return (uint)y * grid_ptr_->info.width + (uint)x;
}
// checked, it`s correct.
Utility::AngleRange CollisionDetection::GetNode3DAvailableSteeringAngleRange(const Node3D &node3d)
{
  // DLOG(INFO) << "GetNode3DAvailableSteeringAngleRange in:";
  Utility::AngleRange out;
  float current_heading = Utility::RadNormalization(node3d.GetT());
  const float max_steering_angle = Utility::ConvertDegToRad(30);
  float starting_angle, range;

  starting_angle = Utility::RadToZeroTo2P(current_heading - max_steering_angle);
  range = (2 * max_steering_angle);
  // DLOG(INFO) << "starting angle is :" << Utility::ConvertRadToDeg(starting_angle) << " range is " << Utility::ConvertRadToDeg(range);
  out = std::make_pair(starting_angle, range);
  // DLOG(INFO) << "GetNode3DAvailableSteeringAngleRange out.";
  return out;
}
// checked, it`s correct
void CollisionDetection::SetDistanceAngleRangeMap()
{
  // DLOG(INFO) << "SetDistanceAngleRangeMap in:";
  std::vector<std::pair<float, Utility::AngleRange>> distance_angle_range_vec;
  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      // DLOG(INFO) << "current node is " << x << " " << y;
      distance_angle_range_vec.clear();
      uint current_point_index = GetNode3DIndexOnGridMap(x, y);
      Eigen::Vector2f current_point(x, y);
      std::vector<std::pair<float, float>> angle_distance_vec = SweepDistanceAndAngle(Utility::ConvertVector2fToNode3D(current_point), obstacle_detection_range_, false);
      float range_start = -1, range_range = -1, min_distance;
      // flag to indicate whether angle range start or not
      bool free_angle_range_flag = false, obstacle_angle_range_flag = false;
      std::vector<float> distance_vec;
      for (const auto &pair : angle_distance_vec)
      {
        // DLOG(INFO) << "angle is " << Utility::ConvertRadToDeg(pair.first) << " distance is " << pair.second;
        if (pair.second == obstacle_detection_range_)
        {
          if (obstacle_angle_range_flag)
          {
            obstacle_angle_range_flag = false;
            free_angle_range_flag = true;
            range_range = pair.first - range_start;
            // find min distance
            DLOG_IF(WARNING, (distance_vec.size() - Utility::ConvertRadToDeg(range_range)) > 0.1) << "WARNING: distance vec size is not correct!!! current size is " << distance_vec.size() << " range range is " << Utility::ConvertRadToDeg(range_range);
            min_distance = *std::min_element(distance_vec.begin(), distance_vec.end());
            if (range_start != -1 && range_range != -1)
            {
              // push obstacle angle range into map
              distance_angle_range_vec.emplace_back(std::pair<float, Utility::AngleRange>(min_distance, Utility::AngleRange(range_start, range_range)));
              // DLOG(INFO) << "set obstacle angle range end to " << Utility::ConvertRadToDeg(range_start + range_range) << " set min distance to " << min_distance;
              range_start = pair.first;
              // DLOG(INFO) << "set free angle range start to " << Utility::ConvertRadToDeg(range_start);
            }
            else
            {
              range_start = pair.first;
              // DLOG(INFO) << "set free angle range start to " << Utility::ConvertRadToDeg(range_start);
            }
            distance_vec.clear();
          }
          if (!free_angle_range_flag)
          {
            free_angle_range_flag = true;
            obstacle_angle_range_flag = false;
            range_start = pair.first;
            // DLOG(INFO) << "set free angle range start to " << Utility::ConvertRadToDeg(range_start);
            distance_vec.clear();
          }
        }
        else
        {
          distance_vec.emplace_back(pair.second);
          if (free_angle_range_flag)
          {
            free_angle_range_flag = false;
            obstacle_angle_range_flag = true;
            range_range = pair.first - range_start;
            if (range_start != -1 && range_range != -1)
            {
              // push free angle range into map
              distance_angle_range_vec.emplace_back(std::pair<float, Utility::AngleRange>(obstacle_detection_range_, Utility::AngleRange(range_start, range_range)));
              // DLOG(INFO) << "set free angle range end to " << Utility::ConvertRadToDeg(range_start + range_range) << " set min distance to " << obstacle_detection_range_;
              range_start = pair.first;
              // DLOG(INFO) << "set obstacle angle range start to " << Utility::ConvertRadToDeg(range_start);
            }
            else
            {
              range_start = pair.first;
            }
          }
          if (!obstacle_angle_range_flag)
          {
            free_angle_range_flag = false;
            obstacle_angle_range_flag = true;
            range_start = pair.first;
            // DLOG(INFO) << "set obstacle angle range start to " << Utility::ConvertRadToDeg(range_start);
          }
        }
        // if this pair is last pair in angle distance vec, check this angle range is free or obstacle angle range,then push range to out
        if (pair.first == angle_distance_vec.back().first)
        {
          // check if size of distance vec is zero or not, if zero, just push back current distance
          if (distance_vec.size() == 0)
          {
            distance_vec.emplace_back(pair.second);
          }
          if (free_angle_range_flag)
          {
            min_distance = obstacle_detection_range_;
          }
          else
          {
            min_distance = *std::min_element(distance_vec.begin(), distance_vec.end());
          }
          range_range = pair.first - range_start;
          if (range_start != -1 && range_range != -1 && range_range != 0)
          {
            distance_angle_range_vec.emplace_back(std::pair<float, Utility::AngleRange>(min_distance, Utility::AngleRange(range_start, range_range)));
            // DLOG(INFO) << "set last angle range end to " << Utility::ConvertRadToDeg(range_start + range_range) << " set min distance to " << min_distance;
          }
        }
      }
      distance_angle_range_map_.emplace(current_point_index, distance_angle_range_vec);
      DLOG_IF(WARNING, distance_angle_range_vec.size() == 0) << "WARNING: size of distance angle range vec is zero!!!!";
    }
  }
  // DLOG(INFO) << "SetDistanceAngleRangeMap out.";
}

// where to put this collision table in lookup_table.cpp or collisiondetection..cpp?
void CollisionDetection::BuildCollisionLookupTable()
{
  // DLOG(INFO) << "BuildCollisionLookupTable start:";
  collision_lookup_.clear();
  // 0. three for loop to loop every point in the map, similar to lookup_table.cpp
  uint index, fine_index;
  int x_count, y_count;
  bool collision_result;
  std::unordered_map<uint, bool> collision_result_map;
  float x = 0, y = 0, theta = 0;

  const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
  while (x < grid_ptr_->info.width)
  {
    y = 0;
    // theta = 0;
    while (y < grid_ptr_->info.height)
    {
      // theta = 0;
      index = GetNode3DIndexOnGridMap(x, y);
      x_count = 0;
      while (x_count < params_.position_resolution)
      {
        // theta = 0;
        y_count = 0;
        while (y_count < params_.position_resolution)
        {
          theta = 0;
          while (theta < 2 * M_PI)
          {
            fine_index = CalculateFineIndex(x, y, theta);
            // DLOG(INFO) << "current index is " << index << " , fine index is " << fine_index << " coordinate is " << x << " " << y << " " << theta;
            // 1. create polygon accord to its coordinate(x,y), then rotate according to heading angle t.
            //  case: some certain case this polygon is outside map
            if ((x <= params_.vehicle_length / 2 && y <= params_.vehicle_width / 2) ||
                (x >= grid_ptr_->info.width - params_.vehicle_length / 2 && y <= params_.vehicle_width / 2) ||
                (x <= params_.vehicle_length / 2 && y >= grid_ptr_->info.height - params_.vehicle_width / 2) ||
                (x >= grid_ptr_->info.width - params_.vehicle_length / 2 && y >= grid_ptr_->info.height - params_.vehicle_width / 2))
            {
              DLOG(INFO) << "vehicle is outside map";
              collision_result = false;
            }
            else
            {
              Utility::Polygon polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y), params_.vehicle_length, params_.vehicle_width, theta);
              // for (const auto &point : polygon)
              // {
              //   DLOG(INFO) << "point is " << point.x() << " " << point.y();
              // }
              collision_result = CollisionCheck(polygon);
            }

            collision_result_map.emplace(fine_index, collision_result);
            // DLOG(INFO) << "collision result is " << collision_result;
            theta += delta_heading_in_rad;
          }
          y += 1.0f / params_.position_resolution;
          y_count++;
        }
        y--;
        x_count++;
        x += 1.0f / params_.position_resolution;
      }
      x--;

      collision_lookup_.emplace(index, collision_result_map);
      y++;
    }
    x++;
  }
  // DLOG(INFO) << "BuildCollisionLookupTable done.";
}

bool CollisionDetection::CollisionCheck(const Utility::Polygon &polygon)
{
  // DLOG(INFO) << "CollisionCheck in.";
  for (const auto &point : polygon)
  {
    if (!IsOnGrid(point))
    {
      // DLOG(INFO) << "polygon is not on grid: point is " << point.x() << " " << point.y();
      return false;
    }
  }
  // should be convert to int for x and y;
  uint current_point_index = GetNode3DIndexOnGridMap(polygon[0].x(), polygon[0].y());
  // check if this polygon inside map or outside map
  Utility::Polygon map_boundary = Utility::CreatePolygon(grid_ptr_->info.width, grid_ptr_->info.height);
  // DLOG(INFO) << "current polygon start is " << polygon[0].x() << " " << polygon[0].y() << " index is " << current_point_index;

  if (Utility::IsPolygonIntersectWithPolygon(polygon, map_boundary))
  {
    // DLOG(INFO) << "intersect with map boundary.";
    return false;
  }
  // check with in range obstacle
  if (in_range_obstacle_map_.find(current_point_index) != in_range_obstacle_map_.end())
  {
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_point_index])
    {
      if (Utility::IsPolygonIntersectWithPolygon(polygon, in_range_polygon))
      {
        // DLOG(INFO) << "intersect with obstacle.";
        return false;
      }
    }
    // DLOG(INFO) << " not intersect.";
  }
  else
  {
    DLOG(INFO) << "current polygon can`t be found in in_range_obstacle_map.";
  }
  return true;
  // 2. find obstacle in a certain range, this range is determined be vehicle parameter
}

bool CollisionDetection::CollisionCheck(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
{
  // DLOG(INFO) << "CollisionCheck in.";

  if (!IsOnGrid(start) || !IsOnGrid(end))
  {
    DLOG(INFO) << "start or end is not on grid";
    return false;
  }

  // should be convert to int for x and y;
  uint current_start_index = GetNode3DIndexOnGridMap(start.x(), start.y());
  uint current_end_index = GetNode3DIndexOnGridMap(end.x(), end.y());
  // check with in range obstacle
  if (in_range_obstacle_map_.find(current_start_index) != in_range_obstacle_map_.end())
  {
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_start_index])
    {
      // distance from this edge starting from start to end to obstacle should smaller than 0.5* vehicle length, then we can conclude that intersect!!!!
      if (Utility::GetDistanceFromPolygonToSegment(in_range_polygon, start, end) <= 0.5 * params_.vehicle_width)
      {
        // DLOG(INFO) << "intersect with obstacle.";
        return false;
      }
    }
    // DLOG(INFO) << " not intersect.";
  }
  else
  {
    DLOG(INFO) << "current start can`t be found in in_range_obstacle_map.";
  }
  // check with in range obstacle
  if (in_range_obstacle_map_.find(current_end_index) != in_range_obstacle_map_.end())
  {
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_end_index])
    {
      if (Utility::GetDistanceFromPolygonToSegment(in_range_polygon, start, end) <= 0.5 * params_.vehicle_width)
      {
        // DLOG(INFO) << "intersect with obstacle.";
        return false;
      }
    }
    // DLOG(INFO) << " not intersect.";
  }
  else
  {
    DLOG(INFO) << "current end can`t be found in in_range_obstacle_map.";
  }
  return true;
}

bool CollisionDetection::CollisionCheck(const ComputationalGeometry::Segment &segment)
{
  return CollisionCheck(segment.GetStart(), segment.GetEnd());
}

uint CollisionDetection::CalculateFineIndex(const float &x, const float &y, const float &t)
{
  uint out, x_int, y_int, t_index, x_index, y_index;
  const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
  float angle;
  if (abs(t) < 1e-4)
  {
    angle = 0;
  }
  else
  {
    angle = t;
  }
  x_int = (uint)x;
  y_int = (uint)y;
  t_index = (uint)(angle / delta_heading_in_rad);
  x_index = (x - x_int) * params_.position_resolution;
  y_index = (y - y_int) * params_.position_resolution;
  out = t_index * params_.position_resolution * params_.position_resolution + y_index * params_.position_resolution + x_index;
  // DLOG(INFO) << "coordinate is " << x << " " << y << " " << t << " x_int is " << x_int << " y_int is " << y_int << " x_index is " << x_index << " y_index is " << y_index << " t_index is " << t_index << " final index is " << out;

  return out;
}

std::vector<std::pair<float, Utility::AngleRange>> CollisionDetection::FindFreeAngleRangeAndObstacleAngleRange(const Node3D &node3d, bool consider_steering_angle_range)
{
  // DLOG(INFO) << "FindFreeAngleRangeAndObstacleAngleRange in:";
  std::vector<std::pair<float, Utility::AngleRange>> out;
  Utility::AngleRange steering_angle_range(0, Utility::ConvertDegToRad(359.9999));
  if (consider_steering_angle_range)
  {
    steering_angle_range = GetNode3DAvailableSteeringAngleRange(node3d);
  }
  // DLOG(INFO) << "current node is " << node3d.GetX() << " " << node3d.GetY() << " " << Utility::ConvertRadToDeg(node3d.GetT()) << " . steering angle range: start from " << Utility::ConvertRadToDeg(steering_angle_range.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(steering_angle_range));
  uint current_point_index = GetNode3DIndexOnGridMap(node3d);
  std::vector<Utility::AngleRange> free_angle_range_vec;
  free_angle_range_vec.emplace_back(steering_angle_range);
  // std::vector<Utility::Polygon> in_range_obstacle_vec;
  if (distance_angle_range_map_.find(current_point_index) != distance_angle_range_map_.end())
  {
    out = distance_angle_range_map_[current_point_index];
  }
  else
  {
    DLOG(WARNING) << "WARNING: current node index can`t be found in distance_angle_range_map_";
  }

  // DLOG(INFO) << "out size is " << out.size();
  Utility::AngleRange total_ar(-1, -1);
  std::vector<Utility::AngleRange> ar_vec;
  for (const auto &pair : out)
  {
    // DLOG(INFO) << "min distance is " << pair.first << " angle range start is " << Utility::ConvertRadToDeg(pair.second.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second));
    ar_vec.emplace_back(pair.second);
  }
  // check if ar in out is correct?
  std::vector<Utility::AngleRange> sorted_ar_vec = Utility::SortAngleRange(ar_vec);
  for (const auto &ar : sorted_ar_vec)
  {
    // DLOG(INFO) << "ar start from " << Utility::ConvertRadToDeg(ar.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(ar));
    total_ar = Utility::CombineAngleRange(ar, total_ar);
  }
  if (!Utility::IsEqual(steering_angle_range, total_ar))
  {
    DLOG_IF(INFO, out.size() == 0) << "out size is zero!";
    for (const auto &pair : out)
    {
      // DLOG(INFO) << "min distance is " << pair.first << " angle range start is " << Utility::ConvertRadToDeg(pair.second.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second));
      ar_vec.emplace_back(pair.second);
    }
  }
  DLOG_IF(WARNING, !Utility::IsEqual(steering_angle_range, total_ar)) << "WARNING: total ar start from " << Utility::ConvertRadToDeg(total_ar.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(total_ar)) << " is not equal to steering angle range start from " << Utility::ConvertRadToDeg(steering_angle_range.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(steering_angle_range)) << ", Something wrong!!!";

  DLOG_IF(WARNING, out.size() == 0) << "WARNING: Free angle range size is zero!!!!";
  // DLOG(INFO) << "FindFreeAngleRangeAndObstacleAngleRange out.";
  return out;
}

// checked, it`s correct for free angle range.
std::vector<std::pair<float, float>> CollisionDetection::SelectStepSizeAndSteeringAngle(const std::vector<std::pair<float, Utility::AngleRange>> &available_angle_range_vec, const Node3D &pred, const Node3D &goal, const int &number_of_successor)
{
  // DLOG(INFO) << "SelectStepSizeAndSteeringAngle in:";
  std::vector<std::pair<float, float>> out;
  float steering_angle, step_size_obstacle = 10000, step_size_free = obstacle_detection_range_;
  // 1. for step size, it should be weight*(min distance to obstacle) for obstacle angle range
  // find min distance
  for (const auto &pair : available_angle_range_vec)
  {
    step_size_obstacle = std::min(step_size_obstacle, pair.first);
  }

  // weight for step size should be inverse proportional to normalized obstacle density

  float weight_step_size;
  weight_step_size = GetStepSizeWeight(GetNormalizedObstacleDensity(pred));

  DLOG_IF(WARNING, weight_step_size == 0) << "WARNING: weight_step_size is zero!!!!";
  // DLOG(INFO) << "normalized obstacle density is " << GetNormalizedObstacleDensity(pred) << " step size weight for current node is " << weight_step_size;
  // if rest distance to obstacle is less than 1/2 vehicle length,
  float available_step_size_obstacle = ((step_size_obstacle - 0.5 * params_.vehicle_length) > 0) ? (step_size_obstacle - 0.5 * params_.vehicle_length) : 0;
  float available_step_size_free = ((step_size_free - 0.5 * params_.vehicle_length) > 0) ? (step_size_free - 0.5 * params_.vehicle_length) : 0;

  step_size_obstacle = weight_step_size * available_step_size_obstacle;
  step_size_free = weight_step_size * available_step_size_free;
  // make sure step size is larger than a predefined min distance otherwise too many successors
  if (params_.make_step_size_larger_than_one)
  {
    if (step_size_obstacle < 1)
    {
      // DLOG(INFO) << "step size is smaller than  predefined min distance, make it to one!!";

      if (available_step_size_obstacle > 1)
      {
        step_size_obstacle = 1;
      }
      else
      {
        step_size_obstacle = available_step_size_obstacle;
      }
    }
    // make sure step size is larger than a predefined min distance otherwise too many successors
    if (step_size_free < 1)
    {
      // DLOG(INFO) << "step size is smaller than  predefined min distance, make it to one!!";
      if (available_step_size_free > 1)
      {
        step_size_free = 1;
      }
      else
      {
        step_size_free = available_step_size_free;
      }
    }
  }
  // DLOG(INFO) << "obstacle step size is " << step_size_obstacle << " available step size is " << available_step_size_obstacle << " weighting is " << weight_step_size;
  // DLOG(INFO) << "free step size is " << step_size_free << " available step size is " << available_step_size_free << " weighting is " << weight_step_size;
  // DLOG_IF(WARNING, step_size_obstacle < 1) << "obstacle step size is " << step_size_obstacle;
  // DLOG_IF(WARNING, step_size_free < 1) << "free step size is " << step_size_free;
  // 3. find angle to goal
  float angle_to_goal;
  if (params_.add_one_more_successor)
  {
    angle_to_goal = -Utility::RadNormalization(pred.GetT() - goal.GetT());
    bool flag = true;
    if (flag)
    {
      angle_to_goal = -Utility::RadNormalization(pred.GetT() - Utility::GetAngle(pred, goal));
      // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(Utility::RadNormalization(Utility::GetAngle(pred, goal_)));
      // DLOG(INFO) << "steering angle is " << Utility::ConvertRadToDeg(angle_to_goal);
    }
  }

  // DLOG(INFO) << "size of available_angle_range_vec is " << available_angle_range_vec.size();
  for (const auto &pair : available_angle_range_vec)
  {

    // DLOG(INFO) << "current pair second first is " << Utility::ConvertRadToDeg(pair.second.first) << " range is " << Utility::ConvertRadToDeg(pair.second.second);
    // if steering angle range is zero, that means current node is blocked by obstacles
    if (pair.second.second == 0)
    {
      // DLOG(INFO) << "for current node, all available steering range is blocked by obstacle";
      steering_angle = Utility::RadNormalization((pair.second.first - pred.GetT()));
      std::pair<float, float> temp(pair.first, steering_angle);
      if (!Utility::DuplicateCheck(out, temp))
      {
        out.emplace_back(temp);
      }
      continue;
    }
    // 2. for free angle range, steering angle is angle range start + 1/(number of successors+1) * angle range.
    if (pair.first == obstacle_detection_range_)
    {
      // DLOG(INFO) << "in free angle range.";
      if (params_.add_one_more_successor)
      {
        if (Utility::IsAngleRangeInclude(pair.second, angle_to_goal))
        {
          // DLOG(INFO) << "one more node is in free angle range!";
          std::pair<float, float> temp = AddOneMoreStepSizeAndSteeringAngle(angle_to_goal, step_size_free, pred, goal);
          if (!Utility::DuplicateCheck(out, temp))
          {
            out.emplace_back(temp);
          }
        }
      }
      //  how to automatically determine number of successors?Answer: if range <5deg, just create one successor(avg),else, create n successors, each has (min+5*n)deg angles. DONE
      //  if free angle range is too small, then just create only one steering angle
      // DLOG(INFO) << "free angle range divide by (number of successor+1)= " << pair.second.second / (number_of_successor + 1);
      // DLOG(INFO) << "Utility::ConvertDegToRad(5) is " << Utility::ConvertDegToRad(5);
      // DLOG_IF(INFO, pair.second.second / (number_of_successor + 1) < Utility::ConvertDegToRad(5)) << "free angle range too small!!";
      // DLOG(INFO) << "free angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second));
      if (params_.fixed_number_of_steering_angle_in_free_angle_range)
      {
        if (pair.second.second / (number_of_successor + 1) > Utility::ConvertDegToRad(1))
        {
          for (int index = 1; index < number_of_successor + 1; ++index)
          {
            // target orientation=angle range start + (index/number of successor)* angle range range
            //  current orientation=current node theta
            // DLOG(INFO) << "loop to create successor for free angle range.";
            steering_angle = Utility::RadNormalization((pair.second.first + index * pair.second.second / (number_of_successor + 1) - pred.GetT()));
            std::pair<float, float> temp(step_size_free, steering_angle);
            if (!Utility::DuplicateCheck(out, temp))
            {
              // DLOG(INFO) << "steering angle for free angle range is " << Utility::ConvertRadToDeg(steering_angle);
              out.emplace_back(temp);
            }

            // DLOG(INFO) << "for free angle range, step size is " << step_size_free << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
          }
        }
        else
        {
          // DLOG(INFO) << "free angle range is too small!!";
          steering_angle = Utility::RadNormalization((pair.second.first + 0.5 * pair.second.second - pred.GetT()));
          std::pair<float, float> temp(step_size_free, steering_angle);
          if (!Utility::DuplicateCheck(out, temp))
          {
            // DLOG(INFO) << "steering angle for free angle range is " << Utility::ConvertRadToDeg(steering_angle);
            out.emplace_back(temp);
          }
          else
          {
            // DLOG(INFO) << "step size and steering angle are all ready in out!!!";
          }
        }
      }
      else
      {
        if (pair.second.second < Utility::ConvertDegToRad(3))
        {
          // DLOG(INFO) << "create successor for free angle range smaller than 3deg.";
          steering_angle = Utility::RadNormalization((pair.second.first + pair.second.second * 0.5 - pred.GetT()));
          std::pair<float, float> temp(step_size_free, steering_angle);
          if (!Utility::DuplicateCheck(out, temp))
          {
            out.emplace_back(temp);
          }
        }
        else
        {
          // DLOG(INFO) << "create successor for free angle range larger than 3deg.";
          float current_angle = pair.second.first;
          // float steering_angle_step_size = pair.second.second / number_of_successor;
          float steering_angle_step_size = Utility::ConvertDegToRad(5);
          while (current_angle <= Utility::GetAngleRangeEnd(pair.second))
          {
            steering_angle = Utility::RadNormalization((current_angle - pred.GetT()));
            std::pair<float, float> temp(step_size_free, steering_angle);
            if (!Utility::DuplicateCheck(out, temp))
            {
              out.emplace_back(temp);
            }
            current_angle += steering_angle_step_size;
          }
        }
      }
    }
    // 3. for obstacle angle range, two successors are created, first steering angle is range start, second steering angle is range end.
    else
    {
      // DLOG(INFO) << "obstacle angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second));
      if (params_.add_one_more_successor)
      {
        if (!params_.add_one_more_successor_only_in_free_angle_range)
        {
          if (Utility::IsAngleRangeInclude(pair.second, angle_to_goal))
          {
            // DLOG(INFO) << "one more node is in obstacle angle range!";
            std::pair<float, float> temp = AddOneMoreStepSizeAndSteeringAngle(angle_to_goal, step_size_obstacle, pred, goal);
            if (!Utility::DuplicateCheck(out, temp))
            {
              out.emplace_back(temp);
            }
          }
        }
      }
      // how to select steering angle in obstacle range: just select range start or end is not a good idea. if there is free angle range in steering angle range, then do not do steering in obstacle angle range, if there is no free angle range in steering angle range, steering toward free angle range which is closest to steering angle.
      if (params_.steering_angle_towards_free_angle_range_for_obstacle_angle_range)
      {
        // find angle in obstacle angle range which is closest to free angle range
        float closest_angle = IsCloseToFreeAngleRange(available_angle_range_vec, pred, pair.second.first, Utility::GetAngleRangeEnd(pair.second));
        steering_angle = Utility::RadNormalization((closest_angle - pred.GetT()));
        std::pair<float, float> temp(step_size_obstacle, steering_angle);
        if (!Utility::DuplicateCheck(out, temp))
        {
          // DLOG(INFO) << "steering angle for obstacle angle range is " << Utility::ConvertRadToDeg(steering_angle);
          out.emplace_back(temp);
        }
        // DLOG(INFO) << "for obstacle angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " . closest angle is " << Utility::ConvertRadToDeg(closest_angle);
      }
      else
      {
        //  steering angle is range start
        steering_angle = Utility::RadNormalization((pair.second.first - pred.GetT()));
        std::pair<float, float> temp(step_size_obstacle, steering_angle);
        if (!Utility::DuplicateCheck(out, temp))
        {
          out.emplace_back(temp);
        }

        // steering angle is range end
        steering_angle = Utility::RadNormalization((pair.second.first + pair.second.second - pred.GetT()));
        std::pair<float, float> temp1(step_size_obstacle, steering_angle);
        if (!Utility::DuplicateCheck(out, temp1))
        {
          out.emplace_back(temp1);
        }
      }
    }
  }

  for (const auto &pair : out)
  {
    DLOG_IF(INFO, (pred.GetX() > 76) && (pred.GetX() < 77) && (pred.GetY() > 1) && (pred.GetY() < 2)) << "current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT()) << " step size " << pair.first << " steering angle is " << Utility::ConvertRadToDeg(pair.second);
  }
  // DLOG(INFO) << "SelectStepSizeAndSteeringAngle out.";
  return out;
}
// checked, it`s correct.
std::vector<std::pair<float, float>> CollisionDetection::SweepDistanceAndAngle(const Node3D &node3d, const float &radius, bool consider_steering_angle)
{
  // key is angle in deg, value is no collision distance
  std::vector<std::pair<float, float>> out;
  Utility::AngleRange steering_angle_range;
  if (consider_steering_angle)
  {
    steering_angle_range = GetNode3DAvailableSteeringAngleRange(node3d);
  }
  else
  {
    steering_angle_range.first = 0;
    steering_angle_range.second = Utility::ConvertDegToRad(360);
  }

  float angle = steering_angle_range.first, distance = radius;
  for (; angle <= steering_angle_range.first + steering_angle_range.second; angle = angle + Utility::ConvertDegToRad(1))
  {
    // DLOG(INFO) << "angle is " << Utility::ConvertRadToDeg(angle);
    // 1. create segment according node3d, radius and angle
    // ComputationalGeometry::Segment segment = ComputationalGeometry::Segment(Utility::ConvertNod3DToVector2f(node3d), radius, angle);
    distance = FindNoCollisionDistance(node3d, radius, angle);

    out.emplace_back(std::pair<float, float>(angle, distance));
    // DLOG(INFO) << "angle is " << Utility::ConvertRadToDeg(angle) << " distance is " << distance;
  }
  // DLOG(INFO) << "SweepDistanceAndAngle out;";
  return out;
}

// this function seems ok for both no obstacle and obstacle , fully checked
float CollisionDetection::FindNoCollisionDistance(const Node3D &node3d, const float &radius, const float &angle)
{
  // DLOG(INFO) << "FindNoCollisionDistance in:";
  // DLOG(INFO) << "node3d is " << node3d.GetX() << " " << node3d.GetY() << " " << node3d.GetT() << " radius is " << radius << " angle is " << Utility::ConvertRadToDeg(angle);
  // set out to value of radius, if no obstacle in the angle direction, then return radius;
  float out = radius;
  // 1. calculate node3d index
  uint current_index = GetNode3DIndexOnGridMap(node3d);
  // 2. based on this index, find in range obstacles
  if (in_range_obstacle_map_.find(current_index) != in_range_obstacle_map_.end())
  {
    // 4. in this angle direction, check if there are some obstacles
    // 5. get distance from node3d to nearest obstacle in this direction
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_index])
    {
      float temp = Utility::GetDistanceFromPolygonToPointAtAngle(in_range_polygon, Utility::ConvertNod3DToVector2f(node3d), angle);
      // DLOG(INFO) << "temp is " << temp;
      if (temp < 0)
      {
        // DLOG(INFO) << "current in range polygon does not intersect with segment start at node3d along angle direction.";
        continue;
      }
      if (out > temp)
      {
        out = temp;
        // DLOG(INFO) << "out is " << out;
      }
    }
    // DLOG(INFO) << "for in range obstacle of node, not intersect.";
  }

  // DLOG(INFO) << "no collision distance is " << out;
  // DLOG(INFO) << "FindNoCollisionDistance out.";
  return out;
}

void CollisionDetection::BuildObstacleDensityMap(const float &range)
{
  // DLOG(INFO) << "BuildObstacleDensityMap in:";
  // build obstacle density map according to in range obstacle map, seems not correct, need to consider obstacle in a wider range.
  // so use the same way to build this map as in range obstacle map
  // can this two function be merged?,no, the in range obstacle map consider 2d index while this map consider 3d index.
  // definition of obstacle density:currently obstacle density is the number of obstacle in steering angle range and radius(parameter range)

  uint number_of_obstacles = 0;
  uint current_point_index;
  std::vector<Utility::Polygon> obstacle_vec;
  // this angle is the max steering angle for vehicle, unit is deg.
  float max_steering_angle = 30;
  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      // definition of obstacle density:currently obstacle density is the number of obstacle in radius(parameter range), do not consider current orientation, note: index is not fine index for this case
      if (!params_.consider_steering_angle_range_for_obstacle_density)
      {
        uint current_point_index = GetNode3DIndexOnGridMap(x, y);
        Eigen::Vector2f current_point(x, y);
        obstacle_vec.clear();
        for (const auto &polygon : obstacle_vec_)
        {
          if (params_.map_boundary_obstacle)
          {
            // check if this obstacle is map boundary obstacle,if yes, then skip this obstacle
            if (IsBoundaryObstacle(polygon))
            {
              continue;
            }
          }

          float distance = Utility::GetDistanceFromPolygonToPoint(polygon, current_point);

          if (distance < range)
          {
            number_of_obstacles++;
          }
        }
        in_range_obstacle_density_map_.emplace(current_point_index, number_of_obstacles);
        // DLOG(INFO) << "current node is " << x << " " << y << " " << theta << " fine index is " << current_point_index << " number of obstacles is " << number_of_obstacles;
        number_of_obstacles = 0;
      }
      else
      {
        for (uint theta = 0; theta < 360; theta = theta + Utility::ConvertRadToDeg(2 * M_PI / (float)params_.headings))
        {
          // DLOG(INFO) << "current node is " << x << " " << y << " " << theta;
          current_point_index = Get3DIndexOnGridMap(x, y, Utility::ConvertDegToRad(theta));
          Eigen::Vector2f current_point(x, y);
          obstacle_vec.clear();
          Utility::AngleRange current_angle_range;
          float current_angle_range_start = Utility::
              ConvertDegToRad(theta - max_steering_angle);
          float current_angle_range_range = Utility::
              ConvertDegToRad(2 * max_steering_angle);
          current_angle_range.first = current_angle_range_start;
          current_angle_range.second = current_angle_range_range;
          for (const auto &polygon : obstacle_vec_)
          {
            if (params_.map_boundary_obstacle)
            {
              // check if this obstacle is map boundary obstacle,if yes, then skip this obstacle
              if (IsBoundaryObstacle(polygon))
              {
                continue;
              }
            }
            Utility::AngleRange obstacle_angle_range = Utility::GetAngleRangeFromPointToPolygon(polygon, current_point);
            // 1. find obstacle in angle range current orientation +-max steering angle
            if (Utility::IsOverlap(obstacle_angle_range, current_angle_range) ||
                Utility::IsAngleRangeInclude(obstacle_angle_range, current_angle_range) ||
                Utility::IsAngleRangeInclude(current_angle_range, obstacle_angle_range))
            {
              // 2. find its distance from current node to obstacle
              float distance = Utility::GetDistanceFromPolygonToPoint(polygon, current_point);
              // 3. if distance is smaller than range, number of obstacle++
              if (distance < range)
              {
                number_of_obstacles++;
                // DLOG(INFO) << "obstacle origin is " << polygon[0].x() << " " << polygon[0].y() << " . current point is " << x << " " << y << " distance is " << distance;
                // DLOG(INFO) << "obstacle is in the range, distance is " << distance;
              }
            }
          }
          in_range_obstacle_density_map_.emplace(current_point_index, number_of_obstacles);
          // DLOG(INFO) << "current node is " << x << " " << y << " " << theta << " fine index is " << current_point_index << " number of obstacles is " << number_of_obstacles;
          number_of_obstacles = 0;
        }
      }
    }
  }
  // DLOG(INFO) << "BuildObstacleDensityMap out.";
}

uint CollisionDetection::GetNode3DFineIndex(const Node3D &node3d)
{
  return CalculateFineIndex(node3d.GetX(), node3d.GetY(), node3d.GetT());
}

void CollisionDetection::BuildNormalizedObstacleDensityMap()
{
  // DLOG(INFO) << "BuildNormalizedObstacleDensityMap in:";
  // delta here is a small number to ensure denominator is not zero
  float normalized_obstacle_density, delta = 0.01;
  float max_density = 0, min_density = 1000000;
  // normalize use formula = (x-min)/(max-min)
  for (const auto &element : in_range_obstacle_density_map_)
  {

    // DLOG(INFO) << "current density is " << element.second;
    // DLOG(INFO) << "current max density is " << max_density;
    // DLOG(INFO) << "current min density is " << min_density;
    if (element.second > max_density)
    {
      max_density = element.second;
    }
    if (element.second < min_density)
    {
      min_density = element.second;
    }
  }
  // DLOG(INFO) << "min density is " << min_density;
  // DLOG(INFO) << "max density is " << max_density;
  for (const auto &element : in_range_obstacle_density_map_)
  {
    normalized_obstacle_density = (element.second - min_density) / (max_density - min_density + delta);
    // treatment for zero
    // if (normalized_obstacle_density == 0)
    // {
    //   normalized_obstacle_density = normalized_obstacle_density + 0.001;
    // }
    normalized_obstacle_density_map_.emplace(element.first, normalized_obstacle_density);
    // DLOG(INFO) << "current fine index is " << element.first << " normalized obstacle density is " << normalized_obstacle_density;
  }
  // DLOG(INFO) << "BuildNormalizedObstacleDensityMap out.";
}

float CollisionDetection::GetNormalizedObstacleDensity(const Node3D &node3d)
{
  // DLOG(INFO) << "GetNormalizedObstacleDensity in:";
  float obstacle_density;
  uint current_index;
  if (!params_.consider_steering_angle_range_for_obstacle_density)
  {
    current_index = GetNode3DIndexOnGridMap(node3d);
  }
  else
  {
    current_index = GetNode3DFineIndex(node3d);
  }
  if (normalized_obstacle_density_map_.find(current_index) != normalized_obstacle_density_map_.end())
  {
    obstacle_density = normalized_obstacle_density_map_[current_index];
    DLOG_IF(WARNING, std::isnan(obstacle_density)) << "WARNING: normalized obstacle density is NAN!!!";
  }
  else
  {
    DLOG(WARNING) << "WARNING: current index can`t be found in the obstacle density map!!!";
  }
  // DLOG(INFO) << "GetNormalizedObstacleDensity out.";
  return obstacle_density;
}

uint CollisionDetection::Get3DIndexOnGridMap(const float &x, const float &y, const float &theta)
{
  uint out, x_int, y_int, t_index;
  const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
  float angle;
  if (abs(theta) < 1e-4)
  {
    angle = 0;
  }
  else
  {
    angle = theta;
  }
  x_int = (uint)x;
  y_int = (uint)y;
  t_index = (uint)(angle / delta_heading_in_rad);

  out = t_index * params_.headings * grid_ptr_->info.width + y_int * grid_ptr_->info.width + x_int;
  // DLOG(INFO) << "coordinate is " << x << " " << y << " " << Utility::ConvertRadToDeg(theta) << " x_int is " << x_int << " y_int is " << y_int << " t_index is " << t_index << " final index is " << out;
  return out;
}

uint CollisionDetection::Get3DIndexOnGridMap(const Node3D &node3d)
{
  return Get3DIndexOnGridMap(node3d.GetX(), node3d.GetY(), node3d.GetT());
}

float CollisionDetection::GetStepSizeWeight(const float &normalized_obstacle_density)
{
  float step_size_weight;
  bool flag = false;
  if (flag)
  {
    step_size_weight = 1 - normalized_obstacle_density;
  }
  else
  {
    //  make weight step size in range (0,0.9];
    //  manually make it in range(0.1,0.9]
    step_size_weight = -0.8 * normalized_obstacle_density + 0.9;
  }
  return step_size_weight;
}
// checked correct.
void CollisionDetection::AddMapBoundaryAsObstacle(std::vector<Utility::Polygon> &obstacle_vec)
{
  // need to add map boundary to obstacle vec_ for other consideration
  // key point here is to make origin for these boundary obstacle outside map, purpose is to distinguish these boundary obstacle from normal ones.
  // bottom obstacle as bottom boundary
  Utility::Polygon current_polygon;
  current_polygon = Utility::CreatePolygon(Eigen::Vector2f(0, -1), grid_ptr_->info.width, 1);
  obstacle_vec.emplace_back(current_polygon);
  // right obstacle as right boundary
  current_polygon = Utility::CreatePolygon(Eigen::Vector2f(grid_ptr_->info.width, -1), 1, grid_ptr_->info.height + 2);
  obstacle_vec.emplace_back(current_polygon);
  // top obstacle as top boundary
  current_polygon = Utility::CreatePolygon(Eigen::Vector2f(-1, grid_ptr_->info.height), grid_ptr_->info.width + 2, 1);
  obstacle_vec.emplace_back(current_polygon);
  // left obstacle as left boundary
  current_polygon = Utility::CreatePolygon(Eigen::Vector2f(-1, 0), 1, grid_ptr_->info.height);
  obstacle_vec.emplace_back(current_polygon);
}
// checked, no issue
bool CollisionDetection::IsBoundaryObstacle(const Utility::Polygon &obstacle)
{
  // DLOG(INFO) << "IsBoundaryObstacle in:";
  // if the start point is outside map, then this polygon is boundary obstacle
  if (obstacle.size() != 0)
  {
    if (!IsOnGrid(obstacle[0]))
    {
      // DLOG(INFO) << "current obstacle is a boundary obstacle";
      // DLOG(INFO) << "IsBoundaryObstacle out.";
      return true;
    }
  }
  else
  {
    DLOG(WARNING) << "WARNING: obstacle size is zero!!!!";
  }
  // DLOG(INFO) << "IsBoundaryObstacle out.";
  return false;
}

std::pair<float, float> CollisionDetection::AddOneMoreStepSizeAndSteeringAngle(const float &angle_to_goal, const float &step_size, const Node3D &pred, const Node3D &goal)
{
  std::pair<float, float> out;
  // 1. first find distance from current node to goal position
  float distance_to_goal = Utility::GetDistance(pred, goal);
  // DLOG(INFO) << "distance to goal is " << distance_to_goal;
  // 2. if distance to goal is less than step size above. than make it new step size, otherwise use old one

  float new_step_size, steering_angle;
  if (distance_to_goal < step_size)
  {
    new_step_size = distance_to_goal;
  }
  else
  {
    new_step_size = step_size;
  }
  // DLOG(INFO) << "step size is " << step_size;
  // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(angle_to_goal);
  // 4. if angle to goal is in the steering angle range(current orientation +-30deg), then make it steering angle, otherwise 30 or -30 to make angle to goal smaller

  if (std::abs(angle_to_goal) > Utility::ConvertDegToRad(30))
  {
    // if (pred.GetT() > Utility::RadNormalization(goal_.GetT()) && pred.GetT() < Utility::RadNormalization((goal_.GetT() + Utility::ConvertDegToRad(180))))
    if (angle_to_goal < -Utility::ConvertDegToRad(30))
    {
      // right is negative
      steering_angle = -Utility::ConvertDegToRad(30);
    }
    else
      // left is positive
      steering_angle = Utility::ConvertDegToRad(30);
  }

  else
  {
    steering_angle = angle_to_goal;
  }
  out.first = new_step_size;
  out.second = steering_angle;

  DLOG(INFO) << "current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT()) << " and goal orientation is " << Utility::ConvertRadToDeg(goal.GetT()) << " one more step size is " << new_step_size << " and steering angle pair is " << Utility::ConvertRadToDeg(steering_angle);

  return out;
}

float CollisionDetection::IsCloseToFreeAngleRange(const std::vector<std::pair<float, Utility::AngleRange>> &available_angle_range_vec, const Node3D &current, const float &angle1, const float &angle2)
{
  // first check if there is free angle range in available_angle_range_vec, if yes, find closest angle to its free angle range
  // this flag indicates whether there is free angle range in steering angle
  bool free_angle_range_in_steering_angle = false;
  float distance_angle1 = 10000, distance_angle2 = 10000;
  for (const auto &pair : available_angle_range_vec)
  {
    // if this is a free angle range
    if (pair.first == obstacle_detection_range_)
    {
      free_angle_range_in_steering_angle = true;
    }
  }
  if (free_angle_range_in_steering_angle)
  {
    for (const auto &pair : available_angle_range_vec)
    {
      // if this is a free angle range
      if (pair.first == obstacle_detection_range_)
      {
        // check this free angle range is in steering angle range
        float temp1, temp2;
        temp1 = Utility::GetAngleDistance(angle1, pair.second);
        temp2 = Utility::GetAngleDistance(angle2, pair.second);
        distance_angle1 = std::min(temp1, distance_angle1);
        distance_angle2 = std::min(temp2, distance_angle2);
      }
    }
  }
  // second, if no free angle range available_angle_range_vec, then find closest angle to its free angle range not in steering angle range
  else
  {
    uint current_point_index = GetNode3DIndexOnGridMap(current);
    float distance_angle1 = 10000, distance_angle2 = 10000;

    // Utility::AngleRange steering_angle_range = GetNode3DAvailableSteeringAngleRange(current);
    if (distance_angle_range_map_.find(current_point_index) != distance_angle_range_map_.end())
    {
      for (const auto &pair : distance_angle_range_map_[current_point_index])
      {
        // if this is a free angle range
        if (pair.first == obstacle_detection_range_)
        {
          // check this free angle range is in steering angle range
          float temp1, temp2;
          temp1 = Utility::GetAngleDistance(angle1, pair.second);
          temp2 = Utility::GetAngleDistance(angle2, pair.second);
          distance_angle1 = std::min(temp1, distance_angle1);
          distance_angle2 = std::min(temp2, distance_angle2);
        }
      }
    }
    else
    {
      DLOG(WARNING) << "WARNING: current node is not in distance_angle_range_map_!!";
    }
  }

  if (distance_angle1 > distance_angle2)
  {
    // DLOG(INFO) << "for node " << current.GetX() << " " << current.GetY() << " " << current.GetT() << " angle2: " << Utility::ConvertRadToDeg(angle2) << " is closest to free angle range";
    return angle2;
  }
  else
  {
    // DLOG(INFO) << "for node " << current.GetX() << " " << current.GetY() << " " << current.GetT() << " angle1: " << Utility::ConvertRadToDeg(angle1) << " is closest to free angle range";
    return angle1;
  }
}