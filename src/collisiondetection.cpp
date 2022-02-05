#include "collisiondetection.h"

using namespace HybridAStar;

bool CollisionDetection::IsTraversable(const std::shared_ptr<Node2D> &node2d_ptr)
{
  if (!IsOnGrid(node2d_ptr))
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
  getConfiguration(node2d_ptr, x, y, t);
  // 2D collision test
  if (t == 99)
  {
    return !grid_ptr_->data[node2d_ptr->GetIdx()];
  }
  if (true)
  {
    cost = configurationTest(x, y, t) ? 0 : 1;
  }

  return cost <= 0;
};
bool CollisionDetection::IsTraversable(const std::shared_ptr<Node3D> &node3d_ptr)
{
  return IsTraversable(*node3d_ptr);
};

bool CollisionDetection::IsTraversable(const Node3D &node3d)
{
  if (!IsOnGrid(node3d))
  {
    return false;
  }
  /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */

  float x;
  float y;
  float t;
  // assign values to the configuration
  getConfiguration(node3d, x, y, t);
  // 2D collision test
  if (t == 99)
  {
    return !grid_ptr_->data[node3d.GetIdx()];
  }
  if (node3d.GetPred() != nullptr)
  {
    // if (configurationTest(x, y, t))
    if (configurationTest(x, y, t) && configurationTest(node3d, *node3d.GetPred()))
    {
      return true;
    }
  }
  else
  {
    if (configurationTest(x, y, t))
    {
      return true;
    }
  }

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
  return x >= 0 && x < (int)grid_ptr_->info.width &&
         y >= 0 && y < (int)grid_ptr_->info.height;
}
bool CollisionDetection::configurationTest(const float &x, const float &y, const float &t)
{
  if (params_.enable_collision_lookup)
  {
    //1. find index in the collision lookup map
    uint index = GetNode3DIndexOnGridMap(x, y);
    //2. check collision lookup table
    if (collision_lookup_.find(index) != collision_lookup_.end())
    {
      uint fine_index = CalculateFineIndex(x, y, t);
      DLOG(INFO) << "index is " << index << " fine index is " << fine_index;
      Utility::Polygon polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y), params_.vehicle_length, params_.vehicle_width, t);
      bool collision_result;
      collision_result = CollsionCheck(polygon);
      return collision_result;
      if (collision_lookup_[index].find(fine_index) != collision_lookup_[index].end())
      {
        DLOG(INFO) << "collision result is " << collision_lookup_[index][fine_index];
        return collision_lookup_[index][fine_index];
      }
      DLOG(WARNING) << "fine index can`t be found in the collision lookup map, coordinate is " << x << " " << y << " " << Utility::ConvertRadToDeg(t);
    }
    else
    {
      DLOG(WARNING) << "index can`t be found in the collision lookup map, coordinate is " << x << " " << y << " " << Utility::ConvertRadToDeg(t);
    }
  }
  else
  {
    Utility::Polygon polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y), params_.vehicle_length, params_.vehicle_width, t);
    bool collision_result;
    collision_result = CollsionCheck(polygon);
    if (!collision_result)
    {
      DLOG(INFO) << "in collision, coordinate is " << x << " " << y << " " << t;
    }
    return collision_result;
  }
  return false;
}
bool CollisionDetection::configurationTest(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
{
  bool collision_result;
  collision_result = CollsionCheck(start, end);
  if (!collision_result)
  {
    DLOG(INFO) << "in collision, segment start at " << start.x() << " " << start.y() << " end at " << end.x() << " " << end.y();
  }
  return collision_result;
}
bool CollisionDetection::configurationTest(const Node3D &start, const Node3D &end)
{

  return configurationTest(Utility::ConvertNod3DToVector2f(start), Utility::ConvertNod3DToVector2f(end));
}
// bool CollisionDetection::configurationTest(const float &x, const float &y, const float &t)
// {
//   const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
//   int X = (int)x;
//   int Y = (int)y;
//   int iX = (int)((x - (long)x) * params_.position_resolution);
//   iX = iX > 0 ? iX : 0;
//   int iY = (int)((y - (long)y) * params_.position_resolution);
//   iY = iY > 0 ? iY : 0;
//   int iT = (int)(t / delta_heading_in_rad);
//   iT = iT > 0 ? iT : 0;
//   int idx = iY * params_.position_resolution * params_.headings + iX * params_.headings + iT;
//   int cX, cY;

//   for (int i = 0; i < collisionLookup[idx].length; ++i)
//   {
//     cX = (X + collisionLookup[idx].pos[i].x);
//     cY = (Y + collisionLookup[idx].pos[i].y);

//     // make sure the configuration coordinates are actually on the grid
//     if (cX >= 0 && (unsigned int)cX < grid_ptr_->info.width && cY >= 0 && (unsigned int)cY < grid_ptr_->info.height)
//     {
//       if (grid_ptr_->data[cY * grid_ptr_->info.width + cX])
//       {
//         return false;
//       }
//     }
//   }

//   return true;
// }

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
Utility::AngleRangeVec CollisionDetection::FindFreeAngleRange(const Node3D &node3d)
{
  Utility::AngleRangeVec available_angle_range_vec;
  Utility::AngleRange available_angle_range = GetNode3DAvailableAngleRange(node3d);
  //find obstacle in range
  uint node_index = GetNode3DIndexOnGridMap(node3d);
  available_angle_range_vec.emplace_back(available_angle_range);
  // for (const auto &angle_range : available_angle_range_vec)
  // {
  //   DLOG(INFO) << "angle range start is " << angle_range.first << " " << angle_range.second;
  // }
  if (in_range_obstacle_map_.find(node_index) != in_range_obstacle_map_.end())
  {
    for (const auto &polygon : in_range_obstacle_map_.at(node_index))
    {
      // DLOG(INFO) << "polygon origin is " << polygon[0].x() << " " << polygon[0].y();
      //get obstacle angle range
      Utility::AngleRange obstacle_angle_range = Utility::GetAngleRangeFromPointToPolygon(polygon, Utility::ConvertNod3DToVector2f(node3d));
      // DLOG(INFO) << "obstacle angle range start: " << Utility::ConvertRadToDeg(obstacle_angle_range.first) << " range is " << Utility::ConvertRadToDeg(obstacle_angle_range.second);
      for (uint index = 0; index < available_angle_range_vec.size(); ++index)
      {
        // DLOG(INFO) << "angle range start is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].first) << " range is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].second);
        //see if it is overlap with node3d steering range
        if (Utility::IsOverlap(obstacle_angle_range, available_angle_range_vec[index]))
        {

          available_angle_range_vec[index] = Utility::MinusAngleRange(available_angle_range_vec[index], obstacle_angle_range);
          DLOG(INFO) << "two angle range are overlapped";
          // DLOG(INFO) << "new angle range start is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].first) << " range is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].second);
        }
        //obstacle angle range is larger and include the steering angle range,set steering angle range to zero
        else if (Utility::IsAngleRangeInclude(obstacle_angle_range, available_angle_range_vec[index]))
        {
          if (available_angle_range_vec[index].first - obstacle_angle_range.first > obstacle_angle_range.second + obstacle_angle_range.first - (available_angle_range_vec[index].first + available_angle_range_vec[index].second))
          {
            DLOG(INFO) << "angle range end is more close to obstacle range end, set angle range start to end and range to zero.";
            available_angle_range_vec[index].first = available_angle_range_vec[index].first + available_angle_range_vec[index].second;
          }
          available_angle_range_vec[index].second = 0;
          DLOG(INFO) << "obstacle range is including the available angle range";
        }
        //checked
        //obstacle angle range is smaller and included by the steering angle range,generate two steering angle range
        else if (Utility::IsAngleRangeInclude(available_angle_range_vec[index], obstacle_angle_range))
        {
          // DLOG(INFO) << "angle range start is " << Utility::ConvertRadToDeg(angle_range.first) << " range is " << Utility::ConvertRadToDeg(angle_range.second);
          // DLOG(INFO) << "obstacle angle range start: " << Utility::ConvertRadToDeg(obstacle_angle_range.first) << " range is " << Utility::ConvertRadToDeg(obstacle_angle_range.second);
          // DLOG(INFO) << "angle range start is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].first) << " range is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].second);
          // DLOG(INFO) << "obstacle angle range start: " << Utility::ConvertRadToDeg(obstacle_angle_range.first) << " range is " << Utility::ConvertRadToDeg(obstacle_angle_range.second);
          // DLOG(INFO) << "angle range start is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].first) << " range is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].second);
          // DLOG(INFO) << "obstacle angle range start: " << Utility::ConvertRadToDeg(obstacle_angle_range.first) << " range is " << Utility::ConvertRadToDeg(obstacle_angle_range.second);
          float new_angle_range_start = obstacle_angle_range.first + obstacle_angle_range.second;
          float new_angle_range_range = available_angle_range_vec[index].first + available_angle_range_vec[index].second - (obstacle_angle_range.first + obstacle_angle_range.second) > 2 * M_PI ? available_angle_range_vec[index].first + available_angle_range_vec[index].second - (obstacle_angle_range.first + obstacle_angle_range.second) - 2 * M_PI : available_angle_range_vec[index].first + available_angle_range_vec[index].second - (obstacle_angle_range.first + obstacle_angle_range.second);

          Utility::AngleRange new_angle_range(new_angle_range_start, new_angle_range_range);
          available_angle_range_vec.emplace_back(new_angle_range);

          available_angle_range_vec[index].second = obstacle_angle_range.first - available_angle_range_vec[index].first > 0 ? obstacle_angle_range.first - available_angle_range_vec[index].first : obstacle_angle_range.first - available_angle_range_vec[index].first + 2 * M_PI;
          // DLOG(INFO) << "angle range start is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].first) << " range is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].second);
          // DLOG(INFO) << "obstacle angle range start: " << Utility::ConvertRadToDeg(obstacle_angle_range.first) << " range is " << Utility::ConvertRadToDeg(obstacle_angle_range.second);
          DLOG(INFO) << "available range is including the obstacle angle range";
          // DLOG(INFO) << "new angle range start is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].first) << " range is " << Utility::ConvertRadToDeg(available_angle_range_vec[index].second) << " another one start is " << Utility::ConvertRadToDeg(available_angle_range_vec.back().first) << " range is " << Utility::ConvertRadToDeg(available_angle_range_vec.back().second);
          // for (const auto &pair : available_angle_range_vec)
          // {
          //   DLOG(INFO) << "pair first " << Utility::ConvertRadToDeg(pair.first) << " " << Utility::ConvertRadToDeg(pair.second);
          // }
        }
        else
        {
          // DLOG(INFO) << "these two ranges are far away.";
        }
      }
    }
  }
  // for (const auto &angle_range : available_angle_range_vec)
  // {
  //   DLOG(INFO) << "angle range start is " << Utility::ConvertRadToDeg(angle_range.first) << " " << Utility::ConvertRadToDeg(angle_range.second);
  // }
  return available_angle_range_vec;
}

std::vector<std::pair<float, Utility::AngleRange>> CollisionDetection::FindFreeAngleRangeAndStepSize(const Node3D &node3d)
{
  std::vector<std::pair<float, Utility::AngleRange>> out;
  Utility::AngleRange available_angle_range = GetNode3DAvailableAngleRange(node3d);
  //find obstacle in range
  uint node_index = GetNode3DIndexOnGridMap(node3d);
  float step_size_free = params_.free_step_size;
  out.emplace_back(std::pair<float, Utility::AngleRange>(step_size_free, available_angle_range));

  if (distance_angle_range_map_.find(node_index) != distance_angle_range_map_.end())
  {
    // DLOG(INFO) << "size of distance_angle_range_map_ at index: " << node_index << " is " << distance_angle_range_map_.at(node_index).size();
    for (const auto &polygon_pair : distance_angle_range_map_.at(node_index))
    {
      //get obstacle angle range
      Utility::AngleRange obstacle_angle_range = polygon_pair.second;
      float distance_to_obstacle = polygon_pair.first;
      // DLOG(INFO) << "distance to obstacle is " << distance_to_obstacle;
      for (uint index = 0; index < out.size(); ++index)
      {
        //see if it is overlap with node3d steering range
        if (Utility::IsOverlap(obstacle_angle_range, out[index].second))
        {
          out[index].second = Utility::MinusAngleRange(out[index].second, obstacle_angle_range);
          // since this range is free, we would go further
          out[index].first = step_size_free;
          // DLOG(INFO) << "two angle range are overlapped";
        }
        //obstacle angle range is larger and include the steering angle range,set steering angle range to zero
        else if (Utility::IsAngleRangeInclude(obstacle_angle_range, out[index].second))
        {
          if (out[index].second.first - obstacle_angle_range.first > obstacle_angle_range.second + obstacle_angle_range.first - (out[index].second.first + out[index].second.second))
          {
            // DLOG(INFO) << "angle range end is more close to obstacle range end, set angle range start to end and range to zero.";
            out[index].second.first = out[index].second.first + out[index].second.second;
          }
          out[index].second.second = 0;
          // since this range is not free, step size should be 1/10 of obstacle distance
          out[index].first = distance_to_obstacle / 10;

          // DLOG(INFO) << "obstacle range is including the available angle range";
        }
        //checked
        //obstacle angle range is smaller and included by the steering angle range,generate two steering angle range
        else if (Utility::IsAngleRangeInclude(out[index].second, obstacle_angle_range))
        {

          float new_angle_range_start = obstacle_angle_range.first + obstacle_angle_range.second;
          float new_angle_range_range = out[index].second.first + out[index].second.second - (obstacle_angle_range.first + obstacle_angle_range.second) > 2 * M_PI ? out[index].second.first + out[index].second.second - (obstacle_angle_range.first + obstacle_angle_range.second) - 2 * M_PI : out[index].second.first + out[index].second.second - (obstacle_angle_range.first + obstacle_angle_range.second);

          Utility::AngleRange new_angle_range(new_angle_range_start, new_angle_range_range);

          out.emplace_back(std::pair<float, Utility::AngleRange>(step_size_free, new_angle_range));

          out[index].second.second = obstacle_angle_range.first - out[index].second.first > 0 ? obstacle_angle_range.first - out[index].second.first : obstacle_angle_range.first - out[index].second.first + 2 * M_PI;
          out[index].first = step_size_free;
          // DLOG(INFO) << "available range is including the obstacle angle range";
        }
        else
        {
          out[index].first = step_size_free;
          // DLOG(INFO) << "these two ranges are far away.";
        }
      }
    }
  }
  else
  {
    // DLOG(INFO) << "index not found in map";
  }
  return out;
}
//checked
void CollisionDetection::SetObstacleVec()
{
  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      if (grid_ptr_->data[y * grid_ptr_->info.width + x])
      {
        obstacle_vec_.emplace_back(Utility::CreatePolygon(Eigen::Vector2f(x, y)));
        // DLOG(INFO) << "obstacle origin is " << x << " " << y;
      }
    }
  }
}
//checked
void CollisionDetection::SetInRangeObstacle(const float &range)
{
  std::vector<Utility::Polygon> obstacle_vec;
  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      uint current_point_index = y * grid_ptr_->info.width + x;
      Eigen::Vector2f current_point(x, y);
      obstacle_vec.clear();
      for (const auto &polygon : obstacle_vec_)
      {
        float distance = Utility::GetDistanceFromPolygonToPoint(polygon, current_point);
        if (distance < range)
        {
          obstacle_vec.emplace_back(polygon);
          // DLOG(INFO) << "obstacle origin is " << polygon[0].x() << " " << polygon[0].y() << " . current point is " << x << " " << y << " distance is " << distance;
          // DLOG(INFO) << "obstacle is in the range, distance is " << distance;
        }
      }
      in_range_obstacle_map_.emplace(current_point_index, obstacle_vec);
    }
  }
}

uint CollisionDetection::GetNode3DIndexOnGridMap(const Node3D &node3d)
{

  return GetNode3DIndexOnGridMap(node3d.GetX(), node3d.GetY());
}

uint CollisionDetection::GetNode3DIndexOnGridMap(const float &x, const float &y)
{
  // DLOG(INFO) << "coordinate is " << x << " " << y << " "
  //  << " final index is " << (uint)y * map_width_ + (uint)x;
  return (uint)y * map_width_ + (uint)x;
}
//checked, it`s correct.
Utility::AngleRange CollisionDetection::GetNode3DAvailableAngleRange(const Node3D &node3d)
{
  // DLOG(INFO) << "GetNode3DAvailableAngleRange in:";
  Utility::AngleRange out;
  float current_heading = Utility::RadNormalization(node3d.GetT());
  const float max_steering_angle = Utility::ConvertDegToRad(30);
  float starting_angle, range;
  //for forward and backward, steering angle is the same. no need to distinguish these two condition, since angle range is in start angle and range format.
  // if (current_heading > Utility::ConvertDegToRad(150) || current_heading < Utility::ConvertDegToRad(-150))
  // {
  //   starting_angle = Utility::RadToZeroTo2P(current_heading - max_steering_angle);
  //   range = Utility::RadToZeroTo2P(2 * max_steering_angle);
  // }
  // else
  // {
  //   starting_angle = Utility::RadNormalization(current_heading - max_steering_angle);
  //   range = Utility::RadNormalization(2 * max_steering_angle);
  // }
  starting_angle = Utility::RadToZeroTo2P(current_heading - max_steering_angle);
  range = (2 * max_steering_angle);
  // DLOG(INFO) << "starting angle is :" << Utility::ConvertRadToDeg(starting_angle) << " range is " << Utility::ConvertRadToDeg(range);
  out = std::make_pair(starting_angle, range);
  // DLOG(INFO) << "GetNode3DAvailableAngleRange out.";
  return out;
}

void CollisionDetection::SetDistanceAngleRangeMap()
{
  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      uint current_point_index = y * grid_ptr_->info.width + x;
      Eigen::Vector2f current_point(x, y);
      std::vector<Utility::Polygon> in_range_obstacle_vec;
      if (in_range_obstacle_map_.find(current_point_index) != in_range_obstacle_map_.end())
      {
        std::vector<std::pair<float, Utility::AngleRange>> distance_angle_range_vec;
        for (const auto &polygon : in_range_obstacle_map_[current_point_index])
        {
          float distance = Utility::GetDistanceFromPolygonToPoint(polygon, current_point);
          Utility::AngleRange angle_range = Utility::GetAngleRangeFromPointToPolygon(polygon, current_point);
          std::pair<float, Utility::AngleRange> temp = std::make_pair(distance, angle_range);
          distance_angle_range_vec.emplace_back(temp);
        }
        distance_angle_range_map_.emplace(current_point_index, distance_angle_range_vec);
      }
    }
  }
}

std::vector<std::pair<float, Utility::AngleRange>> CollisionDetection::GetDistanceAngleRangeVec(const HybridAStar::Node3D &current_node)
{
  std::vector<std::pair<float, Utility::AngleRange>> out;
  uint current_index = GetNode3DIndexOnGridMap(current_node);
  if (distance_angle_range_map_.find(current_index) != distance_angle_range_map_.end())
  {
    out = distance_angle_range_map_.at(current_index);
  }
  return out;
}

// uint CollisionDetection::CalculateNode3DIndex(const float &x, const float &y, const float &theta) const
// {
//   uint out;
//   const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
//   float angle;
//   if (abs(theta) < 1e-4)
//   {
//     angle = 0;
//   }
//   else
//   {
//     angle = theta;
//   }
//   // DLOG(INFO) << "angle is " << angle;
//   // DLOG(INFO) << "delta heading in rad is " << delta_heading_in_rad;
//   // DLOG(INFO) << "(int)(theta / delta_heading_in_rad) " << (int)(theta / delta_heading_in_rad);
//   out = (int)(angle / delta_heading_in_rad) * map_width_ * map_height_ * params_.position_resolution * params_.position_resolution + (int)(y * params_.position_resolution) * map_width_ + (int)(x * params_.position_resolution);

//   return out;
// }

//where to put this collision table in lookup_table.cpp or collisiondetection..cpp?
void CollisionDetection::BuildCollisionLookupTable()
{
  DLOG(INFO) << "BuildCollisionLookupTable start:";
  collision_lookup_.clear();
  //0. three for loop to loop every point in the map, similar to lookup_table.cpp
  uint index, fine_index;
  int x_count, y_count;
  bool collision_result;
  std::unordered_map<uint, bool> collision_result_map;
  float x = 0, y = 0, theta = 0;

  const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
  while (x < map_width_)
  {
    y = 0;
    // theta = 0;
    while (y < map_height_)
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
            DLOG(INFO) << "current index is " << index << " , fine index is " << fine_index << " coordinate is " << x << " " << y << " " << theta;
            //1. create polygon accord to its coordinate(x,y), then rotate according to heading angle t.
            // case: some certain case this polygon is outside map
            if ((x <= params_.vehicle_length / 2 && y <= params_.vehicle_width / 2) ||
                (x >= map_width_ - params_.vehicle_length / 2 && y <= params_.vehicle_width / 2) ||
                (x <= params_.vehicle_length / 2 && y >= map_height_ - params_.vehicle_width / 2) ||
                (x >= map_width_ - params_.vehicle_length / 2 && y >= map_height_ - params_.vehicle_width / 2))
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
              collision_result = CollsionCheck(polygon);
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
  DLOG(INFO) << "BuildCollisionLookupTable done.";
}

bool CollisionDetection::CollsionCheck(const Utility::Polygon &polygon)
{
  // DLOG(INFO) << "CollisionCheck in.";
  for (const auto &point : polygon)
  {
    if (!IsOnGrid(point))
    {
      DLOG(INFO) << "polygon is not on grid: point is " << point.x() << " " << point.y();
      return false;
    }
  }
  // should be convert to int for x and y;
  uint current_point_index = GetNode3DIndexOnGridMap(polygon[0].x(), polygon[0].y());
  //check if this polygon inside map or outside map
  Utility::Polygon map_boundary = Utility::CreatePolygon(map_width_, map_height_);
  // DLOG(INFO) << "current polygon start is " << polygon[0].x() << " " << polygon[0].y() << " index is " << current_point_index;

  if (Utility::IsPolygonIntersectWithPolygon(polygon, map_boundary))
  {
    DLOG(INFO) << "intersect with map boundary.";
    return false;
  }
  //check with in range obstacle
  if (in_range_obstacle_map_.find(current_point_index) != in_range_obstacle_map_.end())
  {
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_point_index])
    {
      if (Utility::IsPolygonIntersectWithPolygon(polygon, in_range_polygon))
      {
        DLOG(INFO) << "intersect with obstacle.";
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
  //2. find obstacle in a certain range, this range is determined be vehicle parameter
}

bool CollisionDetection::CollsionCheck(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
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
  //check with in range obstacle
  if (in_range_obstacle_map_.find(current_start_index) != in_range_obstacle_map_.end())
  {
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_start_index])
    {
      if (Utility::GetDistanceFromPolygonToSegment(in_range_polygon, start, end) <= params_.vehicle_width)
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
  //check with in range obstacle
  if (in_range_obstacle_map_.find(current_end_index) != in_range_obstacle_map_.end())
  {
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_end_index])
    {
      if (Utility::GetDistanceFromPolygonToSegment(in_range_polygon, start, end) <= params_.vehicle_width)
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