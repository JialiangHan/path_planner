#include "collisiondetection.h"

using namespace HybridAStar;

bool CollisionDetection::IsTraversable(const std::shared_ptr<Node2D> &node2d_ptr) const
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
  else
  {
    cost = configurationCost(x, y, t);
  }
  return cost <= 0;
};
bool CollisionDetection::IsTraversable(const std::shared_ptr<Node3D> &node3d_ptr) const
{
  if (!IsOnGrid(node3d_ptr))
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
  getConfiguration(node3d_ptr, x, y, t);
  // 2D collision test
  if (t == 99)
  {
    return !grid_ptr_->data[node3d_ptr->GetIdx()];
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

bool CollisionDetection::IsTraversable(const Node3D &node3d) const
{
  if (!IsOnGrid(node3d))
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
  getConfiguration(node3d, x, y, t);
  // 2D collision test
  if (t == 99)
  {
    return !grid_ptr_->data[node3d.GetIdx()];
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

bool CollisionDetection::configurationTest(float x, float y, float t) const
{
  const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * params_.position_resolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * params_.position_resolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / delta_heading_in_rad);
  iT = iT > 0 ? iT : 0;
  int idx = iY * params_.position_resolution * params_.headings + iX * params_.headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i)
  {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid_ptr_->info.width && cY >= 0 && (unsigned int)cY < grid_ptr_->info.height)
    {
      if (grid_ptr_->data[cY * grid_ptr_->info.width + cX])
      {
        return false;
      }
    }
  }

  return true;
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
      //get obstacle angle range
      Utility::AngleRange obstacle_angle_range = Utility::GetAngleRangeFromPointToPolygon(polygon, Utility::ConvertNod3DToVector2f(node3d));
      for (auto &angle_range : available_angle_range_vec)
      {
        //see if it is overlap with node3d steering range
        if (Utility::IsOverlap(obstacle_angle_range, angle_range))
        {
          angle_range = Utility::MinusAngleRange(angle_range, obstacle_angle_range);
          DLOG(INFO) << "two angle range are overlapped";
        }
        //obstacle angle range is larger and include the steering angle range,set steering angle range to zero
        if (Utility::IsAngleRangeInclude(obstacle_angle_range, angle_range))
        {
          angle_range.second = 0;
          DLOG(INFO) << "obstacle range is including the available angle range";
        }
        //obstacle angle range is smaller and included by the steering angle range,generate two steering angle range
        if (Utility::IsAngleRangeInclude(angle_range, obstacle_angle_range))
        {
          DLOG(INFO) << "available range is including the obstacle angle range";
          angle_range.second = obstacle_angle_range.first - angle_range.first;

          available_angle_range_vec.emplace_back(Utility::AngleRange(obstacle_angle_range.first + obstacle_angle_range.second, angle_range.first + angle_range.second - (obstacle_angle_range.first + obstacle_angle_range.second)));
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
void CollisionDetection::SetObstacleVec()
{
  for (uint x = 0; x < grid_ptr_->info.width; ++x)
  {
    for (uint y = 0; y < grid_ptr_->info.height; ++y)
    {
      if (grid_ptr_->data[y * grid_ptr_->info.width + x])
      {
        obstacle_vec_.emplace_back(Utility::CreatePolygon(Eigen::Vector2f(x, y)));
      }
    }
  }
}
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
        }
      }
      in_range_obstacle_map_.emplace(current_point_index, obstacle_vec);
    }
  }
}

uint CollisionDetection::GetNode3DIndexOnGridMap(const Node3D &node3d)
{
  return (uint)node3d.GetY() * grid_ptr_->info.width + (uint)node3d.GetX();
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
