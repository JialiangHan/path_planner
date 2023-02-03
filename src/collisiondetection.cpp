#include "collisiondetection.h"

using namespace HybridAStar;
//**********************constructor*******************
CollisionDetection::CollisionDetection(const ParameterCollisionDetection &params, costmap_2d::Costmap2D *_costmap)
{
  // LOG(INFO) << "in CollisionDetection";
  params_ = params;
  // LOG(INFO) << "vehicle length is " << params_.vehicle_length << " width is " << params_.vehicle_width;
  this->grid_ptr_ = nullptr;
  costmap_ = _costmap;
  // Lookup::collisionLookup(collisionLookup);
  resolution_ = _costmap->getResolution();
  origin_y_ = _costmap->getOriginY();
  origin_x_ = _costmap->getOriginX();
}

CollisionDetection::CollisionDetection(const ParameterCollisionDetection &params)
{
  // DLOG(INFO) << "in CollisionDetection";
  params_ = params;
  this->grid_ptr_ = nullptr;
  // Lookup::collisionLookup(collisionLookup);
}

void CollisionDetection::UpdateGrid(const nav_msgs::OccupancyGrid::Ptr &map, bool hybrid_astar)
{
  // LOG(INFO) << "UpdateGrid.";
  ros::Time t0 = ros::Time::now();
  grid_ptr_ = map;
  SetObstacleVec();

  obstacle_detection_range_ = params_.obstacle_detection_range;
  // DLOG(INFO) << "obstacle_detection_range is " << obstacle_detection_range_;
  // std::thread th2(SetInRangeObstacle, ref(obstacle_detection_range_));
  // std::thread th3(std::bind(&CollisionDetection::SetDistanceAngleRangeMap, this));

  SetInRangeObstacle();
  if (hybrid_astar)
  {
    // use only  hybrid a
    SetDistanceAngleRangeMap();
    BuildObstacleDensityMap();
    BuildNormalizedObstacleDensityMap();
  }

  // BuildCollisionLookupTable();
  // th3.join(); // 此时主线程被阻塞直至子线程执行结束。
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  LOG(INFO) << "UpdateGrid in ms: " << d * 1000;
  // LOG(INFO) << "UpdateGrid done.";
}

bool CollisionDetection::IsTraversable(const std::shared_ptr<Node2D> &node2d_ptr)
{
  // DLOG(INFO) << "IsTraversable in:";
  if (!IsOnGrid(node2d_ptr))
  {
    LOG(INFO) << "IsTraversable out.";
    return false;
  }
  float x, y, t;
  // assign values to the configuration
  getConfiguration(node2d_ptr, x, y, t);

  if (node2d_ptr->getSmartPtrPred() != nullptr)
  {
    // if (configurationTest(x, y, t))
    if (configurationTest(x, y, t) && configurationTest(node2d_ptr, node2d_ptr->getSmartPtrPred()))
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
  // DLOG(INFO) << "IsTraversable out. cost is " << cost;
  return false;
};
bool CollisionDetection::IsTraversable(const std::shared_ptr<Node3D> &node3d_ptr)
{
  return IsTraversable(*node3d_ptr);
};

bool CollisionDetection::IsTraversable(const Node3D &node3d)
{
  // DLOG_IF(INFO, (node3d.getX() > 35) && (node3d.getX() < 37) && (node3d.getY() > 12) && (node3d.getY() < 14)) << "IsTraversable in: current node is x " << node3d.getX() << " y " << node3d.getY() << " angle is " << Utility::ConvertRadToDeg(node3d.getT());
  if (!IsOnGrid(node3d))
  {
    // LOG(INFO) << "current node not on grid!";
    return false;
  }

  float x, y, t;
  // assign values to the configuration
  getConfiguration(node3d, x, y, t);
  if (node3d.getSmartPtrPred() != nullptr)
  {
    // if (configurationTest(x, y, t))
    if (configurationTest(x, y, t) && configurationTest(node3d, *node3d.getSmartPtrPred()))
    {
      // DLOG_IF(INFO, (node3d.getX() > 35) && (node3d.getX() < 37) && (node3d.getY() > 12) && (node3d.getY() < 14)) << "IsTraversable in: current node is x " << node3d.getX() << " y " << node3d.getY() << " angle is " << Utility::ConvertRadToDeg(node3d.getT());
      return true;
    }
    // LOG(INFO) << "configurationTest fail!";
  }
  else
  {
    // LOG(INFO) << "current node has not pred!";
    if (configurationTest(x, y, t))
    {
      // DLOG_IF(INFO, (node3d.getX() > 35) && (node3d.getX() < 37) && (node3d.getY() > 12) && (node3d.getY() < 14)) << "IsTraversable in: current node is x " << node3d.getX() << " y " << node3d.getY() << " angle is " << Utility::ConvertRadToDeg(node3d.getT());
      return true;
    }
    // LOG(INFO) << "configurationTest fail!";
  }
  // LOG(INFO) << "IsTraversable out.";
  return false;
}

bool CollisionDetection::IsTraversable(const Node3D &current_node, const Node3D &previous_node)
{ // DLOG(INFO) << "IsTraversable in: current node is x " << node3d.getX() << " y " << node3d.getY() << " angle is " << Utility::ConvertRadToDeg(node3d.getT());
  if (!IsOnGrid(current_node) || !IsOnGrid(previous_node))
  {
    // DLOG(INFO) << "IsTraversable out.";
    return false;
  }

  // if (configurationTest(x, y, t))
  if (configurationTest(current_node.getX(), current_node.getY(), current_node.getT()) && configurationTest(current_node, previous_node) && configurationTest(previous_node.getX(), previous_node.getY(), previous_node.getT()))
  {
    // DLOG(INFO) << "IsTraversable out.";
    return true;
  }
  // DLOG(INFO) << "IsTraversable out.";
  return false;
}

bool CollisionDetection::IsTraversable(const geometry_msgs::PoseStamped &pose)
{
  Node3D node3d;
  Utility::TypeConversion(pose, node3d);
  return IsTraversable(node3d);
}

bool CollisionDetection::IsOnGrid(const Eigen::Vector2f &point) const
{
  return IsOnGrid(point.x(), point.y());
}
bool CollisionDetection::IsOnGrid(const Node3D &node3d) const
{
  return IsOnGrid(node3d.getX(), node3d.getY());
}

bool CollisionDetection::IsOnGrid(const Node2D &node2d) const
{
  return IsOnGrid(node2d.getX(), node2d.getY());
}

bool CollisionDetection::IsOnGrid(const std::shared_ptr<Node2D> &node2d_ptr) const
{
  return IsOnGrid(node2d_ptr->getX(), node2d_ptr->getY());
}

bool CollisionDetection::IsOnGrid(const std::shared_ptr<Node3D> &node3d_ptr) const
{
  return IsOnGrid(node3d_ptr->getX(), node3d_ptr->getY());
}

bool CollisionDetection::IsOnGrid(const float &x, const float &y) const
{
  // this function is similar to costmap worldtomap
  unsigned int mx = 0, my = 0;
  if (costmap_->worldToMap(x, y, mx, my))
  {
    return true;
  }
  // LOG(WARNING) << "WARNING: current node " << x << " " << y << " is not on grid!!!";
  return false;
}
bool CollisionDetection::configurationTest(const float &x, const float &y, const float &t)
{
  // LOG(INFO) << "current node is " << x << " " << y << " " << Utility::ConvertRadToDeg(t);
  // check current point is in collision and if vehicle move to this point, if vehicle in this point is in collision?, startX,startY unit are in cell not in meter
  unsigned int startX, startY;
  if (!costmap_->worldToMap(x, y, startX, startY))
  {
    LOG(INFO) << "current node " << x << " " << y << " " << t << " is not on grid.";
    return false;
  }
  // simple collision check
  if (costmap_->getCharMap()[startX + startY * costmap_->getSizeInCellsX()] >= 250)
  {
    // LOG(INFO) << "current node " << x << " " << y << " " << t << " charmap value is " << (int)costmap_->getCharMap()[startX + startY * costmap_->getSizeInCellsX()];
    return false;
  }
  Utility::Polygon polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y), params_.vehicle_length, params_.vehicle_width, t);
  bool collision_result;
  // LOG(INFO) << "vehicle length is " << params_.vehicle_length << " width is " << params_.vehicle_width;
  collision_result = CollisionCheck(polygon);
  if (!collision_result)
  {
    // DLOG_IF(INFO, (x > 35) && (x < 37) && (y > 12) && (y < 14)) << "configurationTest in: current node is x " << x << " y " << y << " angle is " << Utility::ConvertRadToDeg(t);
  }
  return collision_result;
}
bool CollisionDetection::configurationTest(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
{
  bool collision_result;
  collision_result = CollisionCheck(start, end);
  if (!collision_result)
  {
    // LOG(INFO) << "in collision, segment start at " << start.x() << " " << start.y() << " end at " << end.x() << " " << end.y();
  }
  return collision_result;
}
bool CollisionDetection::configurationTest(const Node3D &start, const Node3D &end)
{
  Eigen::Vector2f start_vec, end_vec;
  Utility::TypeConversion(start, start_vec);
  Utility::TypeConversion(end, end_vec);
  return configurationTest(start_vec, end_vec);
}

bool CollisionDetection::configurationTest(const std::shared_ptr<Node2D> &node2d_start_ptr, const std::shared_ptr<Node2D> &node2d_end_ptr)
{
  Eigen::Vector2f start_vec, end_vec;
  Utility::TypeConversion(*node2d_start_ptr, start_vec);
  Utility::TypeConversion(*node2d_end_ptr, end_vec);
  return configurationTest(start_vec, end_vec);
}

void CollisionDetection::getConfiguration(const Node3D &node, float &x, float &y, float &t) const
{
  x = node.getX();
  y = node.getY();
  t = node.getT();
}
void CollisionDetection::getConfiguration(const std::shared_ptr<Node2D> &node2d_ptr, float &x, float &y, float &t) const
{
  x = node2d_ptr->getX();
  y = node2d_ptr->getY();
  // avoid 2D collision checking
  t = 99;
}
void CollisionDetection::getConfiguration(const std::shared_ptr<Node3D> &node3d_ptr, float &x, float &y, float &t) const
{
  x = node3d_ptr->getX();
  y = node3d_ptr->getY();
  t = node3d_ptr->getT();
}

// checked
void CollisionDetection::SetObstacleVec()
{
  // LOG(INFO) << "SetObstacleVec in:";
  ros::Time t0 = ros::Time::now();
  Utility::Polygon current_polygon;
  // unit for x_limit,y_limit,x,y are meter
  float x_limit = origin_x_ + grid_ptr_->info.width * resolution_;
  float y_limit = origin_y_ + grid_ptr_->info.height * resolution_;
  for (float x = origin_x_; x < x_limit;)
  {
    for (float y = origin_y_; y < y_limit;)
    {
      if (grid_ptr_->data[GetNode3DIndexOnGridMap(x, y)])
      {
        // polygon is a width=height=resolution rectangle
        current_polygon = Utility::CreatePolygon(Eigen::Vector2f(x, y), resolution_, resolution_);
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
        // LOG(INFO) << "obstacle origin is " << x << " " << y;
      }
      else
      {
        // LOG(INFO) << "free";
      }
      y = y + resolution_;
    }
    x = x + resolution_;
  }
  if (params_.map_boundary_obstacle)
  {
    AddMapBoundaryAsObstacle(obstacle_vec_);
  }
  // for (const auto &polygon : obstacle_vec_)
  // {
  //   DLOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y();
  // }
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  LOG(INFO) << "SetObstacleVec in ms: " << d * 1000;
  // LOG(INFO) << "SetObstacleVec out.";
}
// checked
void CollisionDetection::SetInRangeObstacle()
{
  // LOG(INFO) << "SetInRangeObstacle in:";
  ros::Time t0 = ros::Time::now();
  std::vector<Utility::Polygon> obstacle_vec;
  float x_limit = origin_x_ + grid_ptr_->info.width * resolution_;
  float y_limit = origin_y_ + grid_ptr_->info.height * resolution_;
  float distance = 0;
  for (float x = origin_x_; x < x_limit;)
  {
    for (float y = origin_y_; y < y_limit;)
    {
      // LOG(INFO) << "current node is " << x << " " << y;
      uint current_point_index = GetNode3DIndexOnGridMap(x, y);
      Eigen::Vector2f current_point(x, y);
      obstacle_vec.clear();
      for (const auto &polygon : obstacle_vec_)
      {
        distance = Utility::GetDistanceFromPolygonToPoint(polygon, current_point);
        if (distance < obstacle_detection_range_)
        {
          obstacle_vec.emplace_back(polygon);
          // LOG(INFO) << "obstacle first point is " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << "  and its in the range. current point is " << x << " " << y << " distance is " << distance;
        }
      }
      // LOG(INFO) << "current node is " << x << " " << y << " index is " << current_point_index;
      in_range_obstacle_map_.emplace(current_point_index, obstacle_vec);
      y = y + resolution_;
    }
    x = x + resolution_;
  }
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  LOG(INFO) << "SetInRangeObstacle in ms: " << d * 1000;
  // LOG(INFO) << "SetInRangeObstacle out.";
}
// checked,it`s correct.
uint CollisionDetection::GetNode3DIndexOnGridMap(const Node3D &node3d)
{

  return GetNode3DIndexOnGridMap(node3d.getX(), node3d.getY());
}
// checked,it`s correct.
uint CollisionDetection::GetNode3DIndexOnGridMap(const float &x, const float &y)
{
  uint index = std::floor(std::round(((y - origin_y_) / resolution_) * 1000) / 1000) * grid_ptr_->info.width + std::floor(std::round(((x - origin_x_) / resolution_) * 1000) / 1000);
  // LOG(INFO) << "coordinate is " << x << " " << y << " "
  // << " final index is " << index << " origin x is " << origin_x_ << " origin y is " << origin_y_ << " resolution is " << resolution_ << " map width is " << grid_ptr_->info.width;
  // LOG(INFO) << "idx is " << index << " width is " << grid_ptr_->info.width << " resolution is " << resolution_ << " origin x is " << origin_x_ << " origin y is " << origin_y_ << " x is " << x << " y is " << y << " std::floor(std::round(((y - origin_y) / resolution) * 1000) / 1000)   is " << std::floor(std::round(((y - origin_y_) / resolution_) * 1000) / 1000) << " std::floor(std::round(((x - origin_x) / resolution) * 1000) / 1000) is " << std::floor(std::round(((x - origin_x_) / resolution_) * 1000) / 1000) << " ((y - origin_y) / resolution)  is " << std::fixed << std::setprecision(7) << ((y - origin_y_) / resolution_) << " ((x - origin_x) / resolution) is " << std::fixed << std::setprecision(7) << ((x - origin_x_) / resolution_);
  return index;
}
// checked, it`s correct.
Utility::AngleRange CollisionDetection::GetNode3DAvailableSteeringAngleRange(const Node3D &node3d)
{
  // DLOG(INFO) << "GetNode3DAvailableSteeringAngleRange in:";
  Utility::AngleRange out;
  float current_heading = Utility::RadNormalization(node3d.getT());
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
  // LOG(INFO) << "SetDistanceAngleRangeMap in:";
  ros::Time t0 = ros::Time::now();
  std::vector<std::pair<float, Utility::AngleRange>> distance_angle_range_vec;
  Node3D node_3d;
  float x_limit = origin_x_ + grid_ptr_->info.width * resolution_;
  float y_limit = origin_y_ + grid_ptr_->info.height * resolution_;
  for (float x = origin_x_; x < x_limit;)
  {
    for (float y = origin_y_; y < y_limit;)
    {
      // LOG(INFO) << "current node is " << x << " " << y;
      distance_angle_range_vec.clear();

      uint current_point_index = GetNode3DIndexOnGridMap(x, y);
      Eigen::Vector2f current_point(x, y);
      Utility::TypeConversion(current_point, node_3d);
      std::vector<std::pair<float, float>> angle_distance_vec = SweepDistanceAndAngle(node_3d, obstacle_detection_range_, false);
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
            LOG_IF(WARNING, (distance_vec.size() - Utility::ConvertRadToDeg(range_range)) > 0.1) << "WARNING: distance vec size is not correct!!! current size is " << distance_vec.size() << " range range is " << Utility::ConvertRadToDeg(range_range);
            min_distance = *std::min_element(distance_vec.begin(), distance_vec.end());
            // LOG_IF(WARNING, min_distance == 0) << "WARNING: min_distance is zero!!! ";
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
          // DLOG_IF(INFO, pair.second == 0) << "insert current distance to distance vec: " << pair.second;
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
          // DLOG(INFO) << "in last angle distance vec";

          // check if size of distance vec is zero or not, if zero, just push back current distance
          if (distance_vec.size() == 0)
          {
            distance_vec.emplace_back(pair.second);
            // DLOG_IF(INFO, pair.second == 0) << "insert current distance to distance vec: " << pair.second;
          }
          if (free_angle_range_flag)
          {
            min_distance = obstacle_detection_range_;
            // DLOG(INFO) << "current angle range is free angle range, set min distance to obstacle detection range.";
          }
          else
          {
            min_distance = *std::min_element(distance_vec.begin(), distance_vec.end());
            // DLOG(INFO) << "current angle range is obstacle angle range, set min distance to min of distance vec.";
          }
          range_range = pair.first - range_start;
          if (range_start != -1 && range_range != -1 && range_range != 0)
          {
            distance_angle_range_vec.emplace_back(std::pair<float, Utility::AngleRange>(min_distance, Utility::AngleRange(range_start, range_range)));
          }
        }
      }
      distance_angle_range_map_.emplace(current_point_index, distance_angle_range_vec);
      DLOG_IF(WARNING, distance_angle_range_vec.size() == 0) << "WARNING: size of distance angle range vec is zero!!!!";
      y = y + resolution_;
    }
    x = x + resolution_;
  }
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  LOG(INFO) << "SetDistanceAngleRangeMap in ms: " << d * 1000;
  // LOG(INFO) << "SetDistanceAngleRangeMap out.";
}
// Not used
//  where to put this collision table in lookup_table.cpp or collisiondetection..cpp?
void CollisionDetection::BuildCollisionLookupTable()
{
  // DLOG(INFO) << "BuildCollisionLookupTable start:";
  collision_lookup_.clear();
  // 0. three for loop to loop every point in the map, similar to lookup_table.cpp
  uint index, fine_index;
  int x_count, y_count;
  bool collision_result;
  std::unordered_map<uint, bool> collision_result_map;
  float x = origin_x_, y = origin_y_, theta = 0;

  const float delta_heading_in_rad = 2 * M_PI / (float)params_.headings;
  while (x < grid_ptr_->info.width * resolution_)
  {
    y = origin_y_;
    while (y < grid_ptr_->info.height * resolution_)
    {
      index = GetNode3DIndexOnGridMap(x, y);
      x_count = 0;
      while (x_count < params_.position_resolution)
      {
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
            if (((x - origin_x_) <= params_.vehicle_length / 2 && (y - origin_y_) <= params_.vehicle_width / 2) ||
                (x >= (grid_ptr_->info.width * resolution_ - params_.vehicle_length / 2) && (y - origin_y_) <= params_.vehicle_width / 2) ||
                ((x - origin_x_) <= params_.vehicle_length / 2 && y >= (grid_ptr_->info.height * resolution_ - params_.vehicle_width / 2)) ||
                (x >= (grid_ptr_->info.width * resolution_ - params_.vehicle_length / 2) && y >= (grid_ptr_->info.height * resolution_ - params_.vehicle_width / 2)))
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
          y += (1.0f / params_.position_resolution) * resolution_;
          y_count++;
        }
        y = y - resolution_;
        x_count++;
        x += (1.0f / params_.position_resolution) * resolution_;
      }
      x = x - resolution_;

      collision_lookup_.emplace(index, collision_result_map);
      y = y + resolution_;
    }
    x = x + resolution_;
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
      // LOG(INFO) << "polygon is not on grid: point is " << point.x() << " " << point.y() << " Polygon: " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y();
      return false;
    }
  }
  // should be convert to int for x and y;
  uint current_point_index = GetNode3DIndexOnGridMap(polygon[0].x(), polygon[0].y());

  // check with in range obstacle
  if (in_range_obstacle_map_.find(current_point_index) != in_range_obstacle_map_.end())
  {
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_point_index])
    {
      if (Utility::IsPolygonIntersectWithPolygon(polygon, in_range_polygon))
      {
        // LOG(INFO) << "intersect with obstacle. Polygon: " << polygon[0].x() << " " << polygon[0].y() << " second point is " << polygon[1].x() << " " << polygon[1].y() << " third point is " << polygon[2].x() << " " << polygon[2].y() << " fourth point is " << polygon[3].x() << " " << polygon[3].y() << " in range polygon is " << in_range_polygon[0].x() << " " << in_range_polygon[0].y() << " second point is " << in_range_polygon[1].x() << " " << in_range_polygon[1].y() << " third point is " << in_range_polygon[2].x() << " " << in_range_polygon[2].y() << " fourth point is " << in_range_polygon[3].x() << " " << in_range_polygon[3].y();
        return false;
      }
    }
    // DLOG(INFO) << " not intersect.";
  }
  else
  {
    LOG(INFO) << "current polygon can`t be found in in_range_obstacle_map.";
  }
  return true;
  // 2. find obstacle in a certain range, this range is determined be vehicle parameter
}

bool CollisionDetection::CollisionCheck(const Eigen::Vector2f &start, const Eigen::Vector2f &end)
{
  // DLOG(INFO) << "CollisionCheck in.";
  if (!IsOnGrid(start) || !IsOnGrid(end))
  {
    LOG(INFO) << "start or end is not on grid";
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
    LOG(INFO) << "current start " << start.x() << " " << start.y() << " index is " << current_start_index << " can`t be found in in_range_obstacle_map.";
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
    LOG(INFO) << "current end " << end.x() << " " << end.y() << " index is " << current_end_index << " can`t be found in in_range_obstacle_map.";
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

  x_int = std::floor(std::round((x / resolution_) * 10000) / 10000) * resolution_;
  y_int = std::floor(std::round((y / resolution_) * 10000) / 10000) * resolution_;
  t_index = std::floor(std::round((angle / delta_heading_in_rad) * 10000) / 10000);
  x_index = (x - x_int) * params_.position_resolution;
  y_index = (y - y_int) * params_.position_resolution;
  out = t_index * params_.position_resolution * params_.position_resolution + y_index * params_.position_resolution + x_index;
  // DLOG(INFO) << "coordinate is " << x << " " << y << " " << t << " x_int is " << x_int << " y_int is " << y_int << " x_index is " << x_index << " y_index is " << y_index << " t_index is " << t_index << " final index is " << out;

  return out;
}

std::vector<std::pair<float, Utility::AngleRange>> CollisionDetection::FindFreeAngleRangeAndObstacleAngleRange(const Node3D &node3d)
{
  std::vector<std::pair<float, Utility::AngleRange>> out;
  // 1. get steering angle range
  Utility::AngleRange steering_angle_range(0, Utility::ConvertDegToRad(359.9999));

  // LOG(INFO) << "current node is " << node3d.getX() << " " << node3d.getY() << " " << Utility::ConvertRadToDeg(node3d.getT()) << " . steering angle range: start from " << Utility::ConvertRadToDeg(steering_angle_range.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(steering_angle_range));
  uint current_point_index = GetNode3DIndexOnGridMap(node3d);
  // 2. find common range of free/obstacle angle range and steering angle range
  if (distance_angle_range_map_.find(current_point_index) != distance_angle_range_map_.end())
  {
    for (const auto &pair : distance_angle_range_map_[current_point_index])
    {
      Utility::AngleRange common = Utility::FindCommonAngleRange(pair.second, steering_angle_range);
      if (common.first != -1 && common.second != -1)
      {
        // LOG(INFO) << "inserting: current obstacle angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " steering angle range is " << Utility::ConvertRadToDeg(steering_angle_range.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(steering_angle_range)) << " common angle range is " << Utility::ConvertRadToDeg(common.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(common));
        out.emplace_back(pair.first, common);
        // DLOG_IF(INFO, pair.first == 0) << "inserting: current min distance to obstacle is " << pair.first << " current obstacle angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " steering angle range is " << Utility::ConvertRadToDeg(steering_angle_range.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(steering_angle_range));
      }
    }
  }
  else
  {
    // LOG(WARNING) << "WARNING: current node index can`t be found in distance_angle_range_map_";
  }

  // DLOG_IF(WARNING, !Utility::IsEqual(steering_angle_range, total_ar)) << "WARNING: total ar start from " << Utility::ConvertRadToDeg(total_ar.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(total_ar)) << " is not equal to steering angle range start from " << Utility::ConvertRadToDeg(steering_angle_range.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(steering_angle_range)) << ", Something wrong!!!";

  // DLOG_IF(WARNING, out.size() == 0) << "WARNING: Free angle range size is zero!!!! current node is " << node3d.getX() << " " << node3d.getY() << " " << Utility::ConvertRadToDeg(node3d.getT()) << " . steering angle range: start from " << Utility::ConvertRadToDeg(steering_angle_range.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(steering_angle_range));

  // for (const auto element : out)
  // {
  // LOG(INFO) << "element min distance to obstacle is " << element.first << " angle range: start from " << Utility::ConvertRadToDeg(element.second.first) << " end is " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(element.second));
  // }
  // DLOG(INFO) << "FindFreeAngleRangeAndObstacleAngleRange out.";
  return out;
}

// checked, it`s correct for free angle range.
std::vector<std::pair<float, float>> CollisionDetection::SelectStepSizeAndSteeringAngle(const std::vector<std::pair<float, Utility::AngleRange>> &available_angle_range_vec, const Node3D &pred, const Node3D &goal, const int &number_of_successor, const float &fixed_step_size, const float &distance_start_to_goal)
{
  std::vector<std::pair<float, float>> out;
  float steering_angle, distance_to_goal = Utility::GetDistance(pred, goal);

  // 1. find angle to goal
  float angle_to_goal, orientation_diff, steering_angle_toward_goal;
  // final_orientation;
  if (params_.add_one_more_successor)
  {
    float weighting = distance_to_goal / distance_start_to_goal;
    angle_to_goal = Utility::GetAngle(pred, goal);
    orientation_diff = -Utility::RadNormalization(pred.getT() - goal.getT());
    steering_angle_toward_goal = -Utility::RadNormalization(weighting * angle_to_goal + (1 - weighting) * orientation_diff);
    // final_orientation = Utility::RadNormalization(pred.getT() + steering_angle_toward_goal);
    // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(Utility::RadNormalization(Utility::GetAngle(pred, goal_)));
    // DLOG(INFO) << "steering angle is " << Utility::ConvertRadToDeg(angle_to_goal);
  }
  // DLOG(INFO) << "size of available_angle_range_vec is " << available_angle_range_vec.size();
  for (const auto &pair : available_angle_range_vec)
  {
    // LOG(INFO) << "current pair second first is " << Utility::ConvertRadToDeg(pair.second.first) << " range is " << Utility::ConvertRadToDeg(pair.second.second);
    // if steering angle range is zero, that means current node is blocked by obstacles
    if (pair.second.second == 0)
    {
      // LOG(INFO) << "for current node, all available steering range is blocked by obstacle";
      steering_angle = Utility::RadNormalization((pair.second.first - pred.getT()));
      steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
      std::pair<float, float> temp(pair.first, steering_angle);
      if (!Utility::DuplicateCheck(out, temp, false))
      {
        out.emplace_back(temp);
      }
      continue;
    }
    // 2. for free angle range, steering angle is angle range start + 1/(number of successors+1) * angle range.
    if (pair.first == obstacle_detection_range_)
    {
      // LOG(INFO) << "in free angle range.";
      if (params_.add_one_more_successor)
      {
        // LOG(INFO) << "one more node is in free angle range!";
        std::pair<float, float> temp = AddOneMoreStepSizeAndSteeringAngle(steering_angle_toward_goal, FindStepSize(pred, steering_angle_toward_goal, goal, fixed_step_size), pred, goal);
        if (!Utility::DuplicateCheck(out, temp, false))
        {
          out.emplace_back(temp);
          // LOG_IF(INFO, temp.first < 1) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " step size is " << temp.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(temp.second);
          // DLOG_IF(INFO, distance_to_goal < 2) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " step size is " << temp.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(temp.second);
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
        if (true)
        {
          // LOG(INFO) << "free angle range is too small!!";
          steering_angle = Utility::RadNormalization((pair.second.first + 0.5 * pair.second.second - pred.getT()));
          steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
          std::pair<float, float> temp(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
          if (!Utility::DuplicateCheck(out, temp, false))
          {
            // DLOG(INFO) << "steering angle for free angle range is " << Utility::ConvertRadToDeg(steering_angle);
            out.emplace_back(temp);
            // LOG(INFO) << "for free angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " current orientation is " << Utility::ConvertRadToDeg(pred.getT());
            // DLOG_IF(INFO, (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < -Utility::ConvertDegToRad(30))) << "for free angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " current orientation is " << Utility::ConvertRadToDeg(pred.getT());
          }
          else
          {
            // LOG(INFO) << "duplicated!!";
          }
        }
        else
        {
          if (pair.second.second / (number_of_successor + 1) > Utility::ConvertDegToRad(1))
          {
            for (int index = 1; index < number_of_successor + 1; ++index)
            {
              steering_angle = Utility::RadNormalization((pair.second.first + index * pair.second.second / (number_of_successor + 1) - pred.getT()));
              steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
              std::pair<float, float> temp(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
              if (!Utility::DuplicateCheck(out, temp, false))
              {
                // DLOG(INFO) << "steering angle for free angle range is " << Utility::ConvertRadToDeg(steering_angle);
                out.emplace_back(temp);
                DLOG_IF(INFO, (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < -Utility::ConvertDegToRad(30))) << "for free angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " current orientation is " << Utility::ConvertRadToDeg(pred.getT());
              }
            }
          }
          else
          {
            // LOG(INFO) << "free angle range is too small!!";
            steering_angle = Utility::RadNormalization((pair.second.first + 0.5 * pair.second.second - pred.getT()));
            steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
            std::pair<float, float> temp(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
            if (!Utility::DuplicateCheck(out, temp, false))
            {
              // DLOG(INFO) << "steering angle for free angle range is " << Utility::ConvertRadToDeg(steering_angle);
              out.emplace_back(temp);
              DLOG_IF(INFO, (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < -Utility::ConvertDegToRad(30))) << "for free angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " current orientation is " << Utility::ConvertRadToDeg(pred.getT());
            }
          }
        }
      }
      else
      {
        if (pair.second.second < Utility::ConvertDegToRad(3))
        {
          // LOG(INFO) << "create successor for free angle range smaller than 3deg.";
          steering_angle = Utility::RadNormalization((pair.second.first + pair.second.second * 0.5 - pred.getT()));
          steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
          std::pair<float, float> temp(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
          if (!Utility::DuplicateCheck(out, temp, false))
          {
            out.emplace_back(temp);
          }
        }
        else
        {
          // LOG(INFO) << "create successor for free angle range larger than 3deg.";
          float current_angle = pair.second.first;
          float steering_angle_step_size = Utility::ConvertDegToRad(5);
          while (current_angle <= Utility::GetAngleRangeEnd(pair.second))
          {
            steering_angle = Utility::RadNormalization((current_angle - pred.getT()));
            steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
            std::pair<float, float> temp(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
            if (!Utility::DuplicateCheck(out, temp, false))
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
          // DLOG(INFO) << "one more node is in obstacle angle range!";
          std::pair<float, float> temp = AddOneMoreStepSizeAndSteeringAngle(steering_angle_toward_goal, FindStepSize(pred, steering_angle_toward_goal, goal, fixed_step_size), pred, goal);
          if (!Utility::DuplicateCheck(out, temp, false))
          {
            out.emplace_back(temp);
            // LOG(INFO) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " step size is " << temp.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(temp.second);
            // DLOG_IF(INFO, distance_to_goal < 2) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " step size is " << temp.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(temp.second);
          }
        }
      }
      // how to select steering angle in obstacle range: just select range start or end is not a good idea. if there is free angle range in steering angle range, then do not do steering in obstacle angle range, if there is no free angle range in steering angle range, steering toward free angle range which is closest to steering angle.
      if (params_.steering_angle_towards_free_angle_range_for_obstacle_angle_range)
      {
        // find angle in obstacle angle range which is closest to free angle range
        float closest_angle = IsCloseToFreeAngleRange(available_angle_range_vec, pred, pair.second.first, Utility::GetAngleRangeEnd(pair.second));
        steering_angle = Utility::RadNormalization((closest_angle - pred.getT()));
        steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
        std::pair<float, float> temp(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
        if (!Utility::DuplicateCheck(out, temp, false))
        {
          // DLOG(INFO) << "steering angle for obstacle angle range is " << Utility::ConvertRadToDeg(steering_angle);
          out.emplace_back(temp);
          // LOG(INFO) << "for obstacle angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " . closest angle is " << Utility::ConvertRadToDeg(closest_angle) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " current orientation is " << Utility::ConvertRadToDeg(pred.getT());
          // DLOG_IF(INFO, (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < -Utility::ConvertDegToRad(30))) << "for obstacle angle range start from " << Utility::ConvertRadToDeg(pair.second.first) << " end at " << Utility::ConvertRadToDeg(Utility::GetAngleRangeEnd(pair.second)) << " . closest angle is " << Utility::ConvertRadToDeg(closest_angle) << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " current orientation is " << Utility::ConvertRadToDeg(pred.getT());
        }
        else
        {
          // LOG(INFO) << "duplicated!!";
        }
      }
      else
      {
        // LOG(INFO) << "mark.";
        //  steering angle is range start
        steering_angle = Utility::RadNormalization((pair.second.first - pred.getT()));
        steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
        std::pair<float, float> temp(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
        if (!Utility::DuplicateCheck(out, temp, false))
        {
          out.emplace_back(temp);
        }
        // steering angle is range end
        steering_angle = Utility::RadNormalization((pair.second.first + pair.second.second - pred.getT()));
        // steering_angle = LimitSteeringAngle(steering_angle, Utility::ConvertDegToRad(30));
        std::pair<float, float> temp1(FindStepSize(pred, steering_angle, goal, fixed_step_size), steering_angle);
        if (!Utility::DuplicateCheck(out, temp1, false))
        {
          out.emplace_back(temp1);
        }
        else
        {
          // LOG(INFO) << "duplicated!!";
        }
      }
    }
  }

  // for (const auto &pair : out)
  // {
  //   DLOG_IF(INFO, (pair.first < 1) || (pair.second > Utility::ConvertDegToRad(30)) || (pair.second < -Utility::ConvertDegToRad(30))) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " step size " << pair.first << " steering angle is " << Utility::ConvertRadToDeg(pair.second);
  // }
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

    distance = FindNoCollisionDistance(node3d, radius, angle);

    out.emplace_back(std::pair<float, float>(angle, distance));
    // DLOG(INFO) << "angle is " << Utility::ConvertRadToDeg(angle) << " distance is " << distance;
    // DLOG_IF(INFO, distance == 0) << "angle is " << Utility::ConvertRadToDeg(angle) << " distance is " << distance;
  }
  // DLOG_IF(INFO, distance == 0) << "angle is " << Utility::ConvertRadToDeg(angle) << " distance is " << distance;
  // DLOG(INFO) << "SweepDistanceAndAngle out;";
  return out;
}

// this function seems ok for both no obstacle and obstacle , fully checked
float CollisionDetection::FindNoCollisionDistance(const Node3D &node3d, const float &radius, const float &angle)
{
  // DLOG(INFO) << "FindNoCollisionDistance in:";
  // DLOG(INFO) << "node3d is " << node3d.getX() << " " << node3d.getY() << " " << node3d.getT() << " radius is " << radius << " angle is " << Utility::ConvertRadToDeg(angle);
  // set out to value of radius, if no obstacle in the angle direction, then return radius;
  float out = radius;
  // DLOG_IF(INFO, out == 0) << "distance is " << out;
  // 1. calculate node3d index
  uint current_index = GetNode3DIndexOnGridMap(node3d);
  Eigen::Vector2f node3d_vec;
  Utility::TypeConversion(node3d, node3d_vec);
  // 2. based on this index, find in range obstacles
  if (in_range_obstacle_map_.find(current_index) != in_range_obstacle_map_.end())
  {
    // 4. in this angle direction, check if there are some obstacles
    // 5. get distance from node3d to nearest obstacle in this direction
    for (const auto &in_range_polygon : in_range_obstacle_map_[current_index])
    {
      float temp = Utility::GetDistanceFromPolygonToPointAtAngle(in_range_polygon, node3d_vec, angle);
      // DLOG(INFO) << "temp is " << temp;
      if (temp < 0)
      {
        // DLOG(INFO) << "current in range polygon does not intersect with segment start at node3d along angle direction.";
        continue;
      }
      if (out > temp)
      {
        out = temp;
        // DLOG_IF(INFO, out == 0) << "distance is " << out;
        // DLOG(INFO) << "out is " << out;
      }
    }
    // DLOG(INFO) << "for in range obstacle of node, not intersect.";
  }

  // DLOG(INFO) << "no collision distance is " << out;
  // DLOG(INFO) << "FindNoCollisionDistance out.";
  return out;
}

void CollisionDetection::BuildObstacleDensityMap()
{
  // LOG(INFO) << "BuildObstacleDensityMap in:";
  ros::Time t0 = ros::Time::now();
  // build obstacle density map according to in range obstacle map, seems not correct, need to consider obstacle in a wider range.
  // so use the same way to build this map as in range obstacle map
  // can this two function be merged?,no, the in range obstacle map consider 2d index while this map consider 3d index.
  // definition of obstacle density:currently obstacle density is the number of obstacle in steering angle range and radius(parameter range)

  uint number_of_obstacles = 0;
  uint current_point_index;
  std::vector<Utility::Polygon> obstacle_vec;
  // this angle is the max steering angle for vehicle, unit is deg.
  float max_steering_angle = 30;
  float x_limit = origin_x_ + grid_ptr_->info.width * resolution_;
  float y_limit = origin_y_ + grid_ptr_->info.height * resolution_;
  for (float x = origin_x_; x < x_limit;)
  {
    for (float y = origin_y_; y < y_limit;)
    {
      // definition of obstacle density:currently obstacle density is the number of obstacle in radius(parameter range), do not consider current orientation, note: index is not fine index for this case
      if (!params_.consider_steering_angle_range_for_obstacle_density)
      {
        uint current_point_index = GetNode3DIndexOnGridMap(x, y);
        Eigen::Vector2f current_point(x, y);
        obstacle_vec.clear();
        if (in_range_obstacle_map_.find(current_point_index) != in_range_obstacle_map_.end())
        {
          in_range_obstacle_density_map_.emplace(current_point_index, in_range_obstacle_map_[current_point_index].size());
        }
        else
        {
          LOG(WARNING) << "current index can not be find in in_range_obstacle_density_map_!!";
        }

        // DLOG(INFO) << "current node is " << x << " " << y << " " << theta << " fine index is " << current_point_index << " number of obstacles is " << number_of_obstacles;
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
          float current_angle_range_start = Utility::ConvertDegToRad(theta - max_steering_angle);
          float current_angle_range_range = Utility::ConvertDegToRad(2 * max_steering_angle);
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
              if (distance < obstacle_detection_range_)
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
      y = y + resolution_;
    }
    x = x + resolution_;
  }
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  LOG(INFO) << "BuildObstacleDensityMap in ms: " << d * 1000;
  // LOG(INFO) << "BuildObstacleDensityMap out.";
}

uint CollisionDetection::GetNode3DFineIndex(const Node3D &node3d)
{
  return CalculateFineIndex(node3d.getX(), node3d.getY(), node3d.getT());
}

void CollisionDetection::BuildNormalizedObstacleDensityMap()
{
  // LOG(INFO) << "BuildNormalizedObstacleDensityMap in:";
  ros::Time t0 = ros::Time::now();
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
    // LOG(INFO) << "current fine index is " << element.first << " normalized obstacle density is " << normalized_obstacle_density << " sum is " << sum_normalized_obstacle_density;
    // DLOG(INFO) << "current fine index is " << element.first << " normalized obstacle density is " << normalized_obstacle_density;
  }
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  LOG(INFO) << "BuildNormalizedObstacleDensityMap in ms: " << d * 1000;
  // LOG(INFO) << "BuildNormalizedObstacleDensityMap out.";
}

float CollisionDetection::GetNormalizedObstacleDensity(const Node3D &node3d)
{
  // DLOG(INFO) << "GetNormalizedObstacleDensity in:";
  float obstacle_density = 0;
  uint current_index = 0;
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
  x_int = (uint)(x - origin_x_) / resolution_;
  y_int = (uint)(y - origin_y_) / resolution_;
  t_index = (uint)(angle / delta_heading_in_rad);

  out = t_index * grid_ptr_->info.width * grid_ptr_->info.height + y_int * grid_ptr_->info.width + x_int;
  // DLOG(INFO) << "coordinate is " << x << " " << y << " " << Utility::ConvertRadToDeg(theta) << " x_int is " << x_int << " y_int is " << y_int << " t_index is " << t_index << " final index is " << out;
  return out;
}

uint CollisionDetection::Get3DIndexOnGridMap(const Node3D &node3d)
{
  return Get3DIndexOnGridMap(node3d.getX(), node3d.getY(), node3d.getT());
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
  DLOG_IF(INFO, distance_to_goal < 4) << "step size is " << step_size << " distance to goal is " << distance_to_goal << " new step size is " << new_step_size;
  // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(angle_to_goal);
  // 4. if angle to goal is in the steering angle range(current orientation +-30deg), then make it steering angle, otherwise 30 or -30 to make angle to goal smaller
  steering_angle = LimitSteeringAngle(angle_to_goal, Utility::ConvertDegToRad(30));

  out.first = new_step_size;
  out.second = steering_angle;

  // DLOG(INFO) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " and goal orientation is " << Utility::ConvertRadToDeg(goal.getT()) << " one more step size is " << new_step_size << " and steering angle pair is " << Utility::ConvertRadToDeg(steering_angle) << "distance to goal is " << distance_to_goal;
  // DLOG_IF(INFO, (new_step_size < 4) || (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < -Utility::ConvertDegToRad(30))) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " and goal orientation is " << Utility::ConvertRadToDeg(goal.getT()) << " one more step size is " << new_step_size << " and steering angle pair is " << Utility::ConvertRadToDeg(steering_angle);
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
    // DLOG(INFO) << "for node " << current.getX() << " " << current.getY() << " " << current.getT() << " angle2: " << Utility::ConvertRadToDeg(angle2) << " is closest to free angle range";
    return angle2;
  }
  else
  {
    // DLOG(INFO) << "for node " << current.getX() << " " << current.getY() << " " << current.getT() << " angle1: " << Utility::ConvertRadToDeg(angle1) << " is closest to free angle range";
    return angle1;
  }
}

float CollisionDetection::LimitSteeringAngle(const float &steering_angle, const float &steering_angle_limit)
{
  float limited_steering_angle;
  if (steering_angle > steering_angle_limit)
  {
    limited_steering_angle = steering_angle_limit;
  }
  else if (steering_angle < -steering_angle_limit)
  {
    limited_steering_angle = -steering_angle_limit;
  }
  else
  {
    // round need to be in degree not rad
    limited_steering_angle = Utility::ConvertDegToRad(std::round(Utility::ConvertRadToDeg(steering_angle) * 10) / 10);
  }
  return limited_steering_angle;
}

std::vector<std::pair<float, float>> CollisionDetection::FindStepSizeAndSteeringAngle(const Node3D &pred, const Node3D &start, const Node3D &goal, const int &number_of_successor, const float &step_size)
{
  // DLOG(INFO) << "FindStepSizeAndSteeringAngle in:";
  std::vector<std::pair<float, float>> out;
  // 1. find steering angle range for current node according to vehicle structure, this step has been done in function at step 2.
  // 2. in steering angle range, find its corresponding distance to obstacle and its angle range
  float distance_start_to_goal = Utility::GetDistance(start, goal);
  std::vector<std::pair<float, Utility::AngleRange>> available_angle_range_vec = FindFreeAngleRangeAndObstacleAngleRange(pred);
  // 3. determine step size and steering angle from previous output
  out = SelectStepSizeAndSteeringAngle(available_angle_range_vec, pred, goal, number_of_successor, step_size, distance_start_to_goal);

  for (const auto &pair : out)
  {
    DLOG_IF(INFO, (pair.first < 1) || (pair.second > Utility::ConvertDegToRad(30)) || (pair.second < -Utility::ConvertDegToRad(30))) << "step size " << pair.first << " steering angle is " << Utility::ConvertRadToDeg(pair.second);
    // LOG(INFO) << "step size " << pair.first << " steering angle is " << Utility::ConvertRadToDeg(pair.second);
  }
  // LOG(INFO) << "FindStepSizeAndSteeringAngle out.";
  return out;
}

float CollisionDetection::FindStepSize(const Node3D &pred, const float &steering_angle, const Node3D &goal, const float &fixed_step_size)
{
  // float step_size = 0, available_step_size = 0, min_distance_to_obstacle;
  float step_size = 0, min_distance_to_obstacle = obstacle_detection_range_;
  float distance_to_goal = Utility::GetDistance(pred, goal);
  std::vector<std::pair<float, Utility::AngleRange>> step_size_angle_range_vec = FindFreeAngleRangeAndObstacleAngleRange(pred);
  float final_orientation = Utility::RadNormalization(pred.getT() + steering_angle);
  for (const auto &element : step_size_angle_range_vec)
  {
    if (Utility::IsAngleRangeInclude(element.second, final_orientation))
    {
      min_distance_to_obstacle = element.first;
      break;
    }
  }
  float weight_step_size = GetStepSizeWeight(GetNormalizedObstacleDensity(pred));

  // available_step_size = ((min_distance_to_obstacle - 0.5 * params_.vehicle_length) > fixed_step_size) ? (min_distance_to_obstacle - 0.5 * params_.vehicle_length) : 0;

  step_size = weight_step_size * min_distance_to_obstacle;

  // make sure step size is larger than a predefined min distance otherwise too many successors
  if (step_size < fixed_step_size)
  {
    // DLOG(INFO) << "step size is smaller than  predefined min distance, make it to one!!";
    step_size = fixed_step_size;
  }
  // make sure step size is larger than a predefined min distance otherwise too many successors

  if (step_size > distance_to_goal)
  {
    step_size = distance_to_goal;
  }

  return step_size;
}