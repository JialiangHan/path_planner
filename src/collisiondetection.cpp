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

// bool CollisionDetection::IsOnGrid(const Node3D *node3d_ptr) const
// {
//   return IsOnGrid(node3d_ptr->GetX(), node3d_ptr->GetY());
// }
bool CollisionDetection::IsOnGrid(const Node2D &node2d) const
{
  return IsOnGrid(node2d.GetX(), node2d.GetY());
}

// bool CollisionDetection::IsOnGrid(const Node2D *node2d_ptr) const
// {
//   return IsOnGrid(node2d_ptr->GetX(), node2d_ptr->GetY());
// }

bool CollisionDetection::IsOnGrid(const std::shared_ptr<Node2D> node2d_ptr) const
{
  return IsOnGrid(node2d_ptr->GetX(), node2d_ptr->GetY());
}

bool CollisionDetection::IsOnGrid(const std::shared_ptr<Node3D> node3d_ptr) const
{
  return IsOnGrid(node3d_ptr->GetX(), node3d_ptr->GetY());
}

bool CollisionDetection::IsOnGrid(const float &x, const float &y) const
{
  return x >= 0 && x < (int)grid_ptr_->info.width &&
         y >= 0 && y < (int)grid_ptr_->info.height;
}

bool CollisionDetection::configurationTest(float x, float y, float t) const {
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

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
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

void CollisionDetection::getConfiguration(const Node2D *node, float &x, float &y, float &t) const
{
  x = node->GetX();
  y = node->GetY();
  // avoid 2D collision checking
  t = 99;
}

void CollisionDetection::getConfiguration(const Node3D *node, float &x, float &y, float &t) const
{
  x = node->GetX();
  y = node->GetY();
  t = node->GetT();
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
