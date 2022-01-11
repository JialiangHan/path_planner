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

bool CollisionDetection::IsOnGrid(const Node3D &node3d) const
{
  return node3d.GetX() >= 0 && node3d.GetX() < grid_ptr_->info.width &&
         node3d.GetY() >= 0 && node3d.GetY() < grid_ptr_->info.height;
}

bool CollisionDetection::IsOnGrid(const Node3D *node3d_ptr) const
{
  return node3d_ptr->GetX() >= 0 && node3d_ptr->GetX() < grid_ptr_->info.width &&
         node3d_ptr->GetY() >= 0 && node3d_ptr->GetY() < grid_ptr_->info.height;
}
bool CollisionDetection::IsOnGrid(const Node2D &node2d) const
{
  return node2d.GetX() >= 0 && node2d.GetX() < (int)grid_ptr_->info.width &&
         node2d.GetY() >= 0 && node2d.GetY() < (int)grid_ptr_->info.height;
}

bool CollisionDetection::IsOnGrid(const Node2D *node2d_ptr) const
{
  return node2d_ptr->GetX() >= 0 && node2d_ptr->GetX() < (int)grid_ptr_->info.width &&
         node2d_ptr->GetY() >= 0 && node2d_ptr->GetY() < (int)grid_ptr_->info.height;
}

bool CollisionDetection::IsOnGrid(const std::shared_ptr<Node2D> node2d_ptr) const
{
  return node2d_ptr->GetX() >= 0 && node2d_ptr->GetX() < (int)grid_ptr_->info.width &&
         node2d_ptr->GetY() >= 0 && node2d_ptr->GetY() < (int)grid_ptr_->info.height;
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

void CollisionDetection::getConfiguration(const std::shared_ptr<Node2D> &node2d_ptr, float &x, float &y, float &t) const
{
  x = node2d_ptr->GetX();
  y = node2d_ptr->GetY();
  // avoid 2D collision checking
  t = 99;
}
