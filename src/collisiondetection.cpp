#include "collisiondetection.h"

using namespace HybridAStar;

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
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
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
