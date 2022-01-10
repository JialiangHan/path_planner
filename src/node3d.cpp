#include "node3d.h"
using namespace HybridAStar;

// CONSTANT VALUES
// possible directions
// const int Node3D::dir = 3;
// possible movements
//const float Node3D::dy[] = { 0,        -0.032869,  0.032869};
//const float Node3D::dx[] = { 0.62832,   0.62717,   0.62717};
//const float Node3D::dt[] = { 0,         0.10472,   -0.10472};

// R = 6, 6.75 DEG
const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

//###################################################
//                                      MOVEMENT COST
//###################################################
void Node3D::updateG(const float &weight_turning, const float &weight_change_of_direction, const float &weight_reverse)
{
  // forward driving
  if (prim < 3) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > 2) {
        g += dx[0] * weight_turning * weight_change_of_direction;
      } else {
        g += dx[0] * weight_turning;
      }
    } else {
      g += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < 3) {
        g += dx[0] * weight_turning * weight_reverse * weight_change_of_direction;
      } else {
        g += dx[0] * weight_turning * weight_reverse;
      }
    } else {
      g += dx[0] * weight_reverse;
    }
  }
}

//###################################################
//                                 3D NODE COMPARISON
//###################################################
bool Node3D::operator == (const Node3D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (t == rhs.t);
}
