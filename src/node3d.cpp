#include "node3d.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
using namespace HybridAStar;

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;
// possible movements
//const float Node3D::dy[] = { 0,        -0.032869,  0.032869};
//const float Node3D::dx[] = { 0.62832,   0.62717,   0.62717};
//const float Node3D::dt[] = { 0,         0.10472,   -0.10472};

// R = 6, 6.75 DEG
const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

// R = 3, 6.75 DEG
//const float Node3D::dy[] = { 0,        -0.0207946, 0.0207946};
//const float Node3D::dx[] = { 0.35342917352,   0.352612,  0.352612};
//const float Node3D::dt[] = { 0,         0.11780972451,   -0.11780972451};

//const float Node3D::dy[] = { 0,       -0.16578, 0.16578};
//const float Node3D::dx[] = { 1.41372, 1.40067, 1.40067};
//const float Node3D::dt[] = { 0,       0.2356194,   -0.2356194};

//###################################################
//                                         IS ON GRID
//###################################################
// bool Node3D::isOnGrid(const int width, const int height) const {
//   return x >= 0 && x < width &&
//          y >= 0 && y < height;
// }

//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::IsInRange(const Node3D &goal, const float &range) const
{
  int random = rand() % 10 + 1;
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < range;
}
//###################################################
//                                        IS close enough
//###################################################
bool Node3D::IsCloseEnough(const Node3D &goal, const float &distance_range, const float &angle_range) const
{
  float dx = std::abs(x - goal.x);
  float dy = std::abs(y - goal.y);
  float distance = (dx * dx) + (dy * dy);
  if (distance < distance_range)
  {
    float angle_diff = std::abs(t - goal.t);
    if (angle_diff <= angle_range)
    {
      DLOG(INFO) << "two node distance and orientation are close enough, return true";
      return true;
    }
    else
    {
      DLOG(INFO) << "two node distance is close enough but orientation is too far is " << angle_diff;
      return false;
    }
  }
  else
  {
    DLOG(INFO) << "too far, distance is " << distance;
    return false;
  }
}
//###################################################
//                                   CREATE SUCCESSOR
//###################################################
// Node3D* Node3D::createSuccessor(const int i) {
//   float xSucc, ySucc, tSucc;

//   // calculate successor positions forward
//   if (i < 3) {
//     xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
//     ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
//     tSucc = Utility::RadToZeroTo2P(t + dt[i]);
//   }
//   // backwards
//   else {
//     xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
//     ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
//     tSucc = Utility::RadToZeroTo2P(t - dt[i - 3]);
//   }

//   return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
// }

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
