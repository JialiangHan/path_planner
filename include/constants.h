#ifndef CONSTANTS
#define CONSTANTS
/*!
   \file constants.h
   \brief This is a collection of constants that are used throughout the project.
   \todo All constants need to be checked and documented
*/

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid

#include <cmath>

/*!
    \brief The namespace that wraps the entire project
    \namespace HybridAStar
*/

namespace HybridAStar {
/*!
    \brief The namespace that wraps constants.h
    \namespace Constants
*/
namespace Constants
{

  // _________________
  // GENERAL CONSTANTS

  /// [m] --- The width of the vehicle
  static const double width = 1.75;
  /// [m] --- The length of the vehicle
  static const double length = 2.65;
  /// [m] --- The minimum turning radius of the vehicle
  static const float min_turning_radius = 6;
  /// [m] --- The number of discretizations in heading
  static const int headings = 72;
  /// [°] --- The discretization value of the heading (goal condition)
  static const float deltaHeadingDeg = 360 / (float)headings;
  /// [c*M_PI] --- The discretization value of heading (goal condition)
  static const float deltaHeadingRad = 2 * M_PI / (float)headings;
  /// [c*M_PI] --- The heading part of the goal condition
  static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
  /// [m] --- The cell size of the 2D grid of the world
  static const float cellSize = 1;

  // ___________________
  // HEURISTIC CONSTANTS

  /// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
  static const float factor2D = sqrt(5) / sqrt(2) + 1;
  /// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
  static const float penaltyTurning = 1.05;
  /// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
  static const float penaltyReversing = 2.0;
  /// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
  static const float penaltyCOD = 2.0;
  /// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
  static const float dubinsShotDistance = 100;

  // ______________________
  // DUBINS LOOKUP SPECIFIC

  /// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
  static const int dubinsWidth = 15;

  // _________________________
  // COLLISION LOOKUP SPECIFIC

  /// [m] -- The bounding box size length and width to precompute all possible headings
  static const int bbSize = std::ceil((sqrt(width * width + length * length) + 4) / cellSize);
  /// [#] --- The sqrt of the number of discrete positions per cell
  static const int positionResolution = 10;
  /// [#] --- The number of discrete positions per cell
  static const int positions = positionResolution * positionResolution;
  /// A structure describing the relative position of the occupied cell based on the center of the vehicle
  struct relPos
  {
    /// the x position relative to the center
    int x;
    /// the y position relative to the center
    int y;
  };
  /// A structure capturing the lookup for each theta configuration
  struct config
  {
    /// the number of cells occupied by this configuration of the vehicle
    int length;
    /*!
     \var relPos pos[64]
     \brief The maximum number of occupied cells
     \todo needs to be dynamic
  */
    relPos pos[64];
  };
}
}

#endif // CONSTANTS

