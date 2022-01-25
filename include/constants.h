#pragma once
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
  static const float width = 1.75;
  /// [m] --- The length of the vehicle
  static const float length = 2.65;

  /// [m] --- The number of discretizations in heading
  static const int headings = 72;

  /// [c*M_PI] --- The discretization value of heading (goal condition)
  static const float deltaHeadingRad = 2 * M_PI / (float)headings;
  /// [m] --- The cell size of the 2D grid of the world
  static const float cellSize = 1;

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
