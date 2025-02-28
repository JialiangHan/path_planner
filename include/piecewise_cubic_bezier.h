/**
 * @file piecewise_cubic_bezier.h
 * @author Jialiang Han
 * @brief this class generate a piecewise cubic bezier curve
 * @version 0.2
 * @date 2021-12-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include <vector>
#include "cubic_bezier.h"
#include "utility.h"
namespace HybridAStar
{
  class PiecewiseCubicBezier
  {
  public:
    /**
   * @brief Construct a new Piecewise Cubic Bezier object
   * 
   */
    PiecewiseCubicBezier(){};
    /**
     * @brief Construct a new Piecewise Cubic Bezier object
     * 
     * @param control_points 
     */
    PiecewiseCubicBezier(const Eigen::Vector3f &start, const Eigen::Vector3f &goal)
    {
      // DLOG(INFO) << "PiecewiseCubicBezier in:";
      start_point_ = start;
      goal_point_ = goal;
      // DLOG(INFO) << "PiecewiseCubicBezier out";
    };

    /**
     * @brief get specific point on this bezier curve
     * 
     * @param u is between [0,1]
     * @return Eigen::Vector3f 
     */
    Eigen::Vector3f GetValueAt(const float &u);

    float GetAngleAt(const float &u);
    float GetCurvatureAt(const float &t);

    float GetTotalCurvature();

    void SetAnchorPoints(const std::vector<Eigen::Vector3f> &anchor_points_vec)
    {
      // DLOG(INFO) << "SetAnchorPoints in:";
      anchor_points3d_vec_ = anchor_points_vec;
      // GetAnchorPointDirection();
      // CalculateControlPoints();
      // CalculatePointsVec();
      CalculateCubicBezier();
      // DLOG(INFO) << "SetAnchorPoints out.";
    }

    /**
     * @brief Get the Length of bezier curve
     * 
     * @return float 
     */
    float GetLength()
    {
      // CalculateCubicBezier();
      CalculateLength();
      return length_;
    };

    std::vector<Eigen::Vector3f> ConvertPiecewiseCubicBezierToVector3f(const int &number_of_points);

    std::vector<Eigen::Vector3f> GetPointsVec() const { return points_vec_; };

    std::vector<Eigen::Vector3f> GetAnchorPoints() const { return anchor_points3d_vec_; };

    std::vector<CubicBezier::CubicBezier> GetCubicBezierVector()
    {
      CalculateCubicBezier();
      return cubic_bezier_vec_;
    };

  private:
    void CalculateLength();
    /**
     * @brief calculate control points according to anchor points,
     * 
     */
    void CalculateControlPoints();
    /**
     * @brief put start,goal anchor points and control points into this vector
     * 
     */
    void CalculatePointsVec();
    /**
     * @brief calculate cubic bezier object using points_vec_
     * 
     */
    void CalculateCubicBezier();

    void GetAnchorPointDirection();

  private:
    // int number_of_cubic_bezier_;
    std::vector<CubicBezier::CubicBezier> cubic_bezier_vec_;
    // points list contains both free,fixed anchor points and control points
    std::vector<Eigen::Vector3f> points_vec_;

    std::vector<Eigen::Vector3f> control_points_vec_;

    // list free anchor points(P), anchor points are the points which bezier curve pass through, not include start and goal
    std::vector<Eigen::Vector3f> anchor_points3d_vec_;

    // std::vector<Eigen::Vector3f> anchor_points2d_vec_;

    std::vector<Eigen::Vector3f> anchor_points_dir_vec_;

    Eigen::Vector3f start_point_;
    Eigen::Vector3f goal_point_;
    /**
     * @brief curve length
     * 
     */
    float length_ = 0;
  };
}
