#include "smoother.h"

using namespace HybridAStar;
//###################################################
//                                     CUSP DETECTION
//###################################################
inline bool isCusp(const std::vector<Node3D>& path, int i) {
  bool revim2 = path[i - 2].getPrim() > 3;
  bool revim1 = path[i - 1].getPrim() > 3;
  bool revi = path[i].getPrim() > 3;
  bool revip1 = path[i + 1].getPrim() > 3;
  //  bool revip2 = path[i + 2].getPrim() > 3 ;
  return (revim2 != revim1 || revim1 != revi || revi != revip1);
}

void Smoother::Clear()
{
  path_.clear();
  preprocessed_path_.clear();
}
//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::SmoothPath(const DynamicVoronoi &voronoi)
{
  // load the current voronoi diagram into the smoother
  this->voronoi_ = voronoi;
  this->map_width_ = voronoi.getSizeX();
  this->map_height_ = voronoi.getSizeY();
  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // remove duplicate points
  PreprocessPath();
  // path objects with all nodes oldPath the original, smoothed_path the resulting smoothed path
  std::vector<Node3D> smoothed_path = preprocessed_path_;
  std::vector<Node3D> path_before_smooth = preprocessed_path_;
  // descent along the gradient until the maximum number of iterations has been reached
  float total_weight = params_.weight_smoothness + params_.weight_curvature + params_.weight_voronoi + params_.weight_obstacle + params_.weight_length;
  if (params_.weight_curvature < 0 ||
      params_.weight_length < 0 ||
      params_.weight_obstacle < 0 ||
      params_.weight_smoothness < 0 ||
      params_.weight_voronoi < 0)
  {
    DLOG(WARNING) << "one of weighting is smaller than 0!!";
  }
  else if (total_weight <= 0)
  {
    DLOG(WARNING) << "total weight is smaller or equal to 0!!!!";
  }
  else
  {
    // DLOG(INFO) << "preprocessed path size is: " << preprocessed_path_.size();
    if (path_before_smooth.size() < 5)
    {
      DLOG(INFO) << "no enough points!!";
      return;
    }
    while (iterations < params_.max_iterations)
    {
      // DLOG(INFO) << iterations << " starts!";
      // choose the first three nodes of the path
      for (uint i = 2; i < path_before_smooth.size() - 2; ++i)
      {
        Eigen::Vector2f xim2(path_before_smooth[i - 2].getX(), path_before_smooth[i - 2].getY());
        Eigen::Vector2f xim1(path_before_smooth[i - 1].getX(), path_before_smooth[i - 1].getY());
        Eigen::Vector2f xi(path_before_smooth[i].getX(), path_before_smooth[i].getY());
        Eigen::Vector2f xip1(path_before_smooth[i + 1].getX(), path_before_smooth[i + 1].getY());
        Eigen::Vector2f xip2(path_before_smooth[i + 2].getX(), path_before_smooth[i + 2].getY());
        Eigen::Vector2f correction(0, 0);
        // //DLOG(INFO) << "xim2: " << xim2(0,0) << " " << xim2(1,0)
        //  << "xim1: " << xim1(0,0) << " " << xim1(1,0)
        //  << "xi: " << xi(0,0) << " " << xi(1,0)
        //  << "xip1:" << xip1(0,0) << " " << xip1(1,0)
        //  << "xip2:" << xip2(0,0) << " " << xip2(1,0);

        // the following points shall not be smoothed
        // keep these points fixed if they are a cusp point or adjacent to one
        // DLOG(INFO) << i << "th node before correction: x: " << xi(0, 0) << " y: " << xi(1, 0);
        if (isCusp(path_before_smooth, i))
        {
          DLOG(INFO) << "node is cusp,skip it!";
          continue;
        }
        // DLOG(INFO) << "correction is " << correction.x() << " " << correction.y();
        // ensure that it is on the grid
        // correction = correction - CurvatureTerm(xim2, xim1, xi, xip1, xip2);

        if (params_.weight_curvature > 0)
        {
          correction = correction - CurvatureTerm(xim1, xi, xip1);
          // DLOG(INFO) << "correction is " << correction.x() << " " << correction.y();
          // DLOG(INFO) << "node after curvature correction is " << (xi + correction).x() << " " << (xi + correction).y();
          if (!isOnGrid(xi + correction))
          {
            DLOG(INFO) << "node after curvature correction is not on grid!!";
            continue;
          }
        }

        if (params_.weight_obstacle > 0)
        {
          // ensure that it is on the grid
          correction = correction - ObstacleTerm(xi);
          // DLOG(INFO) << "node after curvature and obstacle correction is " << (xi + correction).x() << " " << (xi + correction).y();
          if (!isOnGrid(xi + correction))
          {
            DLOG(INFO) << "node after curvature and obstacle correction is not on grid!!";
            continue;
          }
        }

        if (params_.weight_voronoi > 0)
        {
          correction = correction - VoronoiTerm(xi);
          // DLOG(INFO) << "node after curvature, obstacles and voronoi correction  is " << (xi + correction).x() << " " << (xi + correction).y();
          if (!isOnGrid(xi + correction))
          {
            DLOG(INFO) << "node after curvature, obstacles and voronoi correction is not on grid!!";
            continue;
          }
        }

        if (params_.weight_smoothness > 0)
        {
          // ensure that it is on the grid
          correction = correction - SmoothnessTerm(xim2, xim1, xi, xip1, xip2);
          // DLOG(INFO) << "node after curvature, obstacles, voronoi and smoothness correction is " << (xi + correction).x() << " " << (xi + correction).y();
          if (!isOnGrid(xi + correction))
          {
            DLOG(INFO) << "node after curvature, obstacles, voronoi and smoothness correction is not on grid!!";
            continue;
          }
        }

        if (params_.weight_length > 0)
        {
          correction = correction - PathLengthTerm(xim1, xi, xip1);
          // DLOG(INFO) << "node after curvature, obstacles, voronoi and smoothness correction is " << (xi + correction).x() << " " << (xi + correction).y();
          if (!isOnGrid(xi + correction))
          {
            DLOG(INFO) << "node after curvature, obstacles, voronoi and smoothness correction is not on grid!!";
            continue;
          }
        }

        xi = xi + params_.alpha * correction / total_weight;
        if (correction(0, 0) == 0 && correction(1, 0) == 0)
        {
          DLOG(WARNING) << "correction is zero, no correction performed!!!";
        }
        smoothed_path[i].setX(xi(0, 0));
        smoothed_path[i].setY(xi(1, 0));
        Eigen::Vector2f Dxi = xi - xim1;
        smoothed_path[i - 1].setT(std::atan2(Dxi(1, 0), Dxi(0, 0)));
        // DLOG(INFO) << i << "th node after correction: x: " << xi(0, 0) << " y: " << xi(1, 0);
      }

      //add termination for while loop
      if (GetPathDiff(smoothed_path, path_before_smooth) < params_.epsilon)
      {
        DLOG(INFO) << "path diff before and after is too small, terminate while loop!!";
        break;
      }
      else
      {
        path_before_smooth = smoothed_path;
      }
      iterations++;
    }
  }

  path_ = smoothed_path;
  // for (auto node : path_)
  // {
  //   DLOG(INFO) << "smoothed path is x: " << node.getX() << " y: " << node.getY() << " t: " << node.getT();
  // }
}

//###################################################
//                                      OBSTACLE TERM
//###################################################
Eigen::Vector2f Smoother::ObstacleTerm(const Eigen::Vector2f &xi)
{
  Eigen::Vector2f gradient(0, 0);
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi_.getDistance(xi(0, 0), xi(1, 0));
  // the vector determining where the obstacle is
  int x = (int)xi(0, 0);
  int y = (int)xi(1, 0);
  // if the node is within the map
  if (x < map_width_ && x >= 0 && y < map_height_ && y >= 0)
  {
    Eigen::Vector2f obsVct(xi(0, 0) - voronoi_.data[(int)xi(0, 0)][(int)xi(1, 0)].obstX,
                           xi(1, 0) - voronoi_.data[(int)xi(0, 0)][(int)xi(1, 0)].obstY);

    // the closest obstacle is closer than desired correct the path for that
    if (obsDst < params_.obsd_max && obsDst > 1e-6)
    {
      return gradient = params_.weight_obstacle * 2 * (obsDst - params_.obsd_max) * obsVct / obsDst;
    }
  }
  return gradient;
}

//###################################################
//                                       VORONOI TERM
//###################################################
Eigen::Vector2f Smoother::VoronoiTerm(const Eigen::Vector2f &xi)
{
  Eigen::Vector2f gradient(0, 0);

  float obsDst = voronoi_.getDistance(xi(0, 0), xi(1, 0));
  // the vector determining where the obstacle is
  Eigen::Vector2f obsVct(xi(0, 0) - voronoi_.data[(int)xi(0, 0)][(int)xi(1, 0)].obstX,
                         xi(1, 0) - voronoi_.data[(int)xi(0, 0)][(int)xi(1, 0)].obstY);
  // distance to the closest voronoi edge
  float edgDst = 0.0;
  INTPOINT closest_edge_pt = voronoi_.GetClosestVoronoiEdgePoint(xi, edgDst);
  // the vector determining where the voronoi edge is
  Eigen::Vector2f edgVct(xi(0, 0) - closest_edge_pt.x, xi(1, 0) - closest_edge_pt.y);
  //calculate the distance to the closest obstacle from the current node

  if (obsDst < params_.vor_obs_dmax && obsDst > 1e-6)
  {
    //calculate the distance to the closest GVD edge from the current node
    // the node is away from the optimal free space area
    if (edgDst > 0)
    {
      // equations can be found in paper "Path Planning for autonomous vehicle in unknown semi-structured environments"
      Eigen::Vector2f PobsDst_Pxi = obsVct / obsDst;
      Eigen::Vector2f PedgDst_Pxi = edgVct / edgDst;

      float PvorPtn_PedgDst = (params_.alpha / (params_.alpha + obsDst)) * ((obsDst - params_.vor_obs_dmax) / params_.vor_obs_dmax * (obsDst - params_.vor_obs_dmax) / params_.vor_obs_dmax) * (obsDst / ((obsDst + edgDst) * (obsDst + edgDst)));

      float PvorPtn_PobsDst = (params_.alpha / (params_.alpha + obsDst)) * (edgDst / (obsDst + edgDst)) * ((obsDst - params_.vor_obs_dmax) / (params_.vor_obs_dmax * params_.vor_obs_dmax)) * (-(obsDst - params_.vor_obs_dmax) / (params_.alpha + obsDst) - (obsDst - params_.vor_obs_dmax) / (obsDst + edgDst) + 2);

      gradient = params_.weight_voronoi * (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi);

      return gradient;
    }
    return gradient;
  }
  return gradient;
}
/***************original curvature term**************/
Eigen::Vector2f Smoother::CurvatureTerm(const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1)
{
  Eigen::Vector2f gradient(0, 0);
  // the vectors between the nodes
  Eigen::Vector2f Dxi = xi - xim1;
  Eigen::Vector2f Dxip1 = xip1 - xi;
  // orthogonal complements vector
  Eigen::Vector2f p1, p2;

  // the distance of the vectors
  float absDxi = Dxi.norm();
  float absDxip1 = Dxip1.norm();

  // ensure that the absolute values are not null
  if (absDxi > 0 && absDxip1 > 0)
  {
    // the angular change at the node
    float Dphi = std::acos(Utility::Clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1), 1, -1));
    float kappa = Dphi / absDxi;

    // if the curvature is smaller then the maximum do nothing
    if (kappa <= 1 / params_.min_turning_radius)
    {
      return gradient;
    }
    else
    {
      //代入原文公式(2)与(3)之间的公式
      //参考：
      // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for
      //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
      float absDxi1Inv = 1 / absDxi;
      float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
      float u = -absDxi1Inv * PDphi_PcosDphi;
      // calculate the p1 and p2 terms
      p1 = OrthogonalComplements(xi, -xip1) / (absDxi * absDxip1); //公式(4)
      p2 = OrthogonalComplements(-xip1, xi) / (absDxi * absDxip1);
      // calculate the last terms
      float s = Dphi / (absDxi * absDxi);
      Eigen::Vector2f ones(1, 1);
      Eigen::Vector2f ki = u * (-p1 - p2) - (s * ones);
      Eigen::Vector2f kim1 = u * p2 - (s * ones);
      Eigen::Vector2f kip1 = u * p1;

      // calculate the gradient
      gradient = params_.weight_curvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

      if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
      {
        DLOG(INFO) << "nan values in curvature term";
        Eigen::Vector2f zeros(0, 0);
        return zeros;
      }
      // return gradient of 0
      else
      {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else
  {
    DLOG(INFO) << "abs values not larger than 0";
    Eigen::Vector2f zeros(0, 0);
    return zeros;
  }
}
//###################################################
//                                     CURVATURE TERM
//###################################################
Eigen::Vector2f Smoother::CurvatureTerm(const Eigen::Vector2f &xim2, const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1, const Eigen::Vector2f &xip2)
{
  //use curvature square as cost function for curvature term
  Eigen::Vector2f gradient(0, 0);
  // the vectors between the nodes
  Eigen::Vector2f Dxi = xi - xim1;
  Eigen::Vector2f Dxip1 = xip1 - xi;
  Eigen::Vector2f Dxip2 = xip2 - xip1;
  Eigen::Vector2f Dxim1 = xim1 - xim2;

  // the distance of the vectors
  float norm_Dxi = Dxi.norm();
  float norm_Dxip1 = Dxip1.norm();
  float norm_Dxim1 = Dxim1.norm();
  float norm_Dxip2 = Dxip2.norm();
  // ensure that the absolute values are not null
  if (norm_Dxi > 0 && norm_Dxim1 > 0 && norm_Dxip1 > 0)
  {
    // the angular change at the node
    float Dphi = std::acos(Utility::Clamp(Dxi.dot(Dxip1) / (norm_Dxi * norm_Dxip1), 1, -1));
    float Dphip1 = std::acos(Utility::Clamp(Dxip1.dot(Dxip2) / (norm_Dxip1 * norm_Dxip2), 1, -1));
    float Dphim1 = std::acos(Utility::Clamp(Dxim1.dot(Dxi) / (norm_Dxim1 * norm_Dxi), 1, -1));
    float curvature = abs(Dphi) / norm_Dxi;
    DLOG(INFO) << "curvature is " << curvature;
    //only optimize some points which have curvature greater than 1/min_turning_radius
    if (curvature < 1 / params_.min_turning_radius)
    {
      return gradient;
    }
    if (Dphim1 == 0 || Dphi == 0 || Dphip1 == 0)
    {
      // DLOG(INFO) << "one of the changing angle is 0!!!";
      if (Dphi == 0)
      {
        // DLOG(INFO) << "Dxi.dot(Dxip1) / (norm_Dxi * norm_Dxip1) " << Dxi.dot(Dxip1) / (norm_Dxi * norm_Dxip1);
        // DLOG(INFO) << "Utility::Clamp(Dxi.dot(Dxip1) / (norm_Dxi * norm_Dxip1), 1, -1) " << Utility::Clamp(Dxi.dot(Dxip1) / (norm_Dxi * norm_Dxip1), 1, -1);
        // DLOG(INFO) << "Dxi is " << Dxi.x() << " " << Dxi.y();
        // DLOG(INFO) << "Dxip1 is " << Dxip1.x() << " " << Dxip1.y();
      }
      if (Dphip1 == 0)
      {
        //   DLOG(INFO) << "Dxip1.dot(Dxip2) / (norm_Dxip1 * norm_Dxip2) " << Dxip1.dot(Dxip2) / (norm_Dxip1 * norm_Dxip2);
        //   DLOG(INFO) << "Utility::Clamp(Dxip1.dot(Dxip2) / (norm_Dxip1 * norm_Dxip2), 1, -1) " << Utility::Clamp(Dxip1.dot(Dxip2) / (norm_Dxip1 * norm_Dxip2), 1, -1);
        //   DLOG(INFO) << "Dxip1 is " << Dxip1.x() << " " << Dxip1.y();
        //   DLOG(INFO) << "Dxip2 is " << Dxip2.x() << " " << Dxip2.y();
      }
      if (Dphim1 == 0)
      {
        // DLOG(INFO) << "Dxim1.dot(Dxi) / (norm_Dxim1 * norm_Dxi) " << Dxim1.dot(Dxi) / (norm_Dxim1 * norm_Dxi);
        // DLOG(INFO) << "Utility::Clamp(Dxim1.dot(Dxi) / (norm_Dxim1 * norm_Dxi), 1, -1) " << Utility::Clamp(Dxim1.dot(Dxi) / (norm_Dxim1 * norm_Dxi), 1, -1);
        // DLOG(INFO) << "Dxim1 is " << Dxim1.x() << " " << Dxim1.y();
        // DLOG(INFO) << "Dxi is " << Dxi.x() << " " << Dxi.y();
      }
      return gradient;
      ;
    }
    float absDxi1Inv = 1 / norm_Dxi;
    float absDxim1Inv = 1 / norm_Dxim1;
    float absDxip1Inv = 1 / norm_Dxip1;

    float PDphi_PcosDphi = -1 / std::sqrt(1 - std::cos(Dphi) * std::cos(Dphi));
    float PDphim1_PcosDphim1 = -1 / std::sqrt(1 - std::cos(Dphim1) * std::cos(Dphim1));
    float PDphip1_PcosDphip1 = -1 / std::sqrt(1 - std::cos(Dphip1) * std::cos(Dphip1));

    float ui = absDxi1Inv * PDphi_PcosDphi;
    float uim1 = absDxim1Inv * PDphim1_PcosDphim1;
    float uip1 = absDxip1Inv * PDphip1_PcosDphip1;

    Eigen::Vector2f PcosDphi_Pxi = -OrthogonalComplements(xi, -xip1) / (norm_Dxi * norm_Dxip1) - OrthogonalComplements(-xip1, xi) / (norm_Dxi * norm_Dxip1);
    Eigen::Vector2f PcosDphim1_Pxi = OrthogonalComplements(xim1, -xi) / (norm_Dxim1 * norm_Dxi);
    Eigen::Vector2f PcosDphip1_Pxi = OrthogonalComplements(-xip2, xip1) / (norm_Dxip1 * norm_Dxip2);
    Eigen::Vector2f ones(1, 1);
    Eigen::Vector2f Pcurvature_m_Pxi = uim1 * PcosDphim1_Pxi;
    Eigen::Vector2f Pcurvature_i_Pxi = ui * PcosDphi_Pxi - Dphi / (norm_Dxi * norm_Dxi) * ones;
    Eigen::Vector2f Pcurvature_p_Pxi = uip1 * PcosDphip1_Pxi + Dphip1 / (norm_Dxip1 * norm_Dxip1) * ones;

    // calculate the gradient
    gradient = params_.weight_curvature * (2 * Pcurvature_m_Pxi + 2 * Pcurvature_i_Pxi + 2 * Pcurvature_p_Pxi);

    if (std::isnan(gradient(0, 0)) || std::isnan(gradient(1, 0)))
    {
      DLOG(WARNING) << "nan values in curvature term";
      //DLOG(INFO) << "gradient is: " << gradient(0, 0) << " " << gradient(1, 0);
      // //DLOG(INFO) << "weighting is " << params_.weight_curvature;
      // //DLOG(INFO) << "Pcurvature_m_Pxi is nan: x: " << Pcurvature_m_Pxi(0,0) << " y: " << Pcurvature_m_Pxi(1,0);
      // //DLOG(INFO) << "Pcurvature_i_Pxi is nan: x: " << Pcurvature_i_Pxi(0,0) << " y: " << Pcurvature_i_Pxi(1,0);
      // //DLOG(INFO) << "Pcurvature_p_Pxi is nan: x: " << Pcurvature_p_Pxi(0,0) << " y: " << Pcurvature_p_Pxi(1,0);
      if (std::isinf(Pcurvature_m_Pxi(0, 0)) || std::isinf(Pcurvature_m_Pxi(1, 0)))
      {
        //DLOG(INFO) << "Pcurvature_m_Pxi is inf: x: " << Pcurvature_m_Pxi(0, 0) << " y: " << Pcurvature_m_Pxi(1, 0);
        if (std::isinf(uim1))
        {
          //DLOG(INFO) << "uim1 is " << uim1;
          if (std::isinf(absDxim1Inv))
          {
            //DLOG(INFO) << "absDxim1Inv is " << absDxim1Inv;
            //DLOG(INFO) << "norm_Dxim1 is " << norm_Dxim1;
          }
          if (std::isinf(PDphim1_PcosDphim1))
          {
            //DLOG(INFO) << "PDphim1_PcosDphim1 is " << PDphim1_PcosDphim1;
            if ((1 - std::cos(Dphim1) * std::cos(Dphim1)) == 0)
            {
              //DLOG(INFO) << "1- cosDpim1 == 0";
              //DLOG(INFO) << "Dphim1 is " << Dphim1;
            }
          }
        }
        if (std::isinf(PcosDphim1_Pxi(0, 0)) || std::isinf(PcosDphim1_Pxi(1, 0)))
        {
          //DLOG(INFO) << "PcosDphim1_Pxi is inf: x: " << PcosDphim1_Pxi(0, 0) << " y: " << PcosDphim1_Pxi(1, 0);
          if (norm_Dxim1 * norm_Dxi == 0)
          {
            //DLOG(INFO) << "norm_Dxim1 is: " << norm_Dxim1 << " norm_Dxi is: " << norm_Dxi;
          }
          if (std::isinf(OrthogonalComplements(xim1, -xi)(0, 0)) || std::isinf(OrthogonalComplements(xim1, -xi)(1, 0)))
          {
            //DLOG(INFO) << "xim1 is: x: " << xim1(0, 0) << " y: " << xim1(1, 0) << " length is: " << xim1.norm();
            //DLOG(INFO) << "xi is: x: " << xi(0, 0) << " y: " << xi(1, 0) << " length is: " << xi.norm();
          }
        }
      }
      if (std::isnan(Pcurvature_m_Pxi(0, 0)) || std::isnan(Pcurvature_m_Pxi(1, 0)))
      {
        //DLOG(INFO) << "Pcurvature_m_Pxi is nan: x: " << Pcurvature_m_Pxi(0, 0) << " y: " << Pcurvature_m_Pxi(1, 0);
        if (std::isnan(PcosDphim1_Pxi(0, 0)) || std::isnan(PcosDphim1_Pxi(1, 0)))
        {
          //DLOG(INFO) << "PcosDphim1_Pxi is nan: x: " << PcosDphim1_Pxi(0, 0) << " y: " << PcosDphim1_Pxi(1, 0);
          if (norm_Dxim1 * norm_Dxi == 0)
          {
            //DLOG(INFO) << "norm_Dxim1 is: " << norm_Dxim1 << " norm_Dxi is: " << norm_Dxi;
          }
          if (std::isnan(OrthogonalComplements(xim1, -xi)(0, 0)) || std::isnan(OrthogonalComplements(xim1, -xi)(1, 0)))
          {
            //DLOG(INFO) << "xim1 is: x: " << xim1(0, 0) << " y: " << xim1(1, 0) << " length is: " << xim1.norm();
            //DLOG(INFO) << "xi is: x: " << xi(0, 0) << " y: " << xi(1, 0) << " length is: " << xi.norm();
          }
        }
      }
      if (std::isnan(Pcurvature_i_Pxi(0, 0)) || std::isnan(Pcurvature_i_Pxi(1, 0)))
      {
        //DLOG(INFO) << "Pcurvature_i_Pxi is nan: x: " << Pcurvature_i_Pxi(0, 0) << " y: " << Pcurvature_i_Pxi(1, 0);
        if (std::isnan(PcosDphi_Pxi(0, 0)) || std::isnan(PcosDphi_Pxi(1, 0)))
        {
          //DLOG(INFO) << "PcosDphi_Pxi is nan: x: " << PcosDphi_Pxi(0, 0) << " y: " << PcosDphi_Pxi(1, 0);
          if (norm_Dxi * norm_Dxip1 == 0)
          {
            //DLOG(INFO) << "norm_Dxi is: " << norm_Dxi << " norm_Dxip1 is: " << norm_Dxip1;
          }
          if (std::isnan(OrthogonalComplements(xi, -xip1)(0, 0)) || std::isnan(OrthogonalComplements(xi, -xip1)(1, 0)))
          {
            //DLOG(INFO) << "xip1 is: x: " << xip1(0, 0) << " y: " << xip1(1, 0) << " length is: " << xip1.norm();
            //DLOG(INFO) << "xi is: x: " << xi(0, 0) << " y: " << xi(1, 0) << " length is: " << xi.norm();
          }
        }
        if (norm_Dxi == 0)
        {
          //DLOG(INFO) << "norm_Dxi is: " << norm_Dxi;
        }
        if (std::isnan(Dphi))
        {
          //DLOG(INFO) << "Dphi is " << Dphi;
          //DLOG(INFO) << "inside acos : " << Dxi.dot(Dxip1) / (norm_Dxi * norm_Dxip1);
        }
      }
      if (std::isnan(Pcurvature_p_Pxi(0, 0)) || std::isnan(Pcurvature_p_Pxi(1, 0)))
      {
        //DLOG(INFO) << "Pcurvature_p_Pxi is nan: x: " << Pcurvature_p_Pxi(0, 0) << " y: " << Pcurvature_p_Pxi(1, 0);
        if (std::isnan(PcosDphip1_Pxi(0, 0)) || std::isnan(PcosDphip1_Pxi(1, 0)))
        {
          //DLOG(INFO) << "PcosDphip1_Pxi is nan: x: " << PcosDphip1_Pxi(0, 0) << " y: " << PcosDphip1_Pxi(1, 0);
          if (norm_Dxip1 * norm_Dxip2 == 0)
          {
            //DLOG(INFO) << "norm_Dxip1 is: " << norm_Dxip1 << " norm_Dxip2 is: " << norm_Dxip2;
          }
          if (std::isnan(OrthogonalComplements(-xip2, xip1)(0, 0)) || std::isnan(OrthogonalComplements(-xip2, xip1)(1, 0)))
          {
            //DLOG(INFO) << "xip2 is: x: " << xip2(0, 0) << " y: " << xip2(1, 0) << " length is: " << xip2.norm();
            //DLOG(INFO) << "xip1 is: x: " << xip1(0, 0) << " y: " << xip1(1, 0) << " length is: " << xip1.norm();
          }
        }
        if (norm_Dxip1 == 0)
        {
          //DLOG(INFO) << "norm_Dxip1 is: " << norm_Dxip1;
        }
        if (std::isnan(Dphip1))
        {
          //DLOG(INFO) << "Dphip1 is " << Dphip1;
          //DLOG(INFO) << "inside acos : " << Dxip1.dot(Dxip2) / (norm_Dxip1 * norm_Dxip2);
        }
      }
      Eigen::Vector2f zeros(0, 0);
      return zeros;
    }
    // return gradient of 0
    else
    {
      return gradient;
    }
  }
  // return gradient of 0
  else
  {
    //DLOG(WARNING) << "abs values not larger than 0";
    if (norm_Dxi <= 0)
    {
      //DLOG(INFO) << "absDxi value is: " << norm_Dxi      << " Dxi x: " << Dxi(0, 0)      << " Dxi y: " << Dxi(1, 0)      << " xi x: " << xi(0, 0)      << " xi y: " << xi(1, 0)      << " xim1 x: " << xim1(0, 0)      << " xim1 y: " << xim1(1, 0);
      if (xi == xim1)
      {
        //DLOG(WARNING) << "xi is equal to xim1!!!!";
      }
    }
    if (norm_Dxip1 <= 0)
    {
      //DLOG(INFO) << "absDxip1 value is: " << norm_Dxip1      << " Dxip1 x: " << Dxip1(0, 0)      << " Dxip1 y: " << Dxip1(1, 0) << " xip1 x: " << xip1(0, 0)      << " xip1 y: " << xip1(1, 0)      << " xi x: " << xi(0, 0)      << " xi y: " << xi(1, 0);
      if (xi == xip1)
      {
        //DLOG(WARNING) << "xi is equal to xip1!!!!";
      }
    }
    Eigen::Vector2f zeros(0, 0);
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Eigen::Vector2f Smoother::SmoothnessTerm(const Eigen::Vector2f &xim2, const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1, const Eigen::Vector2f &xip2)
{
  //this is correct, see https://zhuanlan.zhihu.com/p/118666410
  return params_.weight_smoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}
Eigen::Vector2f Smoother::PathLengthTerm(const Eigen::Vector2f &xim1, const Eigen::Vector2f &xi, const Eigen::Vector2f &xip1)
{
  return params_.weight_length * 2 * (2 * xi - xim1 - xip1);
}
int Smoother::PreprocessPath()
{
  //remove some duplicates
  for (uint index = 0; index < path_.size(); ++index)
  {
    // //DLOG(INFO) << "path point is x: " << path_[index].getX() << " y: " << path_[index].getY() << " t: " << path_[index].getT();
    if (index != path_.size() - 1)
    {
      if (path_[index] == path_[index + 1])
      {
        //DLOG(WARNING) << "path point: " << index << " is equal to next point!!";
        continue;
      }
    }
    preprocessed_path_.emplace_back(path_[index]);
  }
  return 1;
}
float Smoother::GetPathDiff(const std::vector<Node3D> &path_before_smooth, const std::vector<Node3D> &path_after_smooth)
{
  if (path_before_smooth.size() != path_after_smooth.size())
  {
    //DLOG(WARNING) << "two path size are not equal!!!";
    return INFINITY;
  }
  else
  {
    float diff = 0.0;
    for (uint i = 0; i < path_after_smooth.size(); ++i)
    {
      Eigen::Vector2f xi_before(path_before_smooth[i].getX(), path_before_smooth[i].getY());
      Eigen::Vector2f xi_after(path_after_smooth[i].getX(), path_after_smooth[i].getY());
      diff += (xi_after - xi_before).norm();
    }
    diff = diff / path_after_smooth.size();
    // DLOG(INFO) << "path diff is " << diff;
    // //DLOG(INFO) << "path diff is : " << diff;
    return diff;
  }
}

Eigen::Vector2f Smoother::OrthogonalComplements(const Eigen::Vector2f &a, const Eigen::Vector2f &b)
{
  Eigen::Vector2f out;
  out = a - a.dot(b) * b / b.norm() / b.norm();
  return out;
}
