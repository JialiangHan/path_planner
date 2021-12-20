#include "smoother.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
using namespace HybridAStar;
//###################################################
//                                     CUSP DETECTION
//###################################################
inline bool isCusp(const std::vector<Node3D>& path, int i) {
  bool revim2 = path[i - 2].getPrim() > 3 ;
  bool revim1 = path[i - 1].getPrim() > 3 ;
  bool revi   = path[i].getPrim() > 3 ;
  bool revip1 = path[i + 1].getPrim() > 3 ;
  //  bool revip2 = path[i + 2].getPrim() > 3 ;

  return (revim2 != revim1 || revim1 != revi || revi != revip1);
}
//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath(DynamicVoronoi& voronoi) {
  // load the current voronoi diagram into the smoother
  this->voronoi_ = voronoi;
  this->map_width_ = voronoi.getSizeX();
  this->map_height_ = voronoi.getSizeY();
  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // the maximum iterations for the gd smoother
  int maxIterations = 1000;

  PreprocessPath();
  // path objects with all nodes oldPath the original, smoothed_path the resulting smoothed path
  std::vector<Node3D> smoothed_path = preprocessed_path_;

  // descent along the gradient until the maximum number of iterations has been reached
  float totalWeight = weight_smoothness_ + weight_curvature_ + weight_voronoi_ + weight_obstacle_ + weight_length_;
  DLOG(INFO) << "preprocessed path size is: " << preprocessed_path_.size();
  while (iterations < maxIterations)
  {
    DLOG(INFO) << iterations << " starts!";
    // choose the first three nodes of the path
    for (uint i = 2; i < smoothed_path.size() - 2; ++i)
    {

      Vector2D xim2(smoothed_path[i - 2].getX(), smoothed_path[i - 2].getY());
      Vector2D xim1(smoothed_path[i - 1].getX(), smoothed_path[i - 1].getY());
      Vector2D xi(smoothed_path[i].getX(), smoothed_path[i].getY());
      Vector2D xip1(smoothed_path[i + 1].getX(), smoothed_path[i + 1].getY());
      Vector2D xip2(smoothed_path[i + 2].getX(), smoothed_path[i + 2].getY());
      Vector2D correction;
      // DLOG(INFO) << "xim2: " << xim2.getX() << " " << xim2.getY()
      //  << "xim1: " << xim1.getX() << " " << xim1.getY()
      //  << "xi: " << xi.getX() << " " << xi.getY()
      //  << "xip1:" << xip1.getX() << " " << xip1.getY()
      //  << "xip2:" << xip2.getX() << " " << xip2.getY();

      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      DLOG(INFO) << i << "th node before correction: x: " << xi.getX() << " y: " << xi.getY();
      if (isCusp(smoothed_path, i))
      {
        DLOG(INFO) << "node is cusp,skip it!";
        continue;
      }

      // ensure that it is on the grid
      correction = correction - curvatureTerm(xim2, xim1, xi, xip1, xip2);
      if (!isOnGrid(xi + correction))
      {
        DLOG(INFO) << "node after curvature correction is not on grid!!";
        continue;
      }
      // ensure that it is on the grid
      correction = correction - obstacleTerm(xi);
      if (!isOnGrid(xi + correction))
      {
        DLOG(INFO) << "node after curvature and obstacle correction is not on grid!!";
        continue;
      }

      correction = correction - voronoiTerm(xi);
      if (!isOnGrid(xi + correction))
      {
        DLOG(INFO) << "node after curvature, obstacles and voronoi correction is not on grid!!";
        continue;
      }
      // ensure that it is on the grid
      correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      if (!isOnGrid(xi + correction))
      {
        DLOG(INFO) << "node after curvature, obstacles, voronoi and smoothness correction is not on grid!!";
        continue;
      }
      correction = correction - PathLengthTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction))
      {
        DLOG(INFO) << "node after curvature, obstacles, voronoi and smoothness correction is not on grid!!";
        continue;
      }

      xi = xi + alpha_ * correction / totalWeight;
      if (correction.getX() == 0 && correction.getY() == 0)
      {
        DLOG(WARNING) << "correction is zero, no correction performed!!!";
      }
      smoothed_path[i].setX(xi.getX());
      smoothed_path[i].setY(xi.getY());
      Vector2D Dxi = xi - xim1;
      smoothed_path[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));
      DLOG(INFO) << i << "th node after correction: x: " << xi.getX() << " y: " << xi.getY();
    }

    iterations++;
  }

  path_ = smoothed_path;
  for (auto node : path_)
  {
    DLOG(INFO) << "smoothed path is x: " << node.getX() << " y: " << node.getY() << " t: " << node.getT();
  }
}

void Smoother::tracePath(const Node3D *node, int i, std::vector<Node3D> path)
{
  if (node == nullptr)
  {
    this->path_ = path;
    return;
  }

  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}

//###################################################
//                                      OBSTACLE TERM
//###################################################
Vector2D Smoother::obstacleTerm(Vector2D xi)
{
  Vector2D gradient;
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi_.getDistance(xi.getX(), xi.getY());
  // the vector determining where the obstacle is
  int x = (int)xi.getX();
  int y = (int)xi.getY();
  // if the node is within the map
  if (x < map_width_ && x >= 0 && y < map_height_ && y >= 0)
  {
    Vector2D obsVct(xi.getX() - voronoi_.data[(int)xi.getX()][(int)xi.getY()].obstX,
                    xi.getY() - voronoi_.data[(int)xi.getX()][(int)xi.getY()].obstY);

    // the closest obstacle is closer than desired correct the path for that
    if (obsDst < obsd_max_ && obsDst > 1e-6)
    {
      return gradient = weight_obstacle_ * 2 * (obsDst - obsd_max_) * obsVct / obsDst;
    }
  }
  return gradient;
}

//###################################################
//                                       VORONOI TERM
//###################################################
Vector2D Smoother::voronoiTerm(Vector2D xi)
{
  Vector2D gradient;

  float obsDst = voronoi_.getDistance(xi.getX(), xi.getY());
  // the vector determining where the obstacle is
  Vector2D obsVct(xi.getX() - voronoi_.data[(int)xi.getX()][(int)xi.getY()].obstX,
                  xi.getY() - voronoi_.data[(int)xi.getX()][(int)xi.getY()].obstY);
  // distance to the closest voronoi edge
  double edgDst = 0.0;
  INTPOINT closest_edge_pt = voronoi_.GetClosestVoronoiEdgePoint(xi, edgDst);
  // the vector determining where the voronoi edge is
  Vector2D edgVct(xi.getX() - closest_edge_pt.x, xi.getY() - closest_edge_pt.y);
  //calculate the distance to the closest obstacle from the current node
  //obsDist =  voronoiDiagram.getDistance(node->getX(),node->getY())

  if (obsDst < vor_obs_dmax_ && obsDst > 1e-6)
  {
    //calculate the distance to the closest GVD edge from the current node
    // the node is away from the optimal free space area
    if (edgDst > 0)
    {
      // equations can be found in paper "Path Planning for autonomous vehicle in unknown semi-structured environments"
      Vector2D PobsDst_Pxi = obsVct / obsDst;
      Vector2D PedgDst_Pxi = edgVct / edgDst;

      float PvorPtn_PedgDst = (alpha_ / (alpha_ + obsDst)) * std::pow((obsDst - vor_obs_dmax_) / vor_obs_dmax_, 2) * (obsDst / std::pow(obsDst + edgDst, 2));

      float PvorPtn_PobsDst = (alpha_ / (alpha_ + obsDst)) * (edgDst / (obsDst + edgDst)) * ((obsDst - vor_obs_dmax_) / std::pow(vor_obs_dmax_, 2)) * (-(obsDst - vor_obs_dmax_) / (alpha_ + obsDst) - (obsDst - vor_obs_dmax_) / (obsDst + edgDst) + 2);

      gradient = weight_voronoi_ * (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi);

      return gradient;
    }
    return gradient;
  }
  return gradient;
}

//###################################################
//                                     CURVATURE TERM
//###################################################
Vector2D Smoother::curvatureTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2)
{
  //use curvature square as cost function for curvature term
  Vector2D gradient;
  // the vectors between the nodes
  Vector2D Dxi = xi - xim1;
  Vector2D Dxip1 = xip1 - xi;
  Vector2D Dxip2 = xip2 - xip1;
  Vector2D Dxim1 = xim1 - xim2;

  // the distance of the vectors
  float norm_Dxi = Dxi.length();
  float norm_Dxip1 = Dxip1.length();
  float norm_Dxim1 = Dxim1.length();
  float norm_Dxip2 = Dxip2.length();
  // ensure that the absolute values are not null
  if (norm_Dxi > 0 && norm_Dxim1 > 0 && norm_Dxip1 > 0)
  {
    // the angular change at the node
    float Dphi = std::acos(Helper::clamp(Dxi.dot(Dxip1) / (norm_Dxi * norm_Dxip1), -1, 1));
    float Dphip1 = std::acos(Helper::clamp(Dxip1.dot(Dxip2) / (norm_Dxip1 * norm_Dxip2), -1, 1));
    float Dphim1 = std::acos(Helper::clamp(Dxim1.dot(Dxi) / (norm_Dxim1 * norm_Dxi), -1, 1));

    float absDxi1Inv = 1 / norm_Dxi;
    float absDxim1Inv = 1 / norm_Dxim1;
    float absDxip1Inv = 1 / norm_Dxip1;

    float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
    float PDphim1_PcosDphim1 = -1 / std::sqrt(1 - std::pow(std::cos(Dphim1), 2));
    float PDphip1_PcosDphip1 = -1 / std::sqrt(1 - std::pow(std::cos(Dphip1), 2));

    float ui = absDxi1Inv * PDphi_PcosDphi;
    float uim1 = absDxim1Inv * PDphim1_PcosDphim1;
    float uip1 = absDxip1Inv * PDphip1_PcosDphip1;

    Vector2D PcosDphi_Pxi = -xi.ort(-xip1) / (norm_Dxi * norm_Dxip1) - (-xip1).ort(xi) / (norm_Dxi * norm_Dxip1);
    Vector2D PcosDphim1_Pxi = xim1.ort(-xi) / (norm_Dxim1 * norm_Dxi);
    Vector2D PcosDphip1_Pxi = (-xip2).ort(xip1) / (norm_Dxip1 * norm_Dxip2);

    Vector2D Pcurvature_m_Pxi = uim1 * PcosDphim1_Pxi;
    Vector2D Pcurvature_i_Pxi = ui * PcosDphi_Pxi - Dphi / std::pow(norm_Dxi, 2);
    Vector2D Pcurvature_p_Pxi = uip1 * PcosDphip1_Pxi + Dphip1 / std::pow(norm_Dxip1, 2);

    // calculate the gradient
    gradient = weight_curvature_ * (2 * Pcurvature_m_Pxi + 2 * Pcurvature_i_Pxi + 2 * Pcurvature_p_Pxi);

    if (std::isnan(gradient.getX()) || std::isnan(gradient.getY()))
    {
      DLOG(WARNING) << "nan values in curvature term";
      Vector2D zeros;
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
    DLOG(WARNING) << "abs values not larger than 0";
    if (norm_Dxi <= 0)
    {
      DLOG(INFO) << "absDxi value is: " << norm_Dxi
                 << " Dxi x: " << Dxi.getX()
                 << " Dxi y: " << Dxi.getY()
                 << " xi x: " << xi.getX()
                 << " xi y: " << xi.getY()
                 << " xim1 x: " << xim1.getX()
                 << " xim1 y: " << xim1.getY();
      if (xi == xim1)
      {
        DLOG(WARNING) << "xi is equal to xim1!!!!";
      }
    }
    if (norm_Dxip1 <= 0)
    {
      DLOG(INFO) << "absDxip1 value is: " << norm_Dxip1
                 << " Dxip1 x: " << Dxip1.getX()
                 << " Dxip1 y: " << Dxip1.getY()
                 << " xip1 x: " << xip1.getX()
                 << " xip1 y: " << xip1.getY()
                 << " xi x: " << xi.getX()
                 << " xi y: " << xi.getY();
      if (xi == xip1)
      {
        DLOG(WARNING) << "xi is equal to xip1!!!!";
      }
    }
    Vector2D zeros;
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2)
{
  //this is correct, see https://zhuanlan.zhihu.com/p/118666410
  return weight_smoothness_ * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}
Vector2D Smoother::PathLengthTerm(Vector2D xim1, Vector2D xi, Vector2D xip1)
{
  return weight_length_ * 2 * (2 * xi - xim1 - xip1);
}
int Smoother::PreprocessPath()
{
  //remove some duplicates
  for (uint index = 0; index < path_.size(); ++index)
  {
    // DLOG(INFO) << "path point is x: " << path_[index].getX() << " y: " << path_[index].getY() << " t: " << path_[index].getT();
    if (index != path_.size() - 1)
    {
      if (path_[index] == path_[index + 1])
      {
        DLOG(WARNING) << "path point: " << index << " is equal to next point!!";
        continue;
      }
    }
    preprocessed_path_.emplace_back(path_[index]);
  }
  return 1;
}
