#include "smoother.h"
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
  this->width = voronoi.getSizeX();
  this->height = voronoi.getSizeY();
  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // the maximum iterations for the gd smoother
  int maxIterations = 500;
  // the length of the path in number of nodes
  int pathLength = 0;

  // path objects with all nodes oldPath the original, newPath the resulting smoothed path
  pathLength = path.size();
  std::vector<Node3D> newPath = path;

  // descent along the gradient until the maximum number of iterations has been reached
  float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;

  while (iterations < maxIterations) {

    // choose the first three nodes of the path
    for (int i = 2; i < pathLength - 2; ++i) {

      Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
      Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
      Vector2D xi(newPath[i].getX(), newPath[i].getY());
      Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
      Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
      Vector2D correction;


      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      if (isCusp(newPath, i)) { continue; }

      correction = correction - obstacleTerm(xi);
      if (!isOnGrid(xi + correction)) { continue; }

      correction = correction - voronoiTerm(xi);
      if (!isOnGrid(xi + correction))
      {
        continue;
      }
      // ensure that it is on the grid
      correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid
      correction = correction - curvatureTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid

      xi = xi + alpha_ * correction / totalWeight;
      newPath[i].setX(xi.getX());
      newPath[i].setY(xi.getY());
      Vector2D Dxi = xi - xim1;
      newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));

    }

    iterations++;
  }

  path = newPath;
}

void Smoother::tracePath(const Node3D* node, int i, std::vector<Node3D> path) {
  if (node == nullptr) {
    this->path = path;
    return;
  }

  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}

//###################################################
//                                      OBSTACLE TERM
//###################################################
Vector2D Smoother::obstacleTerm(Vector2D xi) {
  Vector2D gradient;
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi_.getDistance(xi.getX(), xi.getY());
  // the vector determining where the obstacle is
  int x = (int)xi.getX();
  int y = (int)xi.getY();
  // if the node is within the map
  if (x < width && x >= 0 && y < height && y >= 0) {
    Vector2D obsVct(xi.getX() - voronoi_.data[(int)xi.getX()][(int)xi.getY()].obstX,
                    xi.getY() - voronoi_.data[(int)xi.getX()][(int)xi.getY()].obstY);

    // the closest obstacle is closer than desired correct the path for that
    if (obsDst < obsDMax && obsDst > 1e-6)
    {
      return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
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

  if (obsDst < vorObsDMax && obsDst > 1e-6)
  {
    //calculate the distance to the closest GVD edge from the current node
    // the node is away from the optimal free space area
    if (edgDst > 0)
    {
      // equations can be found in paper "Path Planning for autonomous vehicle in unknown semi-structured environments"
      Vector2D PobsDst_Pxi = obsVct / obsDst;
      Vector2D PedgDst_Pxi = edgVct / edgDst;

      float PvorPtn_PedgDst = (alpha_ / (alpha_ + obsDst)) * std::pow((obsDst - vorObsDMax) / vorObsDMax, 2) * (obsDst / std::pow(obsDst + edgDst, 2));

      float PvorPtn_PobsDst = (alpha_ / (alpha_ + obsDst)) * (edgDst / (obsDst + edgDst)) * ((obsDst - vorObsDMax) / std::pow(vorObsDMax, 2)) * (-(obsDst - vorObsDMax) / (alpha_ + obsDst) - (obsDst - vorObsDMax) / (obsDst + edgDst) + 2);

      gradient = wVoronoi * (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi);

      return gradient;
    }
    return gradient;
  }
  return gradient;
}

//###################################################
//                                     CURVATURE TERM
//###################################################
Vector2D Smoother::curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1) {
  Vector2D gradient;
  // the vectors between the nodes
  Vector2D Dxi = xi - xim1;
  Vector2D Dxip1 = xip1 - xi;
  // orthogonal complements vector
  Vector2D p1, p2;

  // the distance of the vectors
  float absDxi = Dxi.length();
  float absDxip1 = Dxip1.length();

  // ensure that the absolute values are not null
  if (absDxi > 0 && absDxip1 > 0) {
    // the angular change at the node
    float Dphi = std::acos(Helper::clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1), -1, 1));
    float kappa = Dphi / absDxi;

    // if the curvature is smaller then the maximum do nothing
    if (kappa <= kappaMax) {
      Vector2D zeros;
      return zeros;
    } else {
      float absDxi1Inv = 1 / absDxi;
      float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
      float u = -absDxi1Inv * PDphi_PcosDphi;
      // calculate the p1 and p2 terms
      p1 = xi.ort(-xip1) / (absDxi * absDxip1);
      p2 = -xip1.ort(xi) / (absDxi * absDxip1);
      // calculate the last terms
      float s = Dphi / (absDxi * absDxi);
      Vector2D ones(1, 1);
      Vector2D ki = u * (-p1 - p2) - (s * ones);
      Vector2D kim1 = u * p2 - (s * ones);
      Vector2D kip1 = u * p1;

      // calculate the gradient
      gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

      if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
        std::cout << "nan values in curvature term" << std::endl;
        Vector2D zeros;
        return zeros;
      }
      // return gradient of 0
      else {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else {
    std::cout << "abs values not larger than 0" << std::endl;
    Vector2D zeros;
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
  //this is correct, see https://zhuanlan.zhihu.com/p/118666410
  return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

