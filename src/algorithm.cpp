#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes
{
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D *lhs, const Node3D *rhs) const
  {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D *lhs, const Node2D *rhs) const
  {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Node3D *Algorithm::HybridAStar(Node3D &start,
                               const Node3D &goal,
                               Node3D *nodes3D,
                               Node2D *nodes2D,
                               int width,
                               int height,
                               std::shared_ptr<CollisionDetection> &configurationSpace,
                               float *dubinsLookup,
                               std::shared_ptr<Visualize> &visualization)
{
  DLOG(INFO) << "Hybrid A star start!!";
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = params_.reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on params_.max_iterations
  int iterations = 0;

  // VISUALIZATION DELAY
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D *, boost::heap::compare<CompareNodes>> priorityQueue;
  priorityQueue openlist;

  // update h value
  UpdateHeuristic(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
  //this N is for analytic expansion
  int N = start.getH() / 2;
  // mark start as open
  start.open();
  // push on priority queue aka open list
  openlist.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D *nPred;
  Node3D *nSucc;

  // float max = 0.f;

  // continue until O empty
  while (!openlist.empty())
  {

    // pop node with lowest cost from priority queue
    nPred = openlist.top();
    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;

    // RViz visualization
    if (params_.visualization)
    {
      visualization->publishNode3DPoses(*nPred);
      visualization->publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed())
    {
      // pop node from the open list and start with a fresh node
      openlist.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen())
    {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      openlist.pop();

      // _________
      // GOAL TEST
      if (iterations > params_.max_iterations)
      {
        DLOG(INFO) << "max iterations reached!!!";
        return nPred;
      }
      else if (*nPred == goal || nPred->IsCloseEnough(goal, params_.epsilon, 2 * M_PI / params_.headings))
      {
        DLOG(INFO) << "Goal reached!!!";
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else
      {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        if (iterations == N)
        {
          DLOG(INFO) << "Start Analytic Expansion!!";
          N = N + nPred->getH() / 2;
          nSucc = AnalyticExpansions(*nPred, goal, configurationSpace);

          if (nSucc != nullptr && (*nSucc == goal || nSucc->IsCloseEnough(goal, params_.epsilon, 2 * M_PI / params_.headings)))
          {
            //DEBUG
            // DLOG(INFO) << "max diff " << max ;
            return nSucc;
          }
        }
        // if (nPred->IsInRange(goal, params_.dubins_shot_distance) && nPred->getPrim() < 3)
        // {
        //   nSucc = AnalyticExpansions(*nPred, goal, configurationSpace);

        //   if (nSucc != nullptr && (*nSucc == goal || nSucc->IsCloseEnough(goal, params_.epsilon, 2 * M_PI / params_.headings)))
        //   {
        //     //DEBUG
        //     // DLOG(INFO) << "max diff " << max ;
        //     return nSucc;
        //   }
        // }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++)
        {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && configurationSpace->isTraversable(nSucc))
          {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc)
            {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc)
              {

                // calculate H value
                UpdateHeuristic(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + params_.tie_breaker)
                {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + params_.tie_breaker)
                {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc)
                {
                  DLOG(INFO) << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                openlist.push(&nodes3D[iSucc]);
                delete nSucc;
              }
              else
              {
                delete nSucc;
              }
            }
            else
            {
              delete nSucc;
            }
          }
          else
          {
            delete nSucc;
          }
        }
      }
    }
  }

  if (openlist.empty())
  {
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float Algorithm::AStar(Node2D &start,
                       Node2D &goal,
                       Node2D *nodes2D,
                       int width,
                       int height,
                       std::shared_ptr<CollisionDetection> &configurationSpace,
                       std::shared_ptr<Visualize> &visualization)
{

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D *, boost::heap::compare<CompareNodes>> openlist;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  openlist.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!openlist.empty())
  {
    // pop node with lowest cost from priority queue
    nPred = openlist.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      openlist.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (params_.visualization2D)
      {
        visualization->publishNode2DPoses(*nPred);
        visualization->publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      openlist.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        std::vector<Node2D *> successor_vec = nPred->CreateSuccessor(params_.possible_direction);
        for (uint i = 0; i < successor_vec.size(); ++i)
        {
          // create possible successor
          nSucc = successor_vec[i];
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) && configurationSpace->isTraversable(nSucc) && !nodes2D[iSucc].isClosed())
          {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG())
            {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              openlist.push(&nodes2D[iSucc]);
              delete nSucc;
            }
            else
            {
              delete nSucc;
            }
          }
          else
          {
            delete nSucc;
          }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void Algorithm::UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D, float *dubinsLookup, int width, int height, std::shared_ptr<CollisionDetection> &configurationSpace, std::shared_ptr<Visualize> &visualization)
{
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (params_.dubins)
  {

    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.getX() - (int)start.getX());
    //    int uY = std::abs((int)goal.getY() - (int)start.getY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (params_.dubinsLookup && uX < params_.dubinsWidth - 1 && uY < params_.dubinsWidth - 1) {
    //      int X = (int)goal.getX() - (int)start.getX();
    //      int Y = (int)goal.getY() - (int)start.getY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / params_.deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / params_.deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / params_.deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / params_.deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / params_.deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / params_.deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / params_.deltaHeadingRad);
    //        h1 = (int)(goal.getT() / params_.deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * params_.dubinsWidth * params_.headings * params_.headings
    //                                + uY *  params_.headings * params_.headings
    //                                + h0 * params_.headings
    //                                + h1];
    //    } else {

    /*if ( std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
    //      // start
    //      double q0[] = { start.getX(), start.getY(), start.getT()};
    //      // goal
    //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, params_.min_turning_radius, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);

    ompl::base::DubinsStateSpace dubinsPath(params_.min_turning_radius);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use a
  if (params_.reverse && !params_.dubins)
  {
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(params_.min_turning_radius);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    DLOG(INFO) << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 ;
  }

  // if two_D heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (params_.two_D && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered())
  {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d AStar and return the cost of the cheapest path for that node
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(AStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization));
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    DLOG(INFO) << "calculated 2D Heuristic in ms: " << d * 1000 ;
  }

  if (params_.two_D)
  {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D *Algorithm::AnalyticExpansions(Node3D &start, const Node3D &goal, std::shared_ptr<CollisionDetection> &configurationSpace)
{
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, params_.min_turning_radius, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D *dubinsNodes = new Node3D[(int)(length / params_.dubins_step_size) + 1];

  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (configurationSpace->isTraversable(&dubinsNodes[i]))
    {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        DLOG(INFO) << "looping shot";
      }

      x += params_.dubins_step_size;
      i++;
    }
    else
    {
      DLOG(INFO) << "Dubins shot collided, discarding the path";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

  DLOG(INFO) << "Dubins shot connected, returning the path";
  return &dubinsNodes[i - 1];
}
