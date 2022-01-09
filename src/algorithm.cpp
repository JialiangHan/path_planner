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
    return lhs->GetC() > rhs->GetC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D *lhs, const Node2D *rhs) const
  {
    return lhs->GetC() > rhs->GetC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Path Algorithm::HybridAStar(Node3D &start,
                            Node3D &goal,
                            Node3D *nodes3D,
                            Node2D *nodes2D,
                            int width,
                            int height,
                            std::shared_ptr<CollisionDetection> &configurationSpace,
                            float *dubinsLookup,
                            std::shared_ptr<Visualize> &visualization)
{
  DLOG(INFO) << "Hybrid A star start!!";
  start_ = start;
  goal_ = goal;
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
  int N = start.GetH() / 2;
  // mark start as open
  start.open();
  // push on priority queue aka open list
  openlist.push(&start);
  iPred = start.setIdx(width, height, (float)2 * M_PI / params_.headings);
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
    iPred = nPred->setIdx(width, height, (float)2 * M_PI / params_.headings);
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
        DLOG(INFO) << "max iterations reached!!!, current iterations is :" << iterations;
        // return nPred;
        TracePath(nPred);
        return path_;
      }
      else if (nPred->IsCloseEnough(goal, params_.epsilon, 2 * M_PI / params_.headings))
      {
        DLOG(INFO) << "Goal reached!!!";
        // return nPred;
        TracePath(nPred);
        return path_;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else
      {
        // _______________________
        // SEARCH WITH DUBINS SHOT

        if (iterations == N)
        {
          DLOG(INFO) << "Start Analytic Expansion, every " << N << "th iterations";
          N = N + nPred->GetH() / 2;
          Path analytical_path = AnalyticExpansions(*nPred, goal, configurationSpace);

          if (analytical_path.size() != 0)
          {
            //DEBUG
            DLOG(INFO) << "Found path through anallytical expansion";
            TracePath(nPred);
            path_.insert(path_.end(), analytical_path.begin(), analytical_path.end());
            return path_;
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++)
        {
          // create possible successor
          // nSucc = nPred->createSuccessor(i);
          nSucc = CreateSuccessor(nPred, i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height, (float)2 * M_PI / params_.headings);

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && configurationSpace->isTraversable(nSucc))
          {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc)
            {

              // calculate new G value
              nSucc->updateG(params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
              newG = nSucc->GetG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].GetG() || iPred == iSucc)
              {

                // calculate H value
                UpdateHeuristic(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->GetC() > nPred->GetC() + params_.tie_breaker)
                {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->GetC() <= nPred->GetC() + params_.tie_breaker)
                {
                  nSucc->SetPred(nPred->GetPred());
                }

                if (nSucc->GetPred() == nSucc)
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

  return path_;
}

//###################################################
//                                        2D A*
//###################################################
float Algorithm::AStar(Node2D &start, Node2D &goal, Node2D *nodes2D, int width, int height, std::shared_ptr<CollisionDetection> &configurationSpace, std::shared_ptr<Visualize> &visualization)
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
        return nPred->GetG();
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
            newG = nSucc->GetG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].GetG())
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
  if (params_.dubins_flag)
  {

    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.GetX() - (int)start.GetX());
    //    int uY = std::abs((int)goal.GetY() - (int)start.GetY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (params_.dubinsLookup && uX < params_.dubinsWidth - 1 && uY < params_.dubinsWidth - 1) {
    //      int X = (int)goal.GetX() - (int)start.GetX();
    //      int Y = (int)goal.GetY() - (int)start.GetY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(Utility::RadToZeroTo2P(M_PI_2 - t) / params_.deltaHeadingRad);
    //        h1 = (int)(Utility::RadToZeroTo2P(M_PI_2 - goal.GetT()) / params_.deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(Utility::RadToZeroTo2P(M_PI_2 - t) / params_.deltaHeadingRad);
    //        h1 = (int)(Utility::RadToZeroTo2P(M_PI_2 - goal.GetT()) / params_.deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(Utility::RadToZeroTo2P(M_PI - t) / params_.deltaHeadingRad);
    //        h1 = (int)(Utility::RadToZeroTo2P(M_PI - goal.GetT()) / params_.deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / params_.deltaHeadingRad);
    //        h1 = (int)(goal.GetT() / params_.deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * params_.dubinsWidth * params_.headings * params_.headings
    //                                + uY *  params_.headings * params_.headings
    //                                + h0 * params_.headings
    //                                + h1];
    //    } else {

    /*if ( std::abs(start.GetX() - goal.GetX()) >= 10 && std::abs(start.GetY() - goal.GetY()) >= 10)*/
    //      // start
    //      double q0[] = { start.GetX(), start.GetY(), start.GetT()};
    //      // goal
    //      double q1[] = { goal.GetX(), goal.GetY(), goal.GetT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, params_.min_turning_radius, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);

    ompl::base::DubinsStateSpace dubinsPath(params_.min_turning_radius);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.GetX(), start.GetY());
    dbStart->setYaw(start.GetT());
    dbEnd->setXY(goal.GetX(), goal.GetY());
    dbEnd->setYaw(goal.GetT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use a
  if (params_.reverse && !params_.dubins_flag)
  {
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(params_.min_turning_radius);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.GetX(), start.GetY());
    rsStart->setYaw(start.GetT());
    rsEnd->setXY(goal.GetX(), goal.GetY());
    rsEnd->setYaw(goal.GetT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    DLOG(INFO) << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 ;
  }

  // if two_D heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (params_.two_D && !nodes2D[(int)start.GetY() * width + (int)start.GetX()].isDiscovered())
  {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.GetX(), start.GetY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.GetX(), goal.GetY(), 0, 0, nullptr);
    // run 2d AStar and return the cost of the cheapest path for that node
    nodes2D[(int)start.GetY() * width + (int)start.GetX()].setG(AStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization));
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    DLOG(INFO) << "calculated 2D Heuristic in ms: " << d * 1000 ;
  }

  if (params_.two_D)
  {
    // offset for same node in cell
    twoDoffset = sqrt(((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) * ((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) +
                      ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())) * ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())));
    twoDCost = nodes2D[(int)start.GetY() * width + (int)start.GetX()].GetG() - twoDoffset;
  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

//###################################################
//                                    ANALYTICAL EXPANSION
//###################################################
Path Algorithm::AnalyticExpansions(const Node3D &start, Node3D &goal, std::shared_ptr<CollisionDetection> &configurationSpace)
{
  Path path_vec;
  int i = 0;
  float x = 0.f;
  if (params_.curve_type == CurveType::dubins)
  {

    // start
    double q0[] = {start.GetX(), start.GetY(), start.GetT()};
    // goal
    double q1[] = {goal.GetX(), goal.GetY(), goal.GetT()};
    // initialize the path
    DubinsPath path;
    // calculate the path
    dubins_init(q0, q1, params_.min_turning_radius, &path);

    float length = dubins_path_length(&path);

    while (x < length)
    {
      double q[3];
      dubins_path_sample(&path, x, q);
      Node3D *node3d;
      node3d->setX(q[0]);
      node3d->setY(q[1]);
      node3d->setT(Utility::RadToZeroTo2P(q[2]));

      // collision check
      if (configurationSpace->isTraversable(node3d))
      {

        // set the predecessor to the previous step
        if (i > 0)
        {
          node3d->SetPred(&path_vec[i - 1]);
        }
        else
        {
          node3d->SetPred(&start);
        }

        if (node3d == node3d->GetPred())
        {
          DLOG(INFO) << "looping shot";
        }
        path_vec.emplace_back(*node3d);
        x += params_.curve_step_size;
        i++;
      }
      else
      {
        DLOG(INFO) << "Dubins shot collided, discarding the path";
        path_vec.clear();
        break;
        // return nullptr;
      }
    }
  }
  else if (params_.curve_type == CurveType::cubic_bezier)
  {
    Eigen::Vector3d vector3d_start = Utility::ConvertNode3DToVector3d(start);
    // DLOG(INFO) << "start point is " << vector3d_start.x() << " " << vector3d_start.y();
    Eigen::Vector3d vector3d_goal = Utility::ConvertNode3DToVector3d(goal);
    int map_width, map_height;
    map_width = configurationSpace->grid->info.width;
    map_height = configurationSpace->grid->info.height;
    CubicBezier::CubicBezier cubic_bezier(vector3d_start, vector3d_goal, map_width, map_height);
    float length = cubic_bezier.GetLength();
    i = 0;
    x = 0;
    path_vec.clear();
    while (x < length)
    {
      // DLOG(INFO) << i << "th iteration";
      Node3D *node3d;
      Node3D current = Utility::ConvertVector3dToNode3D(cubic_bezier.GetValueAt(x / length));
      float curvature = cubic_bezier.GetCurvatureAt(x / length);
      node3d = &current;
      node3d->setT(cubic_bezier.GetAngleAt(x / length));

      // collision check
      if (configurationSpace->isTraversable(node3d))
      {
        // // set the predecessor to the previous step
        // if (i > 0)
        // {
        //   node3d->SetPred(&path_vec[i - 1]);
        // }
        // else
        // {
        //   node3d->SetPred(nullptr);
        // }
        // if (node3d->GetPred() != nullptr)
        // {
        //   DLOG(INFO) << "current node is " << node3d->GetX() << " " << node3d->GetY() << " "
        //              << "its pred is " << node3d->GetPred()->GetX() << " " << node3d->GetPred()->GetY();
        // }

        // if (node3d == node3d->GetPred())
        // {
        //   DLOG(INFO) << "looping shot";
        // }
        if (curvature <= 1 / params_.min_turning_radius)
        {
          path_vec.emplace_back(*node3d);
          x += params_.curve_step_size;
          i++;
        }
        else
        {
          DLOG(INFO) << "cubic bezier curvature greater than 1/min_turning_radius, discarding the path";
          path_vec.clear();
          break;
        }
      }
      else
      {
        DLOG(INFO) << "cubic bezier collided, discarding the path";
        path_vec.clear();
        break;
        // return nullptr;
      }
    }
    // goal.SetPred(&path_vec.back());
    // DLOG(INFO) << "goal point is " << vector3d_goal.x() << " " << vector3d_goal.y();
  }

  //Some kind of collision detection for all curves
  if (path_vec.size() != 0)
  {
    path_vec.emplace_back(goal);
    DLOG(INFO) << "Analytical expansion connected, returning the path";
  }
  return path_vec;
}

Node3D *Algorithm::CreateSuccessor(const Node3D *pred, const int &i)
{

  const float dy[] = {0, -0.0415893, 0.0415893};
  const float dx[] = {0.7068582, 0.705224, 0.705224};
  const float dt[] = {0, 0.1178097, -0.1178097};
  float xSucc, ySucc, tSucc;

  // calculate successor positions forward
  if (i < 3)
  {
    xSucc = pred->GetX() + dx[i] * cos(pred->GetT()) - dy[i] * sin(pred->GetT());
    ySucc = pred->GetY() + dx[i] * sin(pred->GetT()) + dy[i] * cos(pred->GetT());
    tSucc = Utility::RadToZeroTo2P(pred->GetT() + dt[i]);
  }
  // backwards
  else
  {
    xSucc = pred->GetX() - dx[i - 3] * cos(pred->GetT()) - dy[i - 3] * sin(pred->GetT());
    ySucc = pred->GetY() - dx[i - 3] * sin(pred->GetT()) + dy[i - 3] * cos(pred->GetT());
    tSucc = Utility::RadToZeroTo2P(pred->GetT() - dt[i - 3]);
  }
  return new Node3D(xSucc, ySucc, tSucc, pred->GetG(), 0, pred, i);
}

void Algorithm::TracePath(const Node3D *node)
{
  path_.clear();
  while (node != nullptr)
  {

    path_.emplace_back(*node);
    if (*node == start_)
    {
      break;
    }
    // DLOG(INFO) << "current node is " << node->GetX() << " " << node->GetY() << " and its pred is " << node->GetPred()->GetX() << " " << node->GetPred()->GetY();
    node = node->GetPred();
  }
  std::reverse(path_.begin(), path_.end());
}