#include "algorithm.h"

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
                            const std::shared_ptr<LookupTable> &lookup_table_ptr,
                            std::shared_ptr<Visualize> &visualization)
{

  DLOG(INFO) << "Hybrid A star start!!";
  start_ = start;
  goal_ = goal;
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  // int dir = params_.reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on params_.max_iterations
  int iterations = 0;
  int analytical_expansion_counter = 0;
  // VISUALIZATION DELAY
  ros::Duration d(0.003);
  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D *, boost::heap::compare<CompareNodes>> priorityQueue;
  priorityQueue openlist;
  // update h value
  UpdateHeuristic(start, goal, nodes2D, lookup_table_ptr, width, height, configurationSpace, visualization);
  //this N is for analytic expansion
  int N = start.GetH();
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
      // DLOG(INFO) << "nPred is already closesd!";
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
      else if (Utility::IsCloseEnough(*nPred, goal, params_.epsilon, 2 * M_PI / params_.headings))
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
        //analytical expansion
        // DLOG(INFO) << "analytical_expansion_counter is " << analytical_expansion_counter << " N is " << N;
        if (analytical_expansion_counter == N)
        {
          // DLOG(INFO) << "Start Analytic Expansion, every " << N << "th iterations";
          N = nPred->GetH();
          analytical_expansion_counter = 0;
          Path analytical_path = AnalyticExpansions(*nPred, goal, configurationSpace);
          if (analytical_path.size() != 0)
          {
            DLOG(INFO) << "Found path through anallytical expansion";
            TracePath(nPred);
            path_.insert(path_.end(), analytical_path.begin(), analytical_path.end());
            return path_;
          }
        }
        else
        {
          analytical_expansion_counter++;
        }
        std::vector<Node3D *> successor_vec = CreateSuccessor(*nPred, params_.possible_direction);
        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (const auto &node : successor_vec)
        {
          // create possible successor
          // nSucc = nPred->createSuccessor(i);
          nSucc = node;
          // set index of the successor
          iSucc = nSucc->setIdx(width, height, (float)2 * M_PI / params_.headings);
          // ensure successor is on grid and traversable
          if (configurationSpace->IsTraversable(nSucc))
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
                UpdateHeuristic(*nSucc, goal, nodes2D, lookup_table_ptr, width, height, configurationSpace, visualization);
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
        // for (int i = 0; i < dir; i++)
        // {
        //   // create possible successor
        //   // nSucc = nPred->createSuccessor(i);
        //   nSucc = CreateSuccessor(nPred, i);
        //   // set index of the successor
        //   iSucc = nSucc->setIdx(width, height, (float)2 * M_PI / params_.headings);
        //   // ensure successor is on grid and traversable
        //   if (configurationSpace->IsTraversable(nSucc))
        //   {
        //     // ensure successor is not on closed list or it has the same index as the predecessor
        //     if (!nodes3D[iSucc].isClosed() || iPred == iSucc)
        //     {
        //       // calculate new G value
        //       nSucc->updateG(params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
        //       newG = nSucc->GetG();
        //       // if successor not on open list or found a shorter way to the cell
        //       if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].GetG() || iPred == iSucc)
        //       {
        //         // calculate H value
        //         UpdateHeuristic(*nSucc, goal, nodes2D, lookup_table_ptr, width, height, configurationSpace, visualization);
        //         // if the successor is in the same cell but the C value is larger
        //         if (iPred == iSucc && nSucc->GetC() > nPred->GetC() + params_.tie_breaker)
        //         {
        //           delete nSucc;
        //           continue;
        //         }
        //         // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
        //         else if (iPred == iSucc && nSucc->GetC() <= nPred->GetC() + params_.tie_breaker)
        //         {
        //           nSucc->SetPred(nPred->GetPred());
        //         }
        //         if (nSucc->GetPred() == nSucc)
        //         {
        //           DLOG(INFO) << "looping";
        //         }
        //         // put successor on open list
        //         nSucc->open();
        //         nodes3D[iSucc] = *nSucc;
        //         openlist.push(&nodes3D[iSucc]);
        //         delete nSucc;
        //       }
        //       else
        //       {
        //         delete nSucc;
        //       }
        //     }
        //     else
        //     {
        //       delete nSucc;
        //     }
        //   }
        //   else
        //   {
        //     delete nSucc;
        //   }
        // }
      }
    }
  }
  DLOG(INFO) << "open list is empty, end hybrid a star.";
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
  start.UpdateHeuristic(goal);
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
        std::vector<Node2D *> successor_vec = CreateSuccessor(*nPred, params_.possible_direction);
        for (uint i = 0; i < successor_vec.size(); ++i)
        {
          // create possible successor
          nSucc = successor_vec[i];
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (configurationSpace->IsTraversable(nSucc) && !nodes2D[iSucc].isClosed())
          {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->GetG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].GetG())
            {
              // calculate the H value
              nSucc->UpdateHeuristic(goal);
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
void Algorithm::UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D, const std::shared_ptr<LookupTable> &lookup_table_ptr, int width, int height, std::shared_ptr<CollisionDetection> &configurationSpace, std::shared_ptr<Visualize> &visualization)
{
  float curve_cost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  // translate and rotate
  Node3D goal_rotated_translated;
  goal_rotated_translated.setX(abs(goal.GetX() - start.GetX()));
  goal_rotated_translated.setY(abs(goal.GetY() - start.GetY()));
  goal_rotated_translated.setT(Utility::RadNormalization(goal.GetT() - start.GetT()));

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (params_.reverse == false)
  {
    curve_cost = lookup_table_ptr->GetDubinsCost(goal_rotated_translated);
    // DLOG(INFO) << "dubins cost is " << curve_cost;
  }

  // if reversing is active use a
  if (params_.reverse == true)
  {
    curve_cost = lookup_table_ptr->GetReedsSheppCost(goal_rotated_translated);
    // DLOG(INFO) << "cubic bezier cost is " << curve_cost;
  }

  // unconstrained with obstacles
  if (!nodes2D[(int)start.GetY() * width + (int)start.GetX()].isDiscovered())
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

  // offset for same node in cell
  twoDoffset = sqrt(((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) * ((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) +
                    ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())) * ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())));
  twoDCost = nodes2D[(int)start.GetY() * width + (int)start.GetX()].GetG() - twoDoffset;

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(curve_cost, twoDCost));
}

//###################################################
//                                    ANALYTICAL EXPANSION
//###################################################
Path Algorithm::AnalyticExpansions(const Node3D &start, Node3D &goal, std::shared_ptr<CollisionDetection> &configurationSpace)
{
  Path path_vec;
  int i = 0;
  float x = 0.f;
  // if (params_.curve_type == 0)
  // {

  //   // start
  //   double q0[] = {start.GetX(), start.GetY(), start.GetT()};
  //   // goal
  //   double q1[] = {goal.GetX(), goal.GetY(), goal.GetT()};
  //   // initialize the path
  //   DubinsPath path;
  //   // calculate the path
  //   dubins_init(q0, q1, params_.min_turning_radius, &path);

  //   float length = dubins_path_length(&path);

  //   while (x < length)
  //   {
  //     double q[3];
  //     dubins_path_sample(&path, x, q);
  //     Node3D *node3d;
  //     node3d->setX(q[0]);
  //     node3d->setY(q[1]);
  //     node3d->setT(Utility::RadToZeroTo2P(q[2]));

  //     // collision check
  //     if (configurationSpace->IsTraversable(node3d))
  //     {

  //       // // set the predecessor to the previous step
  //       // if (i > 0)
  //       // {
  //       //   node3d->SetPred(&path_vec[i - 1]);
  //       // }
  //       // else
  //       // {
  //       //   node3d->SetPred(&start);
  //       // }

  //       // if (node3d == node3d->GetPred())
  //       // {
  //       //   DLOG(INFO) << "looping shot";
  //       // }
  //       path_vec.emplace_back(*node3d);
  //       x += params_.curve_step_size;
  //       i++;
  //     }
  //     else
  //     {
  //       DLOG(INFO) << "Dubins shot collided, discarding the path";
  //       path_vec.clear();
  //       break;
  //       // return nullptr;
  //     }
  //   }
  // }
  // else if (params_.curve_type == 1)
  // {
  Eigen::Vector3d vector3d_start = Utility::ConvertNode3DToVector3d(start);
  // DLOG(INFO) << "start point is " << vector3d_start.x() << " " << vector3d_start.y();
  Eigen::Vector3d vector3d_goal = Utility::ConvertNode3DToVector3d(goal);
  int map_width, map_height;
  map_width = configurationSpace->grid_ptr_->info.width;
  map_height = configurationSpace->grid_ptr_->info.height;
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
    if (configurationSpace->IsTraversable(node3d))
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
        // DLOG(INFO) << "cubic bezier curvature greater than 1/min_turning_radius, discarding the path";
        path_vec.clear();
        break;
      }
    }
    else
    {
      // DLOG(INFO) << "cubic bezier collided, discarding the path";
      path_vec.clear();
      break;
      // return nullptr;
    }
    // }
    // goal.SetPred(&path_vec.back());
    // DLOG(INFO) << "goal point is " << vector3d_goal.x() << " " << vector3d_goal.y();
  }

  //Some kind of collision detection for all curves
  if (path_vec.size() != 0)
  {
    path_vec.emplace_back(goal);
    // std::string out = params_.reverse == true ? "cubic bezier" : "dubins";
    // DLOG(INFO) << "Analytical expansion connected, returning " << out << " path";
  }
  return path_vec;
}
//TODO change to a vector and add one more successor similar to RRT
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

std::vector<Node3D *> Algorithm::CreateSuccessor(const Node3D &pred, const int &possible_dir)
{
  std::vector<Node3D *> out;
  const float dy[] = {0, -0.0415893, 0.0415893};
  const float dx[] = {0.7068582, 0.705224, 0.705224};
  const float dt[] = {0, 0.1178097, -0.1178097};
  for (int i = 0; i < possible_dir; ++i)
  {
    float xSucc, ySucc, tSucc;
    // calculate successor positions forward
    if (i < 3)
    {
      xSucc = pred.GetX() + dx[i] * cos(pred.GetT()) - dy[i] * sin(pred.GetT());
      ySucc = pred.GetY() + dx[i] * sin(pred.GetT()) + dy[i] * cos(pred.GetT());
      tSucc = Utility::RadToZeroTo2P(pred.GetT() + dt[i]);
    }
    // backwards
    else
    {
      xSucc = pred.GetX() - dx[i - 3] * cos(pred.GetT()) - dy[i - 3] * sin(pred.GetT());
      ySucc = pred.GetY() - dx[i - 3] * sin(pred.GetT()) + dy[i - 3] * cos(pred.GetT());
      tSucc = Utility::RadToZeroTo2P(pred.GetT() - dt[i - 3]);
    }
    out.emplace_back(new Node3D(xSucc, ySucc, tSucc, pred.GetG(), 0, &pred, i));
  }
  return out;
}

std::vector<Node2D *> Algorithm::CreateSuccessor(const Node2D &pred, const int &possible_dir)
{
  std::vector<Node2D *> successor_vec;
  if (possible_dir == 4)
  {
    std::vector<int> delta = {-1, 1};
    for (uint i = 0; i < delta.size(); ++i)
    {
      int x_successor = pred.GetX() + delta[i];
      successor_vec.emplace_back(new Node2D(x_successor, pred.GetY(), pred.GetG(), 0, &pred));
    }
    for (uint i = 0; i < delta.size(); ++i)
    {
      int y_successor = pred.GetY() + delta[i];
      successor_vec.emplace_back(new Node2D(pred.GetX(), y_successor, pred.GetG(), 0, &pred));
    }
  }
  else if (possible_dir == 8)
  {
    std::vector<int> delta_x = {-1, 0, 1};
    std::vector<int> delta_y = {-1, 0, 1};
    for (uint i = 0; i < delta_x.size(); ++i)
    {
      for (uint j = 0; j < delta_y.size(); ++j)
      {
        if (delta_x[i] == 0 && delta_y[j] == 0)
        {
          continue;
        }
        int x_successor = pred.GetX() + delta_x[i];
        int y_successor = pred.GetY() + delta_y[j];
        successor_vec.emplace_back(new Node2D(x_successor, y_successor, pred.GetG(), 0, &pred));
      }
    }
  }
  else
  {
    DLOG(WARNING) << "Wrong possible_dir!!!";
  }
  return successor_vec;
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