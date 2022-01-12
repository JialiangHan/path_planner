#include "hybrid_a_star.h"

namespace HybridAStar
{

  //###################################################
  //                                    NODE COMPARISON
  //###################################################
  /*!
   \brief A structure to sort nodes in a heap structure
*/
  struct CompareNodes
  {
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const std::shared_ptr<Node3D> lhs, const std::shared_ptr<Node3D> rhs) const
    {
      return lhs->GetC() > rhs->GetC();
    }
  };

  HybridAStar::HybridAStar(const ParameterHybridAStar &params,
                           const std::shared_ptr<Visualize> &visualization_ptr)
  {

    params_ = params;
    lookup_table_ptr_.reset(new LookupTable(params_.collision_detection_params));
    configuration_space_ptr_.reset(new CollisionDetection(params_.collision_detection_params));

    visualization_ptr_ = visualization_ptr;

    a_star_ptr_.reset(new AStar(configuration_space_ptr_,
                                visualization_ptr_, params_.possible_direction, params_.visualization2D));
  }

  void HybridAStar::Initialize(nav_msgs::OccupancyGrid::Ptr map)
  {
    //update the configuration space with the current map
    configuration_space_ptr_->UpdateGrid(map);
    int width = configuration_space_ptr_->grid_ptr_->info.width;
    int height = configuration_space_ptr_->grid_ptr_->info.height;
    lookup_table_ptr_->Initialize(width, height);
  }

  //###################################################
  //                                        3D A*
  //###################################################
  Path3D HybridAStar::GetPath(Node3D &start,
                              Node3D &goal,
                              Node3D *nodes3D,
                              Node2D *nodes2D,
                              int width,
                              int height)
  {

    DLOG(INFO) << "Hybrid A star start!!";
    start_ = start;
    goal_ = goal;
    //number of nodes explored
    int number_nodes_explored = 0;
    // PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    float newG;
    // Number of iterations the algorithm has run for stopping based on params_.max_iterations
    int iterations = 0;
    int analytical_expansion_counter = 0;
    // VISUALIZATION DELAY
    ros::Duration d(0.003);
    // OPEN LIST AS BOOST IMPLEMENTATION
    typedef boost::heap::binomial_heap<std::shared_ptr<Node3D>, boost::heap::compare<CompareNodes>> priorityQueue;
    priorityQueue openlist;
    // update h value
    UpdateHeuristic(start, goal, nodes2D, width, height);
    //this N is for analytic expansion
    int N = start.GetH();
    // mark start as open
    start.open();
    // push on priority queue aka open list
    std::shared_ptr<Node3D> start_ptr = std::make_shared<Node3D>(start);
    openlist.push(start_ptr);
    number_nodes_explored++;
    iPred = start.setIdx(width, height, (float)2 * M_PI / params_.headings);
    nodes3D[iPred] = start;
    // NODE POINTER
    std::shared_ptr<Node3D> nPred;
    std::shared_ptr<Node3D> nSucc;
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
        visualization_ptr_->publishNode3DPoses(*nPred);
        visualization_ptr_->publishNode3DPose(*nPred);
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
          DLOG(INFO) << "number of nodes explored is " << number_nodes_explored;
          // return nPred;
          TracePath(nPred);
          return path_;
        }
        else if (Utility::IsCloseEnough(*nPred, goal, params_.goal_range, 2 * M_PI / params_.headings))
        {
          DLOG(INFO) << "Goal reached!!!";
          // return nPred;
          TracePath(nPred);
          DLOG(INFO) << "number of nodes explored is " << number_nodes_explored;
          return path_;
        }
        // ____________________
        // CONTINUE WITH SEARCH
        else
        {
          if (params_.analytical_expansion)
          {
            // _______________________
            //analytical expansion
            // DLOG(INFO) << "analytical_expansion_counter is " << analytical_expansion_counter << " N is " << N;
            if (analytical_expansion_counter == N)
            {
              // DLOG(INFO) << "Start Analytic Expansion, every " << N << "th iterations";
              N = nPred->GetH();
              analytical_expansion_counter = 0;
              Path3D analytical_path = AnalyticExpansions(*nPred, goal, configuration_space_ptr_);
              if (analytical_path.size() != 0)
              {
                DLOG(INFO) << "Found path through anallytical expansion";
                DLOG(INFO) << "number of nodes explored is " << number_nodes_explored;
                TracePath(nPred);
                path_.insert(path_.end(), analytical_path.begin(), analytical_path.end());
                return path_;
              }
            }
            else
            {
              analytical_expansion_counter++;
            }
          }
          std::vector<std::shared_ptr<Node3D>> successor_vec = CreateSuccessor(*nPred, params_.possible_direction);
          // ______________________________
          // SEARCH WITH FORWARD SIMULATION
          for (const auto &node : successor_vec)
          {
            // create possible successor
            number_nodes_explored++;
            nSucc = node;
            // set index of the successor
            iSucc = nSucc->setIdx(width, height, (float)2 * M_PI / params_.headings);
            // ensure successor is on grid and traversable
            if (configuration_space_ptr_->IsTraversable(nSucc))
            {
              // ensure successor is not on closed list or it has the same index as the predecessor
              if (!nodes3D[iSucc].isClosed() || iPred == iSucc)
              {
                // calculate new G value
                UpdateCostSoFar(*nSucc, params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
                // nSucc->updateG(params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
                newG = nSucc->GetG();
                // if successor not on open list or found a shorter way to the cell
                if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].GetG() || iPred == iSucc)
                {
                  // calculate H value
                  UpdateHeuristic(*nSucc, goal, nodes2D, width, height);
                  // if the successor is in the same cell but the C value is larger
                  if (iPred == iSucc && nSucc->GetC() > nPred->GetC() + params_.tie_breaker)
                  {
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
                  std::shared_ptr<Node3D> nSucc_ptr = std::make_shared<Node3D>(nodes3D[iSucc]);
                  openlist.push(nSucc_ptr);
                }
              }
            }
          }
        }
      }
    }
    DLOG(INFO) << "open list is empty, end hybrid a star.";
    DLOG(INFO) << "number of nodes explored is " << number_nodes_explored;
    return path_;
  }

  //###################################################
  //                                         COST TO GO
  //###################################################
  void HybridAStar::UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D, int width, int height)
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
      curve_cost = lookup_table_ptr_->GetDubinsCost(goal_rotated_translated);
      // DLOG(INFO) << "dubins cost is " << curve_cost;
    }

    // if reversing is active use a
    if (params_.reverse == true)
    {
      curve_cost = lookup_table_ptr_->GetReedsSheppCost(goal_rotated_translated);
      // DLOG(INFO) << "rs cost is " << curve_cost;
    }

    // unconstrained with obstacles
    if (!nodes2D[(int)start.GetY() * width + (int)start.GetX()].isDiscovered())
    {
      //    ros::Time t0 = ros::Time::now();
      // create a 2d start node
      Node2D start2d(start.GetX(), start.GetY(), 0, 0, nullptr);
      // create a 2d goal node
      Node2D goal2d(goal.GetX(), goal.GetY(), 0, 0, nullptr);
      a_star_ptr_->Initialize(goal2d, start2d);
      // run 2d AStar and return the cost of the cheapest path for that node
      nodes2D[(int)start.GetY() * width + (int)start.GetX()].SetG(a_star_ptr_->GetAStarCost(nodes2D));
      //    ros::Time t1 = ros::Time::now();
      //    ros::Duration d(t1 - t0);
      //    DLOG(INFO) << "calculated 2D Heuristic in ms: " << d * 1000 ;
    }

    // offset for same node in cell
    twoDoffset = sqrt(((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) * ((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) +
                      ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())) * ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())));
    // DLOG(INFO) << "two d offset is " << twoDoffset;
    twoDCost = nodes2D[(int)start.GetY() * width + (int)start.GetX()].GetG() - twoDoffset;
    // DLOG(INFO) << "current node is " << start.GetX() << " " << start.GetY() << " " << start.GetT();
    // DLOG(INFO) << "a star cost is " << twoDCost;
    // DLOG(INFO) << "rs cost is " << curve_cost;
    // return the maximum of the heuristics, making the heuristic admissable
    start.SetH(std::max(curve_cost, twoDCost));
  }

  //###################################################
  //                                    ANALYTICAL EXPANSION
  //###################################################
  Path3D HybridAStar::AnalyticExpansions(const Node3D &start, Node3D &goal, std::shared_ptr<CollisionDetection> &configuration_space_ptr_)
  {
    Path3D path_vec;
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
    //     if (configuration_space_ptr_->IsTraversable(node3d))
    //     {

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
    map_width = configuration_space_ptr_->grid_ptr_->info.width;
    map_height = configuration_space_ptr_->grid_ptr_->info.height;
    CubicBezier::CubicBezier cubic_bezier(vector3d_start, vector3d_goal, map_width, map_height);
    float length = cubic_bezier.GetLength();
    i = 0;
    x = 0;
    path_vec.clear();
    while (x < length)
    {
      // DLOG(INFO) << i << "th iteration";
      Node3D node3d = Utility::ConvertVector3dToNode3D(cubic_bezier.GetValueAt(x / length));
      float curvature = cubic_bezier.GetCurvatureAt(x / length);
      node3d.setT(cubic_bezier.GetAngleAt(x / length));

      // collision check
      if (configuration_space_ptr_->IsTraversable(node3d))
      {
        if (curvature <= 1 / params_.min_turning_radius)
        {
          path_vec.emplace_back(node3d);
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
  //###################################################
  //                                    create successor
  //###################################################
  std::vector<std::shared_ptr<Node3D>> HybridAStar::CreateSuccessor(const Node3D &pred, const int &possible_dir)
  {
    std::vector<std::shared_ptr<Node3D>> out;
    std::shared_ptr<Node3D> pred_ptr = std::make_shared<Node3D>(pred);
    //assume constant speed.
    float theta = Utility::ConvertDegToRad(params_.steering_angle);
    float step_size = params_.min_turning_radius * theta;
    float distance_to_goal = Utility::GetDistance(pred, goal_);
    if (distance_to_goal < step_size)
    {
      step_size = distance_to_goal;
      theta = step_size / params_.min_turning_radius;
    }

    const float dx[] = {step_size, params_.min_turning_radius * sin(theta), params_.min_turning_radius * sin(theta)};
    const float dy[] = {0, -(params_.min_turning_radius * (1 - cos(theta))), (params_.min_turning_radius * (1 - cos(theta)))};
    const float dt[] = {0, theta, -theta};
    float xSucc, ySucc, tSucc;
    for (int i = 0; i < possible_dir; ++i)
    {
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
      std::shared_ptr<Node3D> temp = std::make_shared<Node3D>(Node3D(xSucc, ySucc, tSucc, pred.GetG(), 0, pred_ptr, i));
      out.emplace_back(temp);
    }
    if (params_.add_one_more_successor)
    {
      //add one more successor,angle is the difference between current node and goal,keep step size the same
      float angle_diff = Utility::RadNormalization(goal_.GetT() - pred.GetT());
      // DLOG(INFO) << "angle diff is " << Utility::ConvertRadToDeg(angle_diff);
      if (angle_diff < theta && angle_diff > -theta)
      {
        int sign = angle_diff > 0 ? 1 : -1;
        xSucc = pred.GetX() + params_.min_turning_radius * sin(abs(angle_diff)) * cos(pred.GetT()) - sign * params_.min_turning_radius * (1 - cos(theta)) * sin(pred.GetT());
        ySucc = pred.GetY() + params_.min_turning_radius * sin(abs(angle_diff)) * sin(pred.GetT()) + sign * params_.min_turning_radius * (1 - cos(theta)) * cos(pred.GetT());
        tSucc = Utility::RadToZeroTo2P(pred.GetT() + angle_diff);
        int prem = angle_diff > 0 ? 1 : 2;
        std::shared_ptr<Node3D> temp = std::make_shared<Node3D>(Node3D(xSucc, ySucc, tSucc, pred.GetG(), 0, pred_ptr, prem));
        out.emplace_back(temp);
        // DLOG(INFO) << "current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT());
        // DLOG(INFO) << "one more successor is " << temp->GetX() << " " << temp->GetY() << " " << Utility::ConvertRadToDeg(temp->GetT());
      }
    }
    return out;
  }

  //###################################################
  //                                    update cost so far
  //###################################################
  void HybridAStar::UpdateCostSoFar(Node3D &node, const float &weight_turning, const float &weight_change_of_direction, const float &weight_reverse)
  {
    float theta = Utility::ConvertDegToRad(params_.steering_angle);
    float step_size = params_.min_turning_radius * theta;
    float pred_cost_so_far = node.GetG();
    int prim = node.GetPrim();
    int pre_prim = node.GetPred()->GetPrim();
    // forward driving
    if (prim < 3)
    {
      // penalize turning
      if (pre_prim != prim)
      {
        // penalize change of direction
        if (pre_prim > 2)
        {
          pred_cost_so_far += step_size * weight_turning * weight_change_of_direction;
        }
        else
        {
          pred_cost_so_far += step_size * weight_turning;
        }
      }
      else
      {
        pred_cost_so_far += step_size;
      }
    }
    // reverse driving
    else
    {
      // penalize turning and reversing
      if (pre_prim != prim)
      {
        // penalize change of direction
        if (pre_prim < 3)
        {
          pred_cost_so_far += step_size * weight_turning * weight_reverse * weight_change_of_direction;
        }
        else
        {
          pred_cost_so_far += step_size * weight_turning * weight_reverse;
        }
      }
      else
      {
        pred_cost_so_far += step_size * weight_reverse;
      }
    }
    node.SetG(pred_cost_so_far);
  }
  void HybridAStar::TracePath(std::shared_ptr<Node3D> node3d_ptr)
  {
    path_.clear();
    while (node3d_ptr != nullptr)
    {

      path_.emplace_back(*node3d_ptr);
      if (*node3d_ptr == start_)
      {
        break;
      }
      // DLOG(INFO) << "current node is " << node->GetX() << " " << node->GetY() << " and its pred is " << node->GetPred()->GetX() << " " << node->GetPred()->GetY();
      node3d_ptr = node3d_ptr->GetPred();
    }
    std::reverse(path_.begin(), path_.end());
  }
}