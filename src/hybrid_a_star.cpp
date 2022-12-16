#include "hybrid_a_star.h"

namespace HybridAStar
{

  HybridAStar::HybridAStar(const ParameterHybridAStar &params,
                           const std::shared_ptr<Visualize> &visualization_ptr)
  {

    params_ = params;
    lookup_table_ptr_.reset(new LookupTable(params_.collision_detection_params));
    configuration_space_ptr_.reset(new CollisionDetection(params_.collision_detection_params));

    visualization_ptr_ = visualization_ptr;

    a_star_ptr_.reset(new AStar(params_.a_star_params,
                                visualization_ptr_));
  }

  void HybridAStar::Initialize(nav_msgs::OccupancyGrid::Ptr map)
  {
    // update the configuration space with the current map
    //  DLOG(INFO) << "hybrid a star initializing";
    configuration_space_ptr_->UpdateGrid(map);
    map_width_ = configuration_space_ptr_->GetMap()->info.width;
    map_height_ = configuration_space_ptr_->GetMap()->info.height;
    lookup_table_ptr_->Initialize(map_width_, map_height_);

    nav_msgs::OccupancyGrid::Ptr a_star_map = TreatAstarMap(map);
    a_star_ptr_->Initialize(a_star_map);

    LOG_IF(INFO, params_.adaptive_steering_angle_and_step_size) << "adaptive_steering_angle_and_step_size";
    LOG_IF(INFO, !params_.adaptive_steering_angle_and_step_size) << "use fixed step size";
    // DLOG(INFO) << "hybrid a star initialized done.   ";
  }

  //###################################################
  //                                        3D A*
  //###################################################
  Utility::Path3D HybridAStar::GetPath(Node3D &start, Node3D &goal, Node3D *nodes3D, Node2D *nodes2D)
  {
    path_.clear();
    DLOG(INFO) << "Hybrid A star start!!";
    start_ = start;
    goal_ = goal;
    // number of nodes explored
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

    priorityQueue openlist;
    // update h value
    UpdateHeuristic(start, goal, nodes2D);
    // this N is for analytic expansion
    int N = start.getCostToGo();
    // mark start as open
    start.setOpenSet();
    // push on priority queue aka open list
    std::shared_ptr<Node3D> start_ptr = std::make_shared<Node3D>(start);
    openlist.push(start_ptr);

    iPred = start.setIdx(map_width_, map_height_, (float)2 * M_PI / params_.headings);
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
      number_nodes_explored++;
      // set index
      iPred = nPred->setIdx(map_width_, map_height_, (float)2 * M_PI / params_.headings);
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

      if (nodes3D[iPred].isClosedSet())
      {
        // DLOG(INFO) << "nPred is already closesd!";
        // pop node from the open list and start with a fresh node
        openlist.pop();
        continue;
      }
      // _________________
      // EXPANSION OF NODE
      else if (nodes3D[iPred].isOpenSet())
      {
        // add node to closed list
        nodes3D[iPred].setClosedSet();
        // remove node from open list
        openlist.pop();
        // _________
        // GOAL TEST
        if (iterations > params_.max_iterations)
        {
          DLOG(INFO) << "max iterations reached!!!, current iterations is :" << iterations;
          LOG(INFO) << "number of nodes explored is " << number_nodes_explored;
          // return nPred;
          TracePath(nPred);
          return path_;
        }
        else if (Utility::IsCloseEnough(*nPred, goal, params_.goal_range, 2 * M_PI / params_.headings, true))
        {
          // DLOG(INFO) << "current node is " << nPred->getX() << " " << nPred->getY() << " " << Utility::ConvertRadToDeg(nPred->getT());
          // DLOG(INFO) << "goal is " << goal.getX() << " " << goal.getY() << " " << Utility::ConvertRadToDeg(goal.getT());
          // DLOG(INFO) << "Goal reached!!!";
          // return nPred;
          TracePath(nPred);
          LOG(INFO) << "number of nodes explored is " << number_nodes_explored;
          return path_;
        }
        // ____________________
        // CONTINUE WITH SEARCH
        else
        {
          if (params_.analytical_expansion)
          {
            // _______________________
            // analytical expansion
            // DLOG(INFO) << "analytical_expansion_counter is " << analytical_expansion_counter << " N is " << N;
            if (params_.analytical_expansion_every_point)
            {
              // DLOG(INFO) << "Start Analytic Expansion, every " << N << "th iterations";
              Utility::Path3D analytical_path = AnalyticExpansions(*nPred, goal);
              if (analytical_path.size() != 0)
              {
                DLOG(INFO) << "Found path through analytical expansion";
                LOG(INFO) << "number of nodes explored is " << number_nodes_explored;
                TracePath(nPred);
                analytical_expansion_index_ = path_.size();
                path_.insert(path_.end(), analytical_path.begin(), analytical_path.end());
                if (params_.piecewise_cubic_bezier_interpolation)
                {

                  ConvertToPiecewiseCubicBezierPath();
                  piecewise_cubic_bezier_path_.insert(piecewise_cubic_bezier_path_.end(), analytical_path.begin(), analytical_path.end());
                  // DLOG(INFO) << "piecewise cubic bezier interpolation.";
                  // DLOG(INFO) << "piecewise_cubic_bezier_path_ size is " << piecewise_cubic_bezier_path_.size();

                  return piecewise_cubic_bezier_path_;
                }
                return path_;
              }
            }
            else
            {
              if (analytical_expansion_counter == N)
              {
                // DLOG(INFO) << "Start Analytic Expansion, every " << N << "th iterations";
                N = nPred->getCostToGo();
                analytical_expansion_counter = 0;
                Utility::Path3D analytical_path = AnalyticExpansions(*nPred, goal);
                if (analytical_path.size() != 0)
                {
                  DLOG(INFO) << "Found path through analytical expansion";
                  LOG(INFO) << "number of nodes explored is " << number_nodes_explored;
                  TracePath(nPred);
                  analytical_expansion_index_ = path_.size();
                  path_.insert(path_.end(), analytical_path.begin(), analytical_path.end());
                  if (params_.piecewise_cubic_bezier_interpolation)
                  {
                    ConvertToPiecewiseCubicBezierPath();
                    piecewise_cubic_bezier_path_.insert(piecewise_cubic_bezier_path_.end(), analytical_path.begin(), analytical_path.end());
                    DLOG(INFO) << "piecewise cubic bezier interpolation.";
                    // DLOG(INFO) << "piecewise_cubic_bezier_path_ size is " << piecewise_cubic_bezier_path_.size();
                    return piecewise_cubic_bezier_path_;
                  }
                  return path_;
                }
              }
              else
              {
                analytical_expansion_counter++;
              }
            }
          }
          std::vector<std::shared_ptr<Node3D>> successor_vec = CreateSuccessor(*nPred);
          // DLOG(INFO) << "successor vec length is " << successor_vec.size();

          // ______________________________
          // SEARCH WITH FORWARD SIMULATION
          for (const auto &node : successor_vec)
          {
            // TODO add a check function to check if this successor are close to nodes already explored, if yes, then skip this successor
            if (DuplicateCheck(openlist, node))
            {
              continue;
            }

            // create possible successor
            nSucc = node;
            // set index of the successor
            iSucc = nSucc->setIdx(map_width_, map_height_, (float)2 * M_PI / params_.headings);
            // ensure successor is on grid and traversable
            if (configuration_space_ptr_->IsTraversable(nSucc))
            {
              // DLOG(INFO) << "index is " << iSucc;
              // ensure successor is not on closed list or it has the same index as the predecessor
              if (!nodes3D[iSucc].isClosedSet())
              {
                // calculate new G value
                UpdateCostSoFar(*nSucc, params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
                // nSucc->updateG(params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
                newG = nSucc->getCostSofar();
                // if successor not on open list or found a shorter way to the cell
                if (!nodes3D[iSucc].isOpenSet() || newG < nodes3D[iSucc].getCostSofar())
                {
                  // calculate H value
                  UpdateHeuristic(*nSucc, goal, nodes2D);
                  // same cell expansion
                  if (iPred == iSucc)
                  {
                    DLOG(INFO) << "successor total cost is " << nSucc->getTotalCost() << " pred total cost is " << nPred->getTotalCost();
                    // no need to reset tie-breaker
                    //  if the successor is in the same cell but the C value is larger
                    if (nSucc->getTotalCost() > nPred->getTotalCost() + params_.tie_breaker)
                    {
                      continue;
                    }
                    // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                    else if (nSucc->getTotalCost() <= nPred->getTotalCost() + params_.tie_breaker)
                    {
                      DLOG(INFO) << "same cell expansion, but successor has lower cost.";
                      nSucc->setSmartPtrPred(nPred->getSmartPtrPred());
                    }
                  }
                  // put successor on open list
                  nSucc->setOpenSet();
                  nodes3D[iSucc] = *nSucc;
                  std::shared_ptr<Node3D> nSucc_ptr = std::make_shared<Node3D>(nodes3D[iSucc]);
                  openlist.push(nSucc_ptr);
                }
              }
            }
            else
            {
              // DLOG(INFO) << "current node is " << nSucc->getX() << " " << nSucc->getY() << " " << Utility::ConvertRadToDeg(nSucc->getT()) << " is in collision!!";
            }
          }
        }
      }
    }
    LOG(INFO) << "open list is empty, end hybrid a star. number of nodes explored is " << number_nodes_explored;
    return path_;
  }

  //###################################################
  //                                         COST TO GO
  //###################################################
  void HybridAStar::UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D)
  {
    // DLOG(INFO) << "UpdateHeuristic in:";
    float curve_cost = 0, twoDCost = 0, twoDoffset = 0;
    // translate and rotate
    Node3D goal_rotated_translated;
    goal_rotated_translated.setX(abs(goal.getX() - start.getX()));
    goal_rotated_translated.setY(abs(goal.getY() - start.getY()));
    goal_rotated_translated.setT(Utility::RadToZeroTo2P(goal.getT() - start.getT()));
    // DLOG(INFO) << "goal is " << goal.getX() << " " << goal.getY() << " " << Utility::ConvertRadToDeg(goal.getT());
    // DLOG(INFO) << "start is " << start.getX() << " " << start.getY() << " " << Utility::ConvertRadToDeg(start.getT());
    // DLOG(INFO) << "goal rotated is " << goal_rotated_translated.getX() << " " << goal_rotated_translated.getY() << " " << Utility::ConvertRadToDeg(goal_rotated_translated.getT());
    // if dubins heuristic is activated calculate the shortest path
    // constrained without obstacles
    if (params_.collision_detection_params.curve_type == 0)
    {
      curve_cost = lookup_table_ptr_->GetDubinsCost(goal_rotated_translated);
      // DLOG(INFO) << "dubins cost is " << curve_cost;
    }

    // if reversing is active use a
    else if (params_.collision_detection_params.curve_type == 1)
    {
      curve_cost = lookup_table_ptr_->GetReedsSheppCost(goal_rotated_translated);
      // DLOG(INFO) << "rs cost is " << curve_cost;
    }
    else
    {
      curve_cost = lookup_table_ptr_->GetCubicBezierCost(goal_rotated_translated);
      // DLOG(INFO) << "cubic bezier cost is " << curve_cost;
    }

    // unconstrained with obstacles
    if (!nodes2D[(int)start.getY() * map_width_ + (int)start.getX()].isDiscovered())
    {
      // ros::Time t0 = ros::Time::now();
      // create a 2d start node
      Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
      // create a 2d goal node
      Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
      // DLOG(INFO) << "A star start node is " << start2d.getX() << " " << start2d.getY() << " goal is " << goal2d.getX() << " " << goal2d.getY();

      // run 2d AStar and return the cost of the cheapest path for that node
      nodes2D[(int)start.getY() * map_width_ + (int)start.getX()].setCostSofar(a_star_ptr_->GetAStarCost(nodes2D, start2d, goal2d, true));
      // ros::Time t1 = ros::Time::now();
      // ros::Duration d(t1 - t0);
      // DLOG(INFO) << "calculated 2D Heuristic in ms: " << d * 1000;
    }

    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    // DLOG(INFO) << "twoD cost before offset is " << nodes2D[(int)start.getY() * map_width_ + (int)start.getX()].getCostSofar() << " two d offset is " << twoDoffset;
    twoDCost = nodes2D[(int)start.getY() * map_width_ + (int)start.getX()].getCostSofar() - twoDoffset;

    // DLOG(INFO) << "current node is " << start.getX() << " " << start.getY() << " " << start.getT() << " a star cost is " << twoDCost << " curve cost is " << curve_cost << " heuristic is " << std::max(curve_cost, twoDCost);
    // DLOG(INFO) << "rs cost is " << curve_cost;
    // return the maximum of the heuristics, making the heuristic admissable
    start.setCostToGo(std::max(curve_cost, twoDCost));
    // DLOG(INFO) << "UpdateHeuristic out.";
  }

  //###################################################
  //                                    ANALYTICAL EXPANSION
  //###################################################
  Utility::Path3D HybridAStar::AnalyticExpansions(const Node3D &start, Node3D &goal)
  {
    Utility::Path3D path_vec;
    int i = 0;
    float x = 0.f;
    Eigen::Vector3f vector3d_start, vector3d_goal;
    Utility::TypeConversion(start, vector3d_start);
    // DLOG(INFO) << "start point is " << vector3d_start.x() << " " << vector3d_start.y();
    Utility::TypeConversion(goal, vector3d_goal);

    // Dubins/RS curve
    if (params_.curve_type_analytical_expansion == 0)
    {
      // start
      float q0[] = {start.getX(), start.getY(), start.getT()};
      // goal
      float q1[] = {goal.getX(), goal.getY(), goal.getT()};
      // initialize the path
      DubinsPath path;
      // calculate the path
      dubins_init(q0, q1, params_.min_turning_radius, &path);
      float length = dubins_path_length(&path);
      while (x < length)
      {
        float q[3];
        dubins_path_sample(&path, x, q);
        Node3D node3d;
        node3d.setX(q[0]);
        node3d.setY(q[1]);
        node3d.setT(Utility::RadToZeroTo2P(q[2]));
        // collision check
        if (configuration_space_ptr_->IsTraversable(node3d))
        {
          path_vec.emplace_back(node3d);
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
    // beizer curve
    if (params_.curve_type_analytical_expansion == 1)
    {
      CubicBezier::CubicBezier cubic_bezier(vector3d_start, vector3d_goal, map_width_, map_height_);
      float length = cubic_bezier.GetLength();
      i = 0;
      x = 0;
      path_vec.clear();
      while (x < length)
      {
        // DLOG(INFO) << i << "th iteration";
        Node3D node3d;
        Utility::TypeConversion(cubic_bezier.GetValueAt(x / length), node3d);

        float curvature = cubic_bezier.GetCurvatureAt(x / length);
        node3d.setT(cubic_bezier.GetAngleAt(x / length));
        // DLOG(INFO) << "current node is " << node3d.getX() << " " << node3d.getY();
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
    }

    // Some kind of collision detection for all curves
    if (path_vec.size() != 0)
    {
      path_vec.emplace_back(goal);
      // std::string out = params_.reverse == true ? "cubic bezier" : "dubins";
      DLOG(INFO) << "Analytical expansion connected, returning path";
      LOG(INFO) << "analytical expansion start at " << start.getX() << " " << start.getY() << " " << Utility::ConvertRadToDeg(start.getT());
    }
    return path_vec;
  }
  //###################################################
  //                                    create successor
  //###################################################

  std::vector<std::shared_ptr<Node3D>> HybridAStar::CreateSuccessor(const Node3D &pred)
  {
    // DLOG(INFO) << "CreateSuccessor in:";
    std::vector<std::shared_ptr<Node3D>> out;
    std::shared_ptr<Node3D> pred_ptr = std::make_shared<Node3D>(pred);
    std::vector<std::pair<float, float>> available_steering_angle_and_step_size_vec;
    float distance_to_goal = Utility::GetDistance(pred, goal_);
    if (true)
    {
      float current_normalized_obstacle_density = configuration_space_ptr_->GetNormalizedObstacleDensity(pred);
      float constant_density = params_.constant_density;
      // LOG(INFO) << "current node normalized obstacle density is " << current_normalized_obstacle_density;
      if (current_normalized_obstacle_density > constant_density)
      {
        // LOG(INFO) << "fixed steering angle and step size";
        // assume constant speed.

        std::vector<float> available_steering_angle_vec = Utility::FormSteeringAngleVec(params_.steering_angle, params_.number_of_successors);
        std::pair<float, float> pair;

        if (params_.step_size > distance_to_goal)
        {
          pair.first = distance_to_goal;
        }
        else
        {
          pair.first = params_.step_size;
        }

        for (const auto &element : available_steering_angle_vec)
        {
          pair.second = element;
          available_steering_angle_and_step_size_vec.emplace_back(pair);
        }
        // LOG(INFO) << "fix step size;";
      }
      else
      {
        // LOG(INFO) << "adaptive steering angle and step size";
        // 1. decide step size and steering angle: in vehicle available range, if there is a free range, then go to that direction,if no free range, then based on the distance to obstacle, decide step size
        available_steering_angle_and_step_size_vec = configuration_space_ptr_->FindStepSizeAndSteeringAngle(pred, start_, goal_, params_.number_of_successors, params_.step_size);
        // DLOG(INFO) << "CreateSuccessor out.";
        // LOG(INFO) << "adaptive step size;";
      }
    }
    else
    {

      // DLOG(INFO) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT());
      if (params_.adaptive_steering_angle_and_step_size)
      {
        // LOG(INFO) << "adaptive steering angle and step size";
        // 1. decide step size and steering angle: in vehicle available range, if there is a free range, then go to that direction,if no free range, then based on the distance to obstacle, decide step size
        available_steering_angle_and_step_size_vec = configuration_space_ptr_->FindStepSizeAndSteeringAngle(pred, start_, goal_, params_.number_of_successors, params_.step_size);
        // DLOG(INFO) << "CreateSuccessor out.";
      }
      else if (params_.adaptive_step_size)
      {
        // LOG(INFO) << "fixed steering angle and step size";
        // assume constant speed.

        std::vector<float> available_steering_angle_vec = Utility::FormSteeringAngleVec(params_.steering_angle, params_.number_of_successors);
        std::pair<float, float> pair;

        for (const auto &element : available_steering_angle_vec)
        {
          pair.first = configuration_space_ptr_->FindStepSize(pred, element, goal_, params_.step_size);
          pair.second = element;
          available_steering_angle_and_step_size_vec.emplace_back(pair);
        }
      }
      else
      {
        // LOG(INFO) << "fixed steering angle and step size";
        // assume constant speed.

        std::vector<float> available_steering_angle_vec = Utility::FormSteeringAngleVec(params_.steering_angle, params_.number_of_successors);
        std::pair<float, float> pair;

        if (params_.step_size > distance_to_goal)
        {
          pair.first = distance_to_goal;
        }
        else
        {
          pair.first = params_.step_size;
        }

        for (const auto &element : available_steering_angle_vec)
        {
          pair.second = element;
          available_steering_angle_and_step_size_vec.emplace_back(pair);
        }
      }
      // for (const auto &element : available_steering_angle_and_step_size_vec)
      // {
      //   LOG(INFO) << "step size is " << element.first << " steering angle is " << Utility::ConvertRadToDeg(element.second);
      // }
    }

    out = CreateSuccessor(pred, available_steering_angle_and_step_size_vec);
    // DLOG(INFO) << "CreateSuccessor out.";
    return out;
  }

  std::vector<std::shared_ptr<Node3D>> HybridAStar::CreateSuccessor(const Node3D &pred, const std::vector<std::pair<float, float>> &step_size_steering_angle_vec)
  {
    std::vector<std::shared_ptr<Node3D>> out;
    float dx, dy, xSucc, ySucc, tSucc, turning_radius, steering_angle;
    int prem;
    std::shared_ptr<Node3D> pred_ptr = std::make_shared<Node3D>(pred);

    for (const auto &pair : step_size_steering_angle_vec)
    {
      if (pair.first <= 1e-3)
      {
        DLOG(INFO) << "current step size is zero, no need to create successor!!";
        continue;
      }
      steering_angle = pair.second;

      turning_radius = pair.first / abs(steering_angle);
      // forward, checked
      // right
      if (steering_angle < 0)
      {
        dx = turning_radius * sin(abs(steering_angle));
        dy = -(turning_radius * (1 - cos(steering_angle)));
        prem = 1;
        // DLOG(INFO) << "forward right, dx " << dx << " dy " << dy;
      }
      // left
      else if (steering_angle > 1e-3)
      {
        dx = turning_radius * sin(abs(steering_angle));
        dy = (turning_radius * (1 - cos(steering_angle)));
        prem = 2;
        // DLOG(INFO) << "forward left, dx " << dx << " dy " << dy;
      }
      // straight forward,checked
      else
      {
        dx = pair.first;
        dy = 0;
        prem = 0;
        // DLOG(INFO) << "forward straight";
      }
      xSucc = pred.getX() + dx * cos(pred.getT()) - dy * sin(pred.getT());
      ySucc = pred.getY() + dx * sin(pred.getT()) + dy * cos(pred.getT());
      tSucc = Utility::RadToZeroTo2P(pred.getT() + steering_angle);
      // DLOG_IF(INFO, (pair.first < 1) || (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < -Utility::ConvertDegToRad(30))) << "in create successor, current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " step size is " << pair.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(steering_angle) << " successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
      // DLOG_IF(INFO, (xSucc > 77) && (xSucc < 79) && (ySucc > 1) && (ySucc < 3)) << "in create successor, current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " step size is " << pair.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(steering_angle) << " successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
      std::shared_ptr<Node3D> temp = std::make_shared<Node3D>(Node3D(xSucc, ySucc, tSucc, pred.getCostSofar(), 0, params_.reverse, nullptr, pred_ptr, prem));
      out.emplace_back(temp);
      if (params_.reverse)
      {
        // backward,checked
        // right
        if (steering_angle > 1e-3)
        {
          prem = 5;
          // DLOG(INFO) << "backward left";
        }
        // left
        else if (steering_angle < 0)
        {
          prem = 4;
          // DLOG(INFO) << "backward right";
        }
        // straight backward
        else
        {
          prem = 3;
          // DLOG(INFO) << "backward straight";
        }
        xSucc = pred.getX() - dx * cos(pred.getT()) - dy * sin(pred.getT());
        ySucc = pred.getY() - dx * sin(pred.getT()) + dy * cos(pred.getT());
        tSucc = Utility::RadToZeroTo2P(pred.getT() - steering_angle);
        // DLOG(INFO) << "successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
        std::shared_ptr<Node3D> temp = std::make_shared<Node3D>(Node3D(xSucc, ySucc, tSucc, pred.getCostSofar(), 0, params_.reverse, nullptr, pred_ptr, prem));
        out.emplace_back(temp);
      }
    }
    DLOG_IF(WARNING, out.size() == 0) << "WARNING: Number of successor is zero!!!!";
    return out;
  }

  //###################################################
  //                                    update cost so far
  //###################################################
  void HybridAStar::UpdateCostSoFar(Node3D &node, const float &weight_turning, const float &weight_change_of_direction, const float &weight_reverse)
  {
    // float theta = Utility::ConvertDegToRad(params_.steering_angle);
    float step_size = Utility::GetDistance(node, *node.getSmartPtrPred());
    float pred_cost_so_far = node.getCostSofar();
    int prim = node.getPrim();
    int pre_prim = node.getSmartPtrPred()->getPrim();
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
    node.setCostSofar(pred_cost_so_far);
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
      // DLOG(INFO) << "current node is " << node->getX() << " " << node->getY() << " and its pred is " << node->getPred()->getX() << " " << node->getPred()->getY();
      node3d_ptr = node3d_ptr->getSmartPtrPred();
    }
    std::reverse(path_.begin(), path_.end());
  }

  void HybridAStar::ConvertToPiecewiseCubicBezierPath()
  {
    // DLOG(INFO) << "ConvertToPiecewiseCubicBezierPath in:";
    std::vector<Eigen::Vector3f> anchor_points_vec;
    piecewise_cubic_bezier_path_.clear();
    Node3D end_point;
    Eigen::Vector3f start_vector, end_point_vec;
    Utility::TypeConversion(start_, start_vector);
    Utility::TypeConversion(end_point, end_point_vec);
    // for (uint jj = 1; jj < path_.size() - 1; ++jj)
    // {
    //   DLOG(INFO) << "path point is " << path_[jj].getX() << " " << path_[jj].getY() << " " << path_[jj].getT();
    //   DLOG(INFO) << "distance to next point is " << Utility::GetDistance(path_[jj], path_[jj + 1]);
    // }
    float distance_to_prev, distance_to_succ;
    for (uint index = 1; index < path_.size() - 1; ++index)
    {
      // DLOG(INFO) << "path point is " << path_[index].getX() << " " << path_[index].getY() << " " << Utility::ConvertRadToDeg(path_[index].getT());
      // not necessary to do this check, greater than 1
      bool flag = true;
      if (flag)
      {
        if (index == analytical_expansion_index_)
        {
          // DLOG(INFO) << "distance between path node is smaller than 1, set it to end point.";
          // DLOG(INFO) << "current path index is " << index;
          end_point.setX(path_[index - 1].getX());
          end_point.setY(path_[index - 1].getY());
          end_point.setT(path_[index - 1].getT());
          break;
        }
        if (Utility::GetDistance(path_[index], path_[index + 1]) < 0.1)
        {
          continue;
          // DLOG(INFO) << "points are too close to each other!!!!";
        }
        Eigen::Vector3f vector_3d;
        Utility::TypeConversion(path_[index], vector_3d);
        anchor_points_vec.emplace_back(vector_3d);
        // DLOG(INFO) << "current path index is " << index;
      }
      else
      {
        distance_to_prev = Utility::GetDistance(path_[index], path_[index - 1]);
        distance_to_succ = Utility::GetDistance(path_[index], path_[index + 1]);
        Eigen::Vector3f vector_3d;
        Utility::TypeConversion(path_[index], vector_3d);
        if (distance_to_succ >= 1 && distance_to_prev >= 1)
        {
          anchor_points_vec.emplace_back(vector_3d);
          // DLOG(INFO) << "anchor points is " << path_[index].getX() << " " << path_[index].getY() << " " << Utility::ConvertRadToDeg(path_[index].getT());
        }
        else if (distance_to_succ >= 1 && distance_to_prev < 1)
        {
          continue;
        }
        else if (distance_to_prev >= 1 && distance_to_succ < 1)
        {
          anchor_points_vec.emplace_back(vector_3d);
          // DLOG(INFO) << "anchor points is " << path_[index].getX() << " " << path_[index].getY() << " " << Utility::ConvertRadToDeg(path_[index].getT());
        }
        if (index == analytical_expansion_index_)
        {
          DLOG(INFO) << "distance between path node is smaller than 1, set it to end point.";
          DLOG(INFO) << "current path index is " << index;
          end_point.setX(path_[index].getX());
          end_point.setY(path_[index].getY());
          end_point.setT(path_[index].getT());
          break;
        }
      }
    }
    // DLOG(INFO) << "end point is " << end_point.getX() << " " << end_point.getY() << " " << Utility::ConvertRadToDeg(end_point.getT());
    PiecewiseCubicBezier pwcb(start_vector, end_point_vec);
    pwcb.SetAnchorPoints(anchor_points_vec);
    // for (const auto &point : anchor_points_vec)
    // {
    //   DLOG(INFO) << "anchor point is " << point.x() << " " << point.y() << " " << Utility::ConvertRadToDeg(point.z());
    // }
    std::vector<Eigen::Vector3f> points_vec = pwcb.GetPointsVec();
    // for (const auto &point : points_vec)
    // {
    //   DLOG(INFO) << "all point is " << point.x() << " " << point.y() << " " << Utility::ConvertRadToDeg(point.z());
    // }
    std::vector<Eigen::Vector3f> path_vec = pwcb.ConvertPiecewiseCubicBezierToVector3f(10);
    for (const auto &vector : path_vec)
    {
      // DLOG(INFO) << "vector is " << vector.x() << " " << vector.y() << " " << Utility::ConvertRadToDeg(vector.z());
      Node3D node3d;
      Utility::TypeConversion(vector, node3d);
      piecewise_cubic_bezier_path_.emplace_back(node3d);
      // DLOG(INFO) << "point is " << node3d.getX() << " " << node3d.getY() << " " << Utility::ConvertRadToDeg(node3d.getT());
    }
    // DLOG(INFO) << "ConvertToPiecewiseCubicBezierPath out.";
  }

  void HybridAStar::AddOneMoreStepSizeAndSteeringAngle(const Node3D &pred, std::vector<std::pair<float, float>> &step_size_steering_angle_pair)
  {
    // 1. first find distance from current node to goal position
    float distance_to_goal = Utility::GetDistance(pred, goal_);
    // DLOG(INFO) << "distance to goal is " << distance_to_goal;
    // 2. if distance to goal is less than step size above. than make it new step size, otherwise use old one

    float step_size, steering_angle;
    DLOG_IF(WARNING, step_size_steering_angle_pair.size() == 0) << "WARNING: step_size_steering_angle_pair size is zero!!!";
    if (step_size_steering_angle_pair.size() != 0)
    {
      step_size = step_size_steering_angle_pair.back().first;
    }
    else
    {
      float weight_step_size = -0.8 * configuration_space_ptr_->GetNormalizedObstacleDensity(pred) + 0.9;
      step_size = weight_step_size * configuration_space_ptr_->GetObstacleDetectionRange();
      // DLOG_IF(INFO, step_size < 1) << "step size is " << step_size;
    }

    if (distance_to_goal < step_size)
    {
      step_size = distance_to_goal;
    }
    // DLOG(INFO) << "step size is " << step_size;
    // 3. find angle to goal
    // TODO how to select this steering angle, is difference between current orientation and goal orientation a good choice or we should choose the angle from current location to goal?
    float angle_to_goal = -Utility::RadNormalization(pred.getT() - goal_.getT());
    bool flag = true;
    if (flag)
    {
      angle_to_goal = -Utility::RadNormalization(pred.getT() - Utility::GetAngle(pred, goal_));
      // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(Utility::RadNormalization(Utility::GetAngle(pred, goal_)));
      // DLOG(INFO) << "steering angle is " << Utility::ConvertRadToDeg(angle_to_goal);
    }
    // DLOG(INFO) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " and goal orientation is " << Utility::ConvertRadToDeg(goal_.getT());
    // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(angle_to_goal);
    // 4. if angle to goal is in the steering angle range(current orientation +-30deg), then make it steering angle, otherwise 30 or -30 to make angle to goal smaller

    if (std::abs(angle_to_goal) > Utility::ConvertDegToRad(30))
    {
      // if (pred.getT() > Utility::RadNormalization(goal_.getT()) && pred.getT() < Utility::RadNormalization((goal_.getT() + Utility::ConvertDegToRad(180))))
      if (angle_to_goal < -Utility::ConvertDegToRad(30))
      {
        // right is negative
        steering_angle = -Utility::ConvertDegToRad(30);
      }
      else
      {
        // left is positive
        steering_angle = Utility::ConvertDegToRad(30);
      }
    }
    else
    {
      steering_angle = angle_to_goal;
    }

    DLOG(INFO) << "current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " and goal orientation is " << Utility::ConvertRadToDeg(goal_.getT()) << " one more step size is " << step_size << " and steering angle pair is " << Utility::ConvertRadToDeg(steering_angle);

    step_size_steering_angle_pair.emplace_back(std::pair<float, float>(step_size, steering_angle));
  }

  nav_msgs::OccupancyGrid::Ptr HybridAStar::TreatAstarMap(nav_msgs::OccupancyGrid::Ptr map)
  {
    nav_msgs::OccupancyGrid::Ptr out;
    out.reset(new nav_msgs::OccupancyGrid);
    out->header = map->header;
    out->info = map->info;
    out->data.resize(map->data.size());
    for (size_t i = 0; i < map->data.size(); i++)
    {
      if (map->data[i] == 0)
      {
        out->data[i] = map->data[i];
      }
      else
      {
        out->data[i] = 0;
      }
    }
    return out;
  }

  bool HybridAStar::DuplicateCheck(priorityQueue &open_list, std::shared_ptr<Node3D> node_3d_ptr)
  {
    for (const auto &element : open_list)
    {
      if (abs(element->getX() - node_3d_ptr->getX()) < 0.2 && abs(element->getY() - node_3d_ptr->getY()) < 0.2 && abs(element->getT() - node_3d_ptr->getT()) < Utility::ConvertDegToRad(3))
      {
        // LOG(INFO) << "node already in open list.";
        return true;
      }
    }
    return false;
  }

  void HybridAStar::updateH(Node3D &start, const Node3D &goal, Node2D *nodes2D,
                            float *dubinsLookup, int width, int height, float inspireAstar)
  {
    float reedsSheppCost = 0;
    // float twoDCost = 0;
    // float twoDoffset = 0;
#ifdef use_ReedsShepp_heuristic
    // 假如车子可以后退，则可以启动Reeds-Shepp 算法
    if (Constants::reverse && !Constants::dubins)
    {
      // reeds_shepp算法还还存在问题，启用可能会造成搜寻路径增加等问题

      ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
      State *rsStart = (State *)reedsSheppPath.allocState();
      State *rsEnd = (State *)reedsSheppPath.allocState();
      rsStart->setXY(start.getX(), start.getY());
      rsStart->setYaw(start.getT());
      rsEnd->setXY(goal.getX(), goal.getY());
      rsEnd->setYaw(goal.getT());
      reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd) * 1.1 +
                       0.04 * start.getCost();
    }
#endif
    start.setCostToGo(std::max(reedsSheppCost, inspireAstar)); // 将两个代价中的最大值作为启发式值
  }

  Node3D *HybridAStar::dubinsShot(Node3D &start, Node3D &goal, costmap_2d::Costmap2D *costmap)
  {
    // start
    float q0[] = {start.getX(), start.getY(), start.getT()};
    // goal
    float q1[] = {goal.getX(), goal.getY(), goal.getT()};
    // initialize the path
    DubinsPath path;
    // calculate the path
    dubins_init(q0, q1, Constants::r, &path);

    unsigned int poseX, poseY;

    int i = 0;
    float x = 0.f;
    float length = dubins_path_length(&path);
    // printf("the length of dubins %f",length);
    Node3D *dubinsNodes = new Node3D[(int)(length / Constants::dubinsStepSize) + 1];

    while (x < length)
    { // 这是跳出循环的条件之一：生成的路径没有达到所需要的长度
      float q[3];
      dubins_path_sample(&path, x, q);
      dubinsNodes[i].setX(q[0]);
      dubinsNodes[i].setY(q[1]);
      dubinsNodes[i].setT(q[2]);

      // collision check
      // 跳出循环的条件之二：生成的路径存在碰撞节点
      costmap->worldToMap(dubinsNodes[i].getX(), dubinsNodes[i].getY(), poseX, poseY);
      if (costmap->getCost(poseX, poseY) <= 1)
      {

        // set the predecessor to the previous step
        if (i > 0)
        {
          dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
        }
        else
        {
          dubinsNodes[i].setPred(&start);
        }

        if (&dubinsNodes[i] == dubinsNodes[i].getPred())
        {
          std::cout << "looping shot";
        }

        x += Constants::dubinsStepSize;
        i++;
      }
      else
      {
        delete[] dubinsNodes;
        return nullptr;
      }
    }
    // 返回末节点，通过getPred()可以找到前一节点。
    return &dubinsNodes[i - 1];
  }

  Node3D *HybridAStar::reedsSheppShot(Node3D &start, Node3D &goal, costmap_2d::Costmap2D *costmap)
  {

    ReedsSheppStateSpace rs_planner(Constants::r);
    double length = -1;
    unsigned int poseX, poseY;
    // start
    double q0[] = {start.getX(), start.getY(), start.getT()};
    // goal
    double q1[] = {goal.getX(), goal.getY(), goal.getT()};
    std::vector<std::vector<double>> rs_path;
    rs_planner.sample(q0, q1, Constants::dubinsStepSize, length, rs_path);
    Node3D *dubinsNodes = new Node3D[(int)(length / Constants::dubinsStepSize) + 1];
    int i = 0;
    dubinsNodes[1].setPred(&start);
    for (auto &point_itr : rs_path)
    {
      dubinsNodes[i].setX(point_itr[0]);
      dubinsNodes[i].setY(point_itr[1]);
      dubinsNodes[i].setT(point_itr[2]);

      // collision check
      costmap->worldToMap(dubinsNodes[i].getX(), dubinsNodes[i].getY(), poseX, poseY);
      if (costmap->getCost(poseX, poseY) < 100)
      {
        // set the predecessor to the previous step
        if (i > 1)
        {
          dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
        }
        // else {
        //   dubinsNodes[i].setPred(&start);
        // }
        if (&dubinsNodes[i] == dubinsNodes[i].getPred())
        {
          std::cout << "looping shot";
        }
        i++;
      }
      else
      {
        delete[] dubinsNodes;
        return nullptr;
      }
    }
    // 返回末节点，通过getPred()可以找到前一节点。
    return &dubinsNodes[i - 1];
  }

  bool HybridAStar::calculatePath(
      const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal,
      int cellsX, int cellsY, std::vector<geometry_msgs::PoseStamped> &plan,
      ros::Publisher &pub, visualization_msgs::MarkerArray &pathNodes)
  {
#ifdef debug_mode
    ros::Time t0 = ros::Time::now();
#endif
    // 初始化优先队列，未来改善混合A*算法性能，这里使用的是二项堆优先队列
    boost::heap::binomial_heap<Node3D *, boost::heap::compare<CompareNodes>> openSet;
    // 初始化并创建一些参数。
    // 创建charMap,charMap中存储的是地图的障碍物信息
    const unsigned char *charMap = costmap->getCharMap();

    /****************************************************************
     * 分辨率太高会导致混合A*计算量过高，且意义不大, 因此
     * 这里把resolution写死，由于本人能力有限暂时此功，其实是无奈之举，
     * 未来需要在后续的代码中加入能够自动调节分辨率的功能，增加代码鲁棒性
     ****************************************************************/
    float resolution = 0.125; // costmap->getResolution()
    // unsigned int originalX, originalY, goalX, goalY;
    int cells_x, cells_y;
    cells_x = cellsX / 2.5;
    cells_y = cellsY / 2.5;
    int counter = 0;
    int dir;
    int iPred;
    float t, g;
    unsigned int dx, dy;
    unsigned int goal_x, goal_y, start_x, start_y;
    costmap->worldToMap(0, 0, dx, dy);
    dx = dx / 2.5;
    dy = dy / 2.5;
    // t为目标点朝向
    t = tf::getYaw(start.pose.orientation);
    Node3D *startPose = new Node3D(start.pose.position.x, start.pose.position.y, t, 0, 0, false, nullptr);

    costmap->worldToMap(goal.pose.position.x,
                        goal.pose.position.y, goal_x, goal_y);
    costmap->worldToMap(start.pose.position.x,
                        start.pose.position.y, start_x, start_y);

    //************************************************
    // 建立启发势场取决于地图大小,231(nodes/ms)
    std::unordered_map<int, std::shared_ptr<Node2D>> dp_map =
        grid_a_star_heuristic_generator_->GenerateDpMap(goal_x, goal_y, costmap);

#ifdef debug_mode
    ros::Time end_time = ros::Time::now();
    ros::Time t1 = ros::Time::now();
    ros::Duration d(end_time - t0);
    std::cout << "generate heuristic map costed " << d * 1000 << " ms" << std::endl;
#endif
    t = tf::getYaw(goal.pose.orientation);
    Node3D *goalPose = new Node3D(goal.pose.position.x, goal.pose.position.y, t, 999, 0, false, nullptr);
    std::unordered_map<int, Node3D *> open_set;
    std::unordered_map<int, Node3D *> closed_set;

    if (Constants::reverse)
    {
      dir = 6;
    }
    else
    {
      dir = 3;
    }
    open_set.emplace(startPose->getIdx(cells_x, Constants::headings, resolution, dx, dy), startPose);
    openSet.push(startPose);
    Node3D *tmpNode;
    Node3D *nSucc;

    while (openSet.size() && counter < Constants::iterations)
    {
      ++counter;
      // 根据混合A*算法，取堆顶的元素作为下查找节点
      tmpNode = openSet.top();

#ifdef SearchingVisulize
      publishSearchNodes(*tmpNode, pub, pathNodes, counter);
#endif

      openSet.pop(); // 出栈
      if (reachGoal(tmpNode, goalPose))
      {
#ifdef debug_mode
        ros::Time t1 = ros::Time::now();
        ros::Duration d(t1 - t0);
        std::cout << "got plan in ms: " << d * 1000 << std::endl;
#endif
        ROS_INFO("Got a plan,loop %d times", counter);
        nodeToPlan(tmpNode, plan);
        return true;
      }
      else
      {
        if (Constants::dubinsShot && tmpNode->isInRange(*goalPose) && !tmpNode->isReverse())
        {
          nSucc = dubinsShot(*tmpNode, *goalPose, costmap);
          // 如果Dubins方法能直接命中，即不需要进入Hybrid A*搜索了，直接返回结果
          if (nSucc != nullptr && reachGoal(nSucc, goalPose))
          {
#ifdef debug_mode
            ros::Time t1 = ros::Time::now();
            ros::Duration d(t1 - t0);
            std::cout << "got plan in ms: " << d * 1000 << std::endl;
#endif
            ROS_INFO("Got a plan,expored %d nodes ", counter);
            nodeToPlan(nSucc, plan);
            return true; // 如果下一步是目标点，可以返回了
          }
        }
        else if (Constants::reedsSheppShot && tmpNode->isInRange(*goalPose) && !tmpNode->isReverse())
        {
          nSucc = reedsSheppShot(*tmpNode, *goalPose, costmap);
          // 如果Dubins方法能直接命中，即不需要进入Hybrid A*搜索了，直接返回结果
          if (nSucc != nullptr && reachGoal(nSucc, goalPose))
          {
#ifdef debug_mode
            ros::Time t1 = ros::Time::now();
            ros::Duration d(t1 - t0);
            std::cout << "got plan in ms: " << d * 1000 << std::endl;
#endif
            ROS_INFO("Got a plan,expored %d nodes ", counter);
            nodeToPlan(nSucc, plan);
            return true; // 如果下一步是目标点，可以返回了
          }
        }
      }
      // 拓展tmpNode临时点目标周围的点，并且使用STL标准库的向量链表进行存储拓展点Node3D的指针数据
      std::vector<Node3D *> adjacentNodes = getAdjacentPoints(dir, cellsX, cellsY, charMap, tmpNode);
      // 将 tmpNode点在pathNode3D中映射的点加入闭集合中
      closed_set.emplace(tmpNode->getIdx(cells_x, Constants::headings, resolution, dx, dy), tmpNode);
      for (std::vector<Node3D *>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it)
      {
        // 使用stl标准库中的interator迭代器遍历相邻点
        Node3D *point = *it;
        iPred = point->getIdx(cells_x, Constants::headings, resolution, dx, dy);
        if (closed_set.find(iPred) != closed_set.end())
        {
          continue;
        }
        g = point->updateCostSofar();
        if (open_set.find(iPred) != open_set.end())
        {
          if (g < open_set[iPred]->getCostSofar())
          {
            point->setCostSofar(g);
            open_set[iPred]->setCostSofar(g);
            openSet.push(point); // 如果符合拓展点要求，则将此点加入优先队列中
          }
        }
        else
        {
          point->setCostSofar(g);
          open_set.emplace(iPred, point);
          costmap->worldToMap(point->getX(), point->getY(), start_x, start_y);
          updateH(*point, *goalPose, NULL, NULL, cells_x, cells_y, dp_map[start_y * cellsX + start_x]->getCostSofar() / 20);
          openSet.push(point); // 如果符合拓展点要求，则将此点加入优先队列中
        }
      }
    }
    return false;
  }

  std::vector<Node3D *> HybridAStar::getAdjacentPoints(int dir, int cells_x,
                                                       int cells_y, const unsigned char *charMap, Node3D *point)
  {
    std::vector<Node3D *> adjacentNodes;
    Node3D *tmpPtr;
    float xSucc;
    float ySucc;
    // float tSucc;
    unsigned int startX, startY;
    float t = point->getT();
    // int index;
    float x = point->getX();
    float y = point->getY();
    // unsigned int u32_x = int(x);
    // unsigned int u32_y = int(y);
    for (int i = 0; i < dir; i++)
    {
      if (i < 3)
      {
        xSucc = x + Constants::dx[i] * cos(t) - Constants::dy[i] * sin(t);
        ySucc = y + Constants::dx[i] * sin(t) + Constants::dy[i] * cos(t);
      }
      else
      {
        xSucc = x - Constants::dx[i - 3] * cos(t) - Constants::dy[i - 3] * sin(t);
        ySucc = y - Constants::dx[i - 3] * sin(t) + Constants::dy[i - 3] * cos(t);
      }
      if (costmap->worldToMap(xSucc, ySucc, startX, startY))
      {

        if (charMap[startX + startY * cells_x] < 250)
        {
          if (i < 3)
          {
            tmpPtr = new Node3D(xSucc, ySucc, t + Constants::dt[i], 999, 0, false, point);
            tmpPtr->setMapCost(charMap[startX + startY * cells_x]);
          }
          else
          {
            tmpPtr = new Node3D(xSucc, ySucc, t - Constants::dt[i - 3], 999, 0, true, point);
            tmpPtr->setMapCost(charMap[startX + startY * cells_x]);
          }
          adjacentNodes.push_back(tmpPtr);
        }
      }
    }

    return adjacentNodes;
  }

  bool HybridAStar::reachGoal(Node3D *node, Node3D *goalPose)
  {
    float nodeX = node->getX();
    float nodeY = node->getY();
    float goalX = goalPose->getX();
    float goalY = goalPose->getY();
    if ((nodeX < (goalX + point_accuracy) && nodeX > (goalX - point_accuracy)) &&
        (nodeY < (goalY + point_accuracy) && nodeY > (goalY - point_accuracy)))
    {
      if (node->getT() < (goalPose->getT() + theta_accuracy) &&
          node->getT() > (goalPose->getT() - theta_accuracy))
      {
        return true;
      }
    }
    return false;
  }

  int HybridAStar::calcIndix(float x, float y, int cells_x, float t)
  {
    return (int(x) * cells_x + int(y)) * Constants::headings + int(t / Constants::headings);
  }

  void HybridAStar::nodeToPlan(Node3D *node, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    Node3D *tmpPtr = node;
    geometry_msgs::PoseStamped tmpPose;
    std::vector<geometry_msgs::PoseStamped> replan;
    // float resolution = costmap->getResolution();
    // unsigned int originalX, originalY;
    tmpPose.header.stamp = ros::Time::now();
    // 参数后期处理，发布到RViz上进行可视化
    while (tmpPtr != nullptr)
    {
      tmpPose.pose.position.x = tmpPtr->getX();
      tmpPose.pose.position.y = tmpPtr->getY();
      tmpPose.header.frame_id = frame_id_;
      tmpPose.pose.orientation = tf::createQuaternionMsgFromYaw(tmpPtr->getT());
      replan.push_back(tmpPose);
      tmpPtr = tmpPtr->getPred();
    }
    int size = replan.size();
    for (int i = 0; i < size; ++i)
    {
      plan.push_back(replan[size - i - 1]);
    }
  }
}