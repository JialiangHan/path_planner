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
    int N = start.GetCostToGo();
    // mark start as open
    start.open();
    // push on priority queue aka open list
    std::shared_ptr<Node3D> start_ptr = std::make_shared<Node3D>(start);
    openlist.push(start_ptr);

    iPred = start.SetIdx(map_width_, map_height_, (float)2 * M_PI / params_.headings);
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
      iPred = nPred->SetIdx(map_width_, map_height_, (float)2 * M_PI / params_.headings);
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
          LOG(INFO) << "number of nodes explored is " << number_nodes_explored;
          // return nPred;
          TracePath(nPred);
          return path_;
        }
        else if (Utility::IsCloseEnough(*nPred, goal, params_.goal_range, 2 * M_PI / params_.headings, true))
        {
          // DLOG(INFO) << "current node is " << nPred->GetX() << " " << nPred->GetY() << " " << Utility::ConvertRadToDeg(nPred->GetT());
          // DLOG(INFO) << "goal is " << goal.GetX() << " " << goal.GetY() << " " << Utility::ConvertRadToDeg(goal.GetT());
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
                N = nPred->GetCostToGo();
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
            iSucc = nSucc->SetIdx(map_width_, map_height_, (float)2 * M_PI / params_.headings);
            // ensure successor is on grid and traversable
            if (configuration_space_ptr_->IsTraversable(nSucc))
            {
              // DLOG(INFO) << "index is " << iSucc;
              // ensure successor is not on closed list or it has the same index as the predecessor
              if (!nodes3D[iSucc].isClosed())
              {
                // calculate new G value
                UpdateCostSoFar(*nSucc, params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
                // nSucc->updateG(params_.penalty_turning, params_.penalty_change_of_direction, params_.penalty_reverse);
                newG = nSucc->GetCostSofar();
                // if successor not on open list or found a shorter way to the cell
                if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].GetCostSofar())
                {
                  // calculate H value
                  UpdateHeuristic(*nSucc, goal, nodes2D);
                  // same cell expansion
                  if (iPred == iSucc)
                  {
                    DLOG(INFO) << "successor total cost is " << nSucc->GetTotalCost() << " pred total cost is " << nPred->GetTotalCost();
                    // no need to reset tie-breaker
                    //  if the successor is in the same cell but the C value is larger
                    if (nSucc->GetTotalCost() > nPred->GetTotalCost() + params_.tie_breaker)
                    {
                      continue;
                    }
                    // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                    else if (nSucc->GetTotalCost() <= nPred->GetTotalCost() + params_.tie_breaker)
                    {
                      DLOG(INFO) << "same cell expansion, but successor has lower cost.";
                      nSucc->SetPred(nPred->GetPred());
                    }
                  }
                  // put successor on open list
                  nSucc->open();
                  nodes3D[iSucc] = *nSucc;
                  std::shared_ptr<Node3D> nSucc_ptr = std::make_shared<Node3D>(nodes3D[iSucc]);
                  openlist.push(nSucc_ptr);
                }
              }
            }
            else
            {
              // DLOG(INFO) << "current node is " << nSucc->GetX() << " " << nSucc->GetY() << " " << Utility::ConvertRadToDeg(nSucc->GetT()) << " is in collision!!";
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
    goal_rotated_translated.SetX(abs(goal.GetX() - start.GetX()));
    goal_rotated_translated.SetY(abs(goal.GetY() - start.GetY()));
    goal_rotated_translated.SetT(Utility::RadToZeroTo2P(goal.GetT() - start.GetT()));
    // DLOG(INFO) << "goal is " << goal.GetX() << " " << goal.GetY() << " " << Utility::ConvertRadToDeg(goal.GetT());
    // DLOG(INFO) << "start is " << start.GetX() << " " << start.GetY() << " " << Utility::ConvertRadToDeg(start.GetT());
    // DLOG(INFO) << "goal rotated is " << goal_rotated_translated.GetX() << " " << goal_rotated_translated.GetY() << " " << Utility::ConvertRadToDeg(goal_rotated_translated.GetT());
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
    if (!nodes2D[(int)start.GetY() * map_width_ + (int)start.GetX()].isDiscovered())
    {
      // ros::Time t0 = ros::Time::now();
      // create a 2d start node
      Node2D start2d(start.GetX(), start.GetY(), 0, 0, nullptr);
      // create a 2d goal node
      Node2D goal2d(goal.GetX(), goal.GetY(), 0, 0, nullptr);
      // DLOG(INFO) << "A star start node is " << start2d.GetX() << " " << start2d.GetY() << " goal is " << goal2d.GetX() << " " << goal2d.GetY();

      // run 2d AStar and return the cost of the cheapest path for that node
      nodes2D[(int)start.GetY() * map_width_ + (int)start.GetX()].SetG(a_star_ptr_->GetAStarCost(nodes2D, start2d, goal2d, true));
      // ros::Time t1 = ros::Time::now();
      // ros::Duration d(t1 - t0);
      // DLOG(INFO) << "calculated 2D Heuristic in ms: " << d * 1000;
    }

    // offset for same node in cell
    twoDoffset = sqrt(((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) * ((start.GetX() - (long)start.GetX()) - (goal.GetX() - (long)goal.GetX())) +
                      ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())) * ((start.GetY() - (long)start.GetY()) - (goal.GetY() - (long)goal.GetY())));
    // DLOG(INFO) << "twoD cost before offset is " << nodes2D[(int)start.GetY() * map_width_ + (int)start.GetX()].GetCostSofar() << " two d offset is " << twoDoffset;
    twoDCost = nodes2D[(int)start.GetY() * map_width_ + (int)start.GetX()].GetCostSofar() - twoDoffset;

    // DLOG(INFO) << "current node is " << start.GetX() << " " << start.GetY() << " " << start.GetT() << " a star cost is " << twoDCost << " curve cost is " << curve_cost << " heuristic is " << std::max(curve_cost, twoDCost);
    // DLOG(INFO) << "rs cost is " << curve_cost;
    // return the maximum of the heuristics, making the heuristic admissable
    start.SetH(std::max(curve_cost, twoDCost));
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
      float q0[] = {start.GetX(), start.GetY(), start.GetT()};
      // goal
      float q1[] = {goal.GetX(), goal.GetY(), goal.GetT()};
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
        node3d.SetX(q[0]);
        node3d.SetY(q[1]);
        node3d.SetT(Utility::RadToZeroTo2P(q[2]));
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
        node3d.SetT(cubic_bezier.GetAngleAt(x / length));
        // DLOG(INFO) << "current node is " << node3d.GetX() << " " << node3d.GetY();
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
      LOG(INFO) << "analytical expansion start at " << start.GetX() << " " << start.GetY() << " " << Utility::ConvertRadToDeg(start.GetT());
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

      // DLOG(INFO) << "current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT());
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
      xSucc = pred.GetX() + dx * cos(pred.GetT()) - dy * sin(pred.GetT());
      ySucc = pred.GetY() + dx * sin(pred.GetT()) + dy * cos(pred.GetT());
      tSucc = Utility::RadToZeroTo2P(pred.GetT() + steering_angle);
      // DLOG_IF(INFO, (pair.first < 1) || (steering_angle > Utility::ConvertDegToRad(30)) || (steering_angle < -Utility::ConvertDegToRad(30))) << "in create successor, current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT()) << " step size is " << pair.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(steering_angle) << " successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
      // DLOG_IF(INFO, (xSucc > 77) && (xSucc < 79) && (ySucc > 1) && (ySucc < 3)) << "in create successor, current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT()) << " step size is " << pair.first << " current steering angle is in DEG: " << Utility::ConvertRadToDeg(steering_angle) << " successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
      std::shared_ptr<Node3D> temp = std::make_shared<Node3D>(Node3D(xSucc, ySucc, tSucc, pred.GetCostSofar(), 0, pred_ptr, prem));
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
        xSucc = pred.GetX() - dx * cos(pred.GetT()) - dy * sin(pred.GetT());
        ySucc = pred.GetY() - dx * sin(pred.GetT()) + dy * cos(pred.GetT());
        tSucc = Utility::RadToZeroTo2P(pred.GetT() - steering_angle);
        // DLOG(INFO) << "successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc);
        std::shared_ptr<Node3D> temp = std::make_shared<Node3D>(Node3D(xSucc, ySucc, tSucc, pred.GetCostSofar(), 0, pred_ptr, prem));
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
    float step_size = Utility::GetDistance(node, *node.GetPred());
    float pred_cost_so_far = node.GetCostSofar();
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
    //   DLOG(INFO) << "path point is " << path_[jj].GetX() << " " << path_[jj].GetY() << " " << path_[jj].GetT();
    //   DLOG(INFO) << "distance to next point is " << Utility::GetDistance(path_[jj], path_[jj + 1]);
    // }
    float distance_to_prev, distance_to_succ;
    for (uint index = 1; index < path_.size() - 1; ++index)
    {
      // DLOG(INFO) << "path point is " << path_[index].GetX() << " " << path_[index].GetY() << " " << Utility::ConvertRadToDeg(path_[index].GetT());
      // not necessary to do this check, greater than 1
      bool flag = true;
      if (flag)
      {
        if (index == analytical_expansion_index_)
        {
          // DLOG(INFO) << "distance between path node is smaller than 1, set it to end point.";
          // DLOG(INFO) << "current path index is " << index;
          end_point.SetX(path_[index - 1].GetX());
          end_point.SetY(path_[index - 1].GetY());
          end_point.SetT(path_[index - 1].GetT());
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
          // DLOG(INFO) << "anchor points is " << path_[index].GetX() << " " << path_[index].GetY() << " " << Utility::ConvertRadToDeg(path_[index].GetT());
        }
        else if (distance_to_succ >= 1 && distance_to_prev < 1)
        {
          continue;
        }
        else if (distance_to_prev >= 1 && distance_to_succ < 1)
        {
          anchor_points_vec.emplace_back(vector_3d);
          // DLOG(INFO) << "anchor points is " << path_[index].GetX() << " " << path_[index].GetY() << " " << Utility::ConvertRadToDeg(path_[index].GetT());
        }
        if (index == analytical_expansion_index_)
        {
          DLOG(INFO) << "distance between path node is smaller than 1, set it to end point.";
          DLOG(INFO) << "current path index is " << index;
          end_point.SetX(path_[index].GetX());
          end_point.SetY(path_[index].GetY());
          end_point.SetT(path_[index].GetT());
          break;
        }
      }
    }
    // DLOG(INFO) << "end point is " << end_point.GetX() << " " << end_point.GetY() << " " << Utility::ConvertRadToDeg(end_point.GetT());
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
      // DLOG(INFO) << "point is " << node3d.GetX() << " " << node3d.GetY() << " " << Utility::ConvertRadToDeg(node3d.GetT());
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
    float angle_to_goal = -Utility::RadNormalization(pred.GetT() - goal_.GetT());
    bool flag = true;
    if (flag)
    {
      angle_to_goal = -Utility::RadNormalization(pred.GetT() - Utility::GetAngle(pred, goal_));
      // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(Utility::RadNormalization(Utility::GetAngle(pred, goal_)));
      // DLOG(INFO) << "steering angle is " << Utility::ConvertRadToDeg(angle_to_goal);
    }
    // DLOG(INFO) << "current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT()) << " and goal orientation is " << Utility::ConvertRadToDeg(goal_.GetT());
    // DLOG(INFO) << "angle to goal is " << Utility::ConvertRadToDeg(angle_to_goal);
    // 4. if angle to goal is in the steering angle range(current orientation +-30deg), then make it steering angle, otherwise 30 or -30 to make angle to goal smaller

    if (std::abs(angle_to_goal) > Utility::ConvertDegToRad(30))
    {
      // if (pred.GetT() > Utility::RadNormalization(goal_.GetT()) && pred.GetT() < Utility::RadNormalization((goal_.GetT() + Utility::ConvertDegToRad(180))))
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

    DLOG(INFO) << "current node is " << pred.GetX() << " " << pred.GetY() << " " << Utility::ConvertRadToDeg(pred.GetT()) << " and goal orientation is " << Utility::ConvertRadToDeg(goal_.GetT()) << " one more step size is " << step_size << " and steering angle pair is " << Utility::ConvertRadToDeg(steering_angle);

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
      if (abs(element->GetX() - node_3d_ptr->GetX()) < 0.2 && abs(element->GetY() - node_3d_ptr->GetY()) < 0.2 && abs(element->GetT() - node_3d_ptr->GetT()) < Utility::ConvertDegToRad(3))
      {
        // LOG(INFO) << "node already in open list.";
        return true;
      }
    }
    return false;
  }
}