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
    LOG(INFO) << "hybrid a star initializing";
    configuration_space_ptr_->UpdateGrid(map, true);
    map_width_ = configuration_space_ptr_->GetMap()->info.width;
    map_height_ = configuration_space_ptr_->GetMap()->info.height;

    lookup_table_ptr_->Initialize(map_width_, map_height_);

    nav_msgs::OccupancyGrid::Ptr a_star_map = TreatAstarMap(map);
    a_star_ptr_->Initialize(a_star_map);

    // LOG_IF(INFO, params_.adaptive_steering_angle_and_step_size) << "adaptive_steering_angle_and_step_size";
    // LOG_IF(INFO, !params_.adaptive_steering_angle_and_step_size) << "use fixed step size";
    LOG(INFO) << "hybrid a star initialized done.   ";
  }

  //###################################################
  //                                        3D A*
  //###################################################
  Utility::Path3D HybridAStar::GetPath(Node3D &start, Node3D &goal, Node3D *nodes3D, Node2D *nodes2D)
  {
    ros::Time t0 = ros::Time::now();
    path_.clear();
    // LOG(INFO) << "Hybrid A star start!!";
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

    astar_cost_map_ = BuildAStarCostMap(goal);
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

    iPred = start.setIdx(map_width_, map_height_, (float)2 * M_PI / params_.collision_detection_params.headings, resolution_, origin_x_, origin_y_);
    // LOG(INFO) << "index for start is " << iPred;
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
      // LOG(INFO) << "current node  is " << nPred->getX() << " " << nPred->getY() << " " << Utility::ConvertRadToDeg(nPred->getT());
      number_nodes_explored++;
      // set index
      iPred = nPred->setIdx(map_width_, map_height_, (float)2 * M_PI / params_.collision_detection_params.headings, resolution_, origin_x_, origin_y_);
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
          TracePath(nPred);
          Utility::RemoveDuplicates(path_);
          path_ = ShortCut(path_, true);
          ros::Time t1 = ros::Time::now();
          ros::Duration d(t1 - t0);
          LOG(INFO) << "max iterations reached!!!, current iterations is : " << iterations << " number of nodes explored is " << number_nodes_explored << " TIME in second: " << d;
          return path_;
        }
        else if (Utility::IsCloseEnough(*nPred, goal, params_.goal_range * resolution_, 2 * M_PI / params_.collision_detection_params.headings, true))
        {
          // DLOG(INFO) << "current node is " << nPred->getX() << " " << nPred->getY() << " " << Utility::ConvertRadToDeg(nPred->getT());
          // DLOG(INFO) << "goal is " << goal.getX() << " " << goal.getY() << " " << Utility::ConvertRadToDeg(goal.getT());
          // DLOG(INFO) << "Goal reached!!!";
          TracePath(nPred);
          Utility::RemoveDuplicates(path_);
          path_ = ShortCut(path_, true);
          ros::Time t1 = ros::Time::now();
          ros::Duration d(t1 - t0);
          LOG(INFO) << "number of nodes explored is " << number_nodes_explored << " TIME in second: " << d;
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
                LOG(INFO) << "Found path through analytical expansion. number of nodes explored is " << number_nodes_explored;
                TracePath(nPred);
                path_ = ShortCut(path_, true);
                analytical_expansion_index_ = path_.size();

                path_.insert(path_.end(), analytical_path.begin(), analytical_path.end());
                Utility::RemoveDuplicates(path_);

                // for (const Gggauto &element : path_)
                // {
                //   LOG(INFO) << "path node is " << element.getX() << " " << element.getY();
                // }
                if (params_.piecewise_cubic_bezier_interpolation)
                {
                  ConvertToPiecewiseCubicBezierPath();
                  piecewise_cubic_bezier_path_.insert(piecewise_cubic_bezier_path_.end(), analytical_path.begin(), analytical_path.end());
                  // DLOG(INFO) << "piecewise cubic bezier interpolation.";
                  // DLOG(INFO) << "piecewise_cubic_bezier_path_ size is " << piecewise_cubic_bezier_path_.size();
                  Utility::RemoveDuplicates(piecewise_cubic_bezier_path_);
                  ros::Time t1 = ros::Time::now();
                  ros::Duration d(t1 - t0);
                  // LOG(INFO) << "TIME in ms: " << d * 1000;
                  LOG(INFO) << "TIME in second: " << d;
                  return piecewise_cubic_bezier_path_;
                }

                ros::Time t1 = ros::Time::now();
                ros::Duration d(t1 - t0);
                // LOG(INFO) << "TIME in ms: " << d * 1000;
                LOG(INFO) << "TIME in second: " << d;
                return path_;
              }
            }
            else
            {
              if (analytical_expansion_counter == N)
              {
                // LOG(INFO) << "Start Analytic Expansion, every " << N << "th iterations";
                N = nPred->getCostToGo();
                analytical_expansion_counter = 0;
                Utility::Path3D analytical_path = AnalyticExpansions(*nPred, goal);
                if (analytical_path.size() != 0)
                {
                  LOG(INFO) << "Found path through analytical expansion, number of nodes explored is " << number_nodes_explored;
                  TracePath(nPred);
                  path_ = ShortCut(path_, true);
                  analytical_expansion_index_ = path_.size();
                  path_.insert(path_.end(), analytical_path.begin(), analytical_path.end());
                  // for (const auto &element : path_)
                  // {
                  //   LOG(INFO) << "path node is " << element.getX() << " " << element.getY();
                  // }
                  if (params_.piecewise_cubic_bezier_interpolation)
                  {
                    ConvertToPiecewiseCubicBezierPath();
                    piecewise_cubic_bezier_path_.insert(piecewise_cubic_bezier_path_.end(), analytical_path.begin(), analytical_path.end());
                    DLOG(INFO) << "piecewise cubic bezier interpolation.";
                    // DLOG(INFO) << "piecewise_cubic_bezier_path_ size is " << piecewise_cubic_bezier_path_.size();
                    Utility::RemoveDuplicates(piecewise_cubic_bezier_path_);
                    ros::Time t1 = ros::Time::now();
                    ros::Duration d(t1 - t0);
                    LOG(INFO) << "TIME in second: " << d;
                    return piecewise_cubic_bezier_path_;
                  }
                  Utility::RemoveDuplicates(path_);
                  ros::Time t1 = ros::Time::now();
                  ros::Duration d(t1 - t0);
                  LOG(INFO) << "TIME in second: " << d;
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
          // LOG(INFO) << "successor vec length is " << successor_vec.size();

          // ______________________________
          // SEARCH WITH FORWARD SIMULATION
          for (const auto &node : successor_vec)
          {
            // create possible successor
            nSucc = node;
            // LOG(INFO) << "current successor is " << nSucc->getX() << " " << nSucc->getY() << " " << Utility::ConvertRadToDeg(nSucc->getT());
            if (DuplicateCheck(openlist, node))
            {
              continue;
            }
            // set index of the successor
            iSucc = nSucc->setIdx(map_width_, map_height_, (float)2 * M_PI / params_.collision_detection_params.headings, resolution_, origin_x_, origin_y_);
            // ensure successor is on grid and traversable
            if (configuration_space_ptr_->IsTraversable(*nSucc))
            {
              // LOG(INFO) << "index is " << iSucc;
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
                    // LOG(INFO) << "successor total cost is " << nSucc->getTotalCost() << " pred total cost is " << nPred->getTotalCost();
                    // no need to reset tie-breaker
                    //  if the successor is in the same cell but the C value is larger
                    if (nSucc->getTotalCost() > nPred->getTotalCost() + params_.tie_breaker)
                    {
                      continue;
                    }
                    // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                    else if (nSucc->getTotalCost() <= nPred->getTotalCost() + params_.tie_breaker)
                    {
                      // LOG(INFO) << "same cell expansion, but successor has lower cost.";
                      nSucc->setSmartPtrPred(nPred->getSmartPtrPred());
                    }
                  }
                  // put successor on open list
                  nSucc->setOpenSet();
                  nodes3D[iSucc] = *nSucc;
                  std::shared_ptr<Node3D> nSucc_ptr = std::make_shared<Node3D>(nodes3D[iSucc]);
                  openlist.push(nSucc_ptr);
                  // LOG(INFO) << "put current node is " << nSucc->getX() << " " << nSucc->getY() << " " << Utility::ConvertRadToDeg(nSucc->getT()) << " into openlist!!";
                }
              }
            }
            else
            {
              // LOG(INFO) << "current node is " << nSucc->getX() << " " << nSucc->getY() << " " << Utility::ConvertRadToDeg(nSucc->getT()) << " is in collision!!";
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
    float curve_cost = 0, twoDCost = 0;
    // create a 2d start node
    Node2D start2d, goal2d;
    Utility::TypeConversion(start, start2d);
    Utility::TypeConversion(goal, goal2d);
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
      // LOG(INFO) << "dubins cost is " << curve_cost;
    }

    // if reversing is active use a
    else if (params_.collision_detection_params.curve_type == 1)
    {
      curve_cost = lookup_table_ptr_->GetReedsSheppCost(goal_rotated_translated);
      // LOG(INFO) << "rs cost is " << curve_cost;
    }
    else
    {
      curve_cost = lookup_table_ptr_->GetCubicBezierCost(goal_rotated_translated);
      // LOG(INFO) << "cubic bezier cost is " << curve_cost;
    }
    //  unconstrained with obstacles
    start2d.setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_);
    if (astar_cost_map_.find(start2d.getIdx()) != astar_cost_map_.end())
    {
      twoDCost = astar_cost_map_[start2d.getIdx()];
    }
    else
    {
      LOG(INFO) << "current start node is " << start2d.getX() << " " << start2d.getY() << " index is " << start2d.getIdx() << " can not be found in astar_cost_map_!!!";
    }

    // LOG(INFO) << "current node is " << start.getX() << " " << start.getY() << " " << start.getT() << " a star cost is " << twoDCost << " curve cost is " << curve_cost << " heuristic is " << std::max(curve_cost, twoDCost);
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
    Node3D node3d;
    int i = 0;
    float x = 0.f;
    Eigen::Vector3f vector3d_start, vector3d_goal;
    Utility::TypeConversion(start, vector3d_start);
    // DLOG(INFO) << "start point is " << vector3d_start.x() << " " << vector3d_start.y();
    Utility::TypeConversion(goal, vector3d_goal);
    std::shared_ptr<Node3D> pred_shared_ptr;
    // Dubins
    if (params_.collision_detection_params.curve_type == 0)
    {
      // start
      float q0[] = {start.getX(), start.getY(), start.getT()};
      // goal
      float q1[] = {goal.getX(), goal.getY(), goal.getT()};
      // initialize the path
      DubinsPath path;
      i = 0;
      // calculate the path
      dubins_init(q0, q1, params_.min_turning_radius, &path);
      float length = dubins_path_length(&path);
      while (x < length)
      {
        float q[3];
        dubins_path_sample(&path, x, q);
        node3d.setX(q[0]);
        node3d.setY(q[1]);
        node3d.setT(Utility::RadToZeroTo2P(q[2]));
        if (i > 0)
        {
          node3d.setSmartPtrPred(pred_shared_ptr);
        }
        else
        {
          node3d.setSmartPtrPred(std::make_shared<Node3D>(start));
        }
        // collision check
        if (configuration_space_ptr_->IsTraversable(node3d))
        {

          path_vec.emplace_back(node3d);
          x += params_.curve_step_size * resolution_;
          i++;
          pred_shared_ptr = std::make_shared<Node3D>(node3d);
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
    // RS curve
    if (params_.collision_detection_params.curve_type == 1)
    {
      ReedsSheppStateSpace rs_planner(params_.min_turning_radius);
      double length = -1;
      // start
      double q0[] = {start.getX(), start.getY(), start.getT()};
      // goal
      double q1[] = {goal.getX(), goal.getY(), goal.getT()};
      i = 0;
      std::vector<std::vector<double>> rs_path;
      rs_planner.sample(q0, q1, params_.curve_step_size * resolution_, length, rs_path);
      for (auto &point_itr : rs_path)
      {
        node3d.setX(point_itr[0]);
        node3d.setY(point_itr[1]);
        node3d.setT(Utility::RadToZeroTo2P(point_itr[2]));
        if (i > 0)
        {
          node3d.setSmartPtrPred(pred_shared_ptr);
        }
        else
        {
          node3d.setSmartPtrPred(std::make_shared<Node3D>(start));
        }
        // collision check
        if (configuration_space_ptr_->IsTraversable(node3d))
        {

          path_vec.emplace_back(node3d);
          i++;
          pred_shared_ptr = std::make_shared<Node3D>(node3d);
        }
        else
        {
          DLOG(INFO) << "RS shot collided, discarding the path";
          path_vec.clear();
          break;
        }
      }
    }
    // bezier curve
    if (params_.collision_detection_params.curve_type == 2)
    {
      CubicBezier::CubicBezier cubic_bezier(vector3d_start, vector3d_goal, map_width_, map_height_);
      float length = cubic_bezier.GetLength();
      i = 0;
      x = 0;
      path_vec.clear();
      while (x < length)
      {
        // DLOG(INFO) << i << "th iteration";

        Utility::TypeConversion(cubic_bezier.GetValueAt(x / length), node3d);
        float curvature = cubic_bezier.GetCurvatureAt(x / length);
        node3d.setT(Utility::RadToZeroTo2P(cubic_bezier.GetAngleAt(x / length)));
        // DLOG(INFO) << "current node is " << node3d.getX() << " " << node3d.getY();
        if (i > 0)
        {
          node3d.setSmartPtrPred(pred_shared_ptr);
        }
        else
        {
          node3d.setSmartPtrPred(std::make_shared<Node3D>(start));
        }
        // collision check
        if (configuration_space_ptr_->IsTraversable(node3d))
        {
          if (curvature <= 1 / params_.min_turning_radius)
          {
            path_vec.emplace_back(node3d);
            x += params_.curve_step_size * resolution_;
            i++;
            pred_shared_ptr = std::make_shared<Node3D>(node3d);
            // DLOG(INFO) << "current node is " << node3d.getX() << " " << node3d.getY();
            // LOG(INFO) << "current node is " << node3d.getX() << " " << node3d.getY() << " " << Utility::ConvertRadToDeg(node3d.getT());
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
      // LOG(INFO) << "start point before conversion is " << start.getX() << " " << start.getY() << " " << Utility::ConvertRadToDeg(start.getT()) << " after conversion is " << vector3d_start.x() << " " << vector3d_start.y() << " " << Utility::ConvertRadToDeg(vector3d_start.z());

      // LOG(INFO) << "goal point before conversion is " << goal.getX() << " " << goal.getY() << " " << Utility::ConvertRadToDeg(goal.getT()) << " after conversion is " << vector3d_goal.x() << " " << vector3d_goal.y() << " " << Utility::ConvertRadToDeg(vector3d_goal.z());
      // LOG(INFO) << "Analytical expansion connected, returning path";
      // LOG(INFO) << "analytical expansion start at " << start.getX() << " " << start.getY() << " " << Utility::ConvertRadToDeg(start.getT());
    }
    return path_vec;
  }

  // ###################################################
  //                                     create successor
  // ###################################################

  std::vector<std::shared_ptr<Node3D>> HybridAStar::CreateSuccessor(const Node3D &pred)
  {
    // LOG(INFO) << "CreateSuccessor in:";
    std::vector<std::shared_ptr<Node3D>> out;
    std::vector<std::pair<float, float>> available_steering_angle_and_step_size_vec;
    float distance_to_goal = Utility::GetDistance(pred, goal_);

    if (params_.adaptive_steering_angle_and_step_size)
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

        if (resolution_ > distance_to_goal)
        {
          pair.first = distance_to_goal;
        }
        else
        {
          pair.first = resolution_;
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
        available_steering_angle_and_step_size_vec = configuration_space_ptr_->FindStepSizeAndSteeringAngle(pred, start_, goal_, params_.number_of_successors, resolution_);
        // DLOG(INFO) << "CreateSuccessor out.";
        // LOG(INFO) << "adaptive step size;";
      }
    }
    else if (params_.adaptive_step_size)
    {
      // LOG(INFO) << "fixed steering angle and step size";
      // assume constant speed.

      std::vector<float> available_steering_angle_vec = Utility::FormSteeringAngleVec(params_.steering_angle, params_.number_of_successors);
      std::pair<float, float> pair;

      for (const auto &element : available_steering_angle_vec)
      {
        pair.first = configuration_space_ptr_->FindStepSize(pred, element, goal_, resolution_);
        pair.second = element;
        available_steering_angle_and_step_size_vec.emplace_back(pair);
      }
    }
    else
    {
      // LOG(INFO) << "fixed steering angle and step size";
      std::vector<float> available_steering_angle_vec = Utility::FormSteeringAngleVec(params_.steering_angle, params_.number_of_successors);
      std::pair<float, float> pair;
      // float step_size = resolution_;
      float step_size = 0.2;
      if (step_size > distance_to_goal)
      {
        pair.first = distance_to_goal;
      }
      else
      {
        pair.first = step_size;
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
    // }

    out = CreateSuccessor(pred, available_steering_angle_and_step_size_vec);
    if (params_.reverse)
    {
      std::vector<std::pair<float, float>> available_steering_angle_and_step_size_vec_reverse;
      std::vector<float> available_steering_angle_vec = Utility::FormSteeringAngleVec(params_.steering_angle, 1);
      std::pair<float, float> pair;
      float step_size = -0.2;
      if (abs(step_size) > distance_to_goal)
      {
        pair.first = -distance_to_goal;
      }
      else
      {
        pair.first = step_size;
      }

      for (const auto &element : available_steering_angle_vec)
      {
        pair.second = element;
        available_steering_angle_and_step_size_vec_reverse.emplace_back(pair);
      }
      std::vector<std::shared_ptr<Node3D>> reverse_successor_vec = CreateSuccessor(pred, available_steering_angle_and_step_size_vec_reverse);
      for (const auto &element : reverse_successor_vec)
      {
        out.emplace_back(element);
      }
    }
    // DLOG(INFO) << "CreateSuccessor out.";
    return out;
  }

  std::vector<std::shared_ptr<Node3D>> HybridAStar::CreateSuccessor(const Node3D &pred, const std::vector<std::pair<float, float>> &step_size_steering_angle_vec)
  {
    // LOG(INFO) << "in CreateSuccessor, current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT());
    std::vector<std::shared_ptr<Node3D>> out;
    float dx, dy, xSucc, ySucc, tSucc, turning_radius, steering_angle;
    int prem;
    bool reverse_flag;
    std::shared_ptr<Node3D> pred_smart_ptr = std::make_shared<Node3D>(pred);
    for (const auto &pair : step_size_steering_angle_vec)
    {
      // DLOG(INFO) << "in CreateSuccessor, current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT());
      if (pair.first < 0)
      {
        reverse_flag = true;
        // LOG(INFO) << "reverse";
      }
      else
      {
        reverse_flag = false;
        // LOG(INFO) << "forward";
      }

      if (abs(pair.first) <= 1e-3)
      {
        LOG(INFO) << "current step size is zero, no need to create successor!!";
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
        if (reverse_flag)
        {
          prem = 4;
          // DLOG(INFO) << "backward right";
        }
        else
        {
          prem = 1;
          // DLOG(INFO) << "forward right, dx " << dx << " dy " << dy;
        }
      }
      // left
      else if (steering_angle > 1e-3)
      {
        dx = turning_radius * sin(abs(steering_angle));
        dy = (turning_radius * (1 - cos(steering_angle)));
        if (reverse_flag)
        {
          prem = 5;
          // DLOG(INFO) << "backward left";
        }
        else
        {
          prem = 2;
          // DLOG(INFO) << "forward left, dx " << dx << " dy " << dy;}
        }
      }
      // straight forward,checked
      else
      {
        dx = pair.first;
        dy = 0;
        if (reverse_flag)
        {
          prem = 3;
          // DLOG(INFO) << "backward straight";
        }
        else
        {
          prem = 0;
          // DLOG(INFO) << "forward straight";}
        }
      }
      // DLOG(INFO) << "in CreateSuccessor, current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT());
      xSucc = pred.getX() + dx * cos(pred.getT()) - dy * sin(pred.getT());
      ySucc = pred.getY() + dx * sin(pred.getT()) + dy * cos(pred.getT());
      tSucc = Utility::RadToZeroTo2P(pred.getT() + steering_angle);
      // DLOG(INFO) << "in CreateSuccessor, current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT()) << " pred_ptr " << pred_ptr->getX() << " " << pred_ptr->getY() << " " << Utility::ConvertRadToDeg(pred_ptr->getT()) << " pred_smart_ptr " << pred_smart_ptr->getX() << " " << pred_smart_ptr->getY() << " " << Utility::ConvertRadToDeg(pred_smart_ptr->getT());
      std::shared_ptr<Node3D> temp = std::make_shared<Node3D>(Node3D(xSucc, ySucc, tSucc, pred.getCostSofar(), 0, reverse_flag, pred_smart_ptr, prem));
      if (configuration_space_ptr_->IsTraversable(temp))
      {
        out.emplace_back(temp);
        // DLOG(INFO) << "in CreateSuccessor, successor is " << temp->getX() << " " << temp->getY() << " " << Utility::ConvertRadToDeg(temp->getT()) << " temp pred is " << temp->getPred()->getX() << " " << temp->getPred()->getY() << " " << Utility::ConvertRadToDeg(temp->getPred()->getT()) << " temp smart ptr " << temp->getSmartPtrPred()->getX() << " " << temp->getSmartPtrPred()->getY() << " " << Utility::ConvertRadToDeg(temp->getSmartPtrPred()->getT()) << " step size is " << pair.first << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
        // DLOG_IF(FATAL, (*temp == *temp->getSmartPtrPred())) << "successor and current are the same node!!! step size is " << pair.first << " steering angle is " << Utility::ConvertRadToDeg(steering_angle) << " successor is " << xSucc << " " << ySucc << " " << Utility::ConvertRadToDeg(tSucc) << " current node is " << pred.getX() << " " << pred.getY() << " " << Utility::ConvertRadToDeg(pred.getT());
      }
      else
      {
        // LOG(INFO) << "in CreateSuccessor, successor: " << temp->getX() << " " << temp->getY() << " " << Utility::ConvertRadToDeg(temp->getT()) << " is in collision, step size is " << pair.first << " steering angle is " << Utility::ConvertRadToDeg(steering_angle);
      }
    }
    // LOG_IF(WARNING, out.size() == 0) << "WARNING: Number of successor is zero!!!!";
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
      // LOG(INFO) << "current node is " << node3d_ptr->getX() << " " << node3d_ptr->getY() << " and its pred is " << node3d_ptr->getSmartPtrPred()->getX() << " " << node3d_ptr->getSmartPtrPred()->getY();
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
    for (uint index = 1; index < path_.size() - 1; ++index)
    {
      // DLOG(INFO) << "path point is " << path_[index].getX() << " " << path_[index].getY() << " " << Utility::ConvertRadToDeg(path_[index].getT());
      // not necessary to do this check, greater than 1
      if (index == analytical_expansion_index_)
      {
        // DLOG(INFO) << "distance between path node is smaller than 1, set it to end point.";
        // DLOG(INFO) << "current path index is " << index;
        end_point.setX(path_[index - 1].getX());
        end_point.setY(path_[index - 1].getY());
        end_point.setT(Utility::RadToZeroTo2P(path_[index - 1].getT()));
        break;
      }
      if (Utility::GetDistance(path_[index], path_[index + 1]) < resolution_)
      {
        continue;
        // DLOG(INFO) << "points are too close to each other!!!!";
      }
        Eigen::Vector3f vector_3d;
        Utility::TypeConversion(path_[index], vector_3d);
        anchor_points_vec.emplace_back(vector_3d);
        // DLOG(INFO) << "current path index is " << index;
    }
    Utility::TypeConversion(end_point, end_point_vec);
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
      // this 0.2 should be a resolution related value, no, unit here are meters, so consider vehicle control p
      if (abs(element->getX() - node_3d_ptr->getX()) < 0.05 && abs(element->getY() - node_3d_ptr->getY()) < 0.05 && abs(element->getT() - node_3d_ptr->getT()) < Utility::ConvertDegToRad(3))
      {
        // LOG(INFO) << "node " << node_3d_ptr->getX() << " " << node_3d_ptr->getY() << " " << Utility::ConvertRadToDeg(node_3d_ptr->getT()) << " is already in open list. closest node is " << element->getX() << " " << element->getY() << " " << Utility::ConvertRadToDeg(element->getT());
        return true;
      }
    }
    return false;
  }

  bool HybridAStar::calculatePath(
      const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal,
      int cellsX, int cellsY, std::vector<geometry_msgs::PoseStamped> &plan,
      ros::Publisher &pub, visualization_msgs::MarkerArray &pathNodes)
  {
    // DLOG(INFO) << "in HybridAStar::calculatePath.";
    Utility::TypeConversion(start, start_);
    Utility::TypeConversion(goal, goal_);
    Node2D *nodes2D = new Node2D[cellsX * cellsY]();
    Node3D *nodes3D = new Node3D[cellsX * cellsY * params_.collision_detection_params.headings + cellsY * cellsX + cellsX]();
    Utility::Path3D path3d = GetPath(start_, goal_, nodes3D, nodes2D);
    Utility::TypeConversion(path3d, plan);
    if (plan.size() != 0)
    {
      delete[] nodes2D;
      delete[] nodes3D;
      return true;
    }
    else
    {
      delete[] nodes2D;
      delete[] nodes3D;
      return false;
    }
  }

  std::unordered_map<int, float> HybridAStar::BuildAStarCostMap(const Node3D &start)
  {
    // LOG(INFO) << "in BuildAStarCostMap.";
    std::unordered_map<int, float> astar_cost_map_;
    Node2D start2d;
    Utility::TypeConversion(start, start2d);

    std::priority_queue<std::pair<int, double>,
                        std::vector<std::pair<int, double>>, cmp>
        open_pq;
    std::unordered_map<int, std::shared_ptr<Node2D>> open_set;
    std::shared_ptr<Node2D> goal_node = std::make_shared<Node2D>(start2d);

    open_set.emplace(goal_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_), goal_node);
    open_pq.emplace(goal_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_), goal_node->getCostSofar());
    while (!open_pq.empty())
    {
      int id = open_pq.top().first;
      open_pq.pop();
      std::shared_ptr<Node2D> current_node = open_set[id];

      a_star_ptr_->UpdateCostSoFar(*current_node);

      astar_cost_map_.emplace(current_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_), current_node->getCostSofar());
      // LOG(INFO) << "put current node " << current_node->getX() << " " << current_node->getY() << " index is " << current_node->getIdx() << " cost is " << current_node->getCostSofar();
      std::vector<std::shared_ptr<Node2D>> adjacent_nodes =
          a_star_ptr_->CreateSuccessor(*current_node, 8);
      for (std::vector<std::shared_ptr<Node2D>>::iterator
               it = adjacent_nodes.begin();
           it != adjacent_nodes.end(); ++it)
      {
        std::shared_ptr<Node2D> next_node = *it;
        if (astar_cost_map_.find(next_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_)) != astar_cost_map_.end())
        {
          continue;
        }
        if (open_set.find(next_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_)) != open_set.end())
        {
          if (open_set[next_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_)]->getCostSofar() > next_node->getCostSofar())
          {
            open_set[next_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_)]->setCostSofar(next_node->getCostSofar());
            open_set[next_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_)]->setSmartPtrPred(current_node);
          }
        }
        else
        {
          next_node->setSmartPtrPred(current_node);
          open_set.emplace(next_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_), next_node);
          open_pq.emplace(next_node->setIdx(map_width_, map_height_, resolution_, origin_x_, origin_y_), next_node->getCostSofar());
        }
      }
    }

    return astar_cost_map_;
  }

  Utility::Path3D HybridAStar::ShortCut(const Utility::Path3D &path, bool consider_steering_angle_limit)
  {
    Utility::Path3D out, input = path;
    // start from last point of path, check if segment between end point and path[n-1] is in collision, if not, then check end point and path[n-2] is in collision
    Node3D current_point, previous_point;
    float steering_angle;
    previous_point = input[input.size() - 2];
    for (uint index1 = input.size() - 1; index1 >= 0;)
    {
      // LOG(INFO) << "index1 is " << index1;
      current_point = input[index1];
      out.emplace_back(current_point);
      for (uint index2 = index1 - 1; index2 >= 0; index2--)
      {
        // LOG(INFO) << "index2 is " << index2;
        if (configuration_space_ptr_->IsTraversable(current_point, input[index2]))
        {
          // LOG(INFO) << "pass collision check, index2 is " << index2;
          previous_point = input[index2];
          if (consider_steering_angle_limit)
          {
            steering_angle = Utility::FindSteeringAngle(previous_point, current_point);
            if (abs(steering_angle) < params_.collision_detection_params.max_steering_angle)
            {
              if (index2 == 0)
              {
                // LOG(INFO) << "index2 is zero, break;";
                index1 = 0;
                out.emplace_back(previous_point);
                break;
              }
              // LOG(INFO) << "steering angle is inside limit, continue.";
              continue;
            }
            else
            {
              // LOG(INFO) << "steering angle is outside limit.";
              out.emplace_back(previous_point);
              if (index2 == 0)
              {
                // LOG(INFO) << "index2 is zero, break;";
                index1 = 0;
              }
              else
              {
                // LOG(INFO) << "index2 is zero, break;";
                index1 = index2 - 1;
              }
              break;
            }
          }

          // end check
          if (index2 == 0)
          {
            // LOG(INFO) << "index2 is zero, break;";
            out.emplace_back(previous_point);
          }
        }
        else
        {
          // LOG(INFO) << "in collision;";
          out.emplace_back(previous_point);
          if (index2 == 0)
          {
            // LOG(INFO) << "index2 is zero, break;";
            index1 = 0;
          }
          else
          {
            // LOG(INFO) << "index2 is zero, break;";
            index1 = index2 - 1;
          }
          break;
        }
        if (index2 == 0)
        {
          // LOG(INFO) << "index2 is zero, break;";
          index1 = 0;
          break;
        }
      }
      if (index1 == 0)
      {
        // LOG(INFO) << "index2 is zero, break;";
        break;
      }
    }
    // if (input == out)
    // {
    //   LOG(INFO) << "index2 is zero, break;";
    //   break;
    // }
    // else
    // {
    //   LOG(INFO) << "index2 is zero, break;";
    //   input = out;
    //   out.clear();
    // }

    // LOG(INFO) << "index2 is zero, break;";
    std::reverse(out.begin(), out.end());
    return out;
  }
}