#include "planner.h"

using namespace HybridAStar;

//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner()
{
  // load all params
  param_manager_.reset(new ParameterManager(nh_));
  param_manager_->LoadParams();
  params_ = param_manager_->GetPlannerParams();
  smoother_ptr_.reset(new Smoother(param_manager_->GetSmootherParams()));
  path_publisher_ptr_.reset(new PathPublisher(param_manager_->GetPathPublisherParams()));
  smoothed_path_publisher_ptr_.reset(new PathPublisher(param_manager_->GetPathPublisherParams(), true));

  visualization_ptr_.reset(new Visualize(param_manager_->GetVisualizeParams()));

  if (params_.use_rrt)
  {
    rrt_planner_ptr_.reset(new RRTPlanner::RRTPlanner(param_manager_->GetRRTPlannerParams(), visualization_ptr_));
  }
  else if (params_.use_a_star)
  {
    a_star_planner_ptr_.reset(new AStar(param_manager_->GetAStarPlannerParams(), visualization_ptr_));
  }

  else
  {
    hybrid_a_star_ptr_.reset(new HybridAStar(param_manager_->GetHybridAStarParams(), visualization_ptr_));
  }

  // _________________
  // TOPICS TO PUBLISH
  pub_start_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (params_.manual)
  {
    sub_map_ = nh_.subscribe("/map", 1, &Planner::SetMap, this);
  }
  else
  {
    sub_map_ = nh_.subscribe("/occ_map", 1, &Planner::SetMap, this);
  }

  sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &Planner::SetGoal, this);
  sub_start_ = nh_.subscribe("/initialpose", 1, &Planner::SetStart, this);
  DLOG(INFO) << "Initialized finished planner!!";
};

//###################################################
//                                                MAP
//###################################################
void Planner::SetMap(const nav_msgs::OccupancyGrid::Ptr map)
{
  DLOG(INFO) << "set map.";
  grid_ = map;

  if (params_.use_rrt)
  {
    rrt_planner_ptr_->Initialize(map);
  }
  else if (params_.use_a_star)
  {
    a_star_planner_ptr_->Initialize(map);
  }
  else
  {
    hybrid_a_star_ptr_->Initialize(map);
    // create array for Voronoi diagram
    ros::Time t0 = ros::Time::now();
    int height = map->info.height;
    int width = map->info.width;
    bool **binMap;
    binMap = new bool *[width];

    for (int x = 0; x < width; x++)
    {
      binMap[x] = new bool[height];
    }

  for (int x = 0; x < width; ++x)
  {
    for (int y = 0; y < height; ++y)
    {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoi_diagram_.initializeMap(width, height, binMap);
  voronoi_diagram_.update();
  voronoi_diagram_.visualize();
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  DLOG(INFO) << "created Voronoi Diagram in ms: " << d * 1000;
  }

  // plan if the switch is not set to manual and a transform is available
  if (!params_.manual && listener_.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr))
  {

    listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_);

    // assign the values to start from base_link
    start_.pose.pose.position.x = transform_.getOrigin().x();
    start_.pose.pose.position.y = transform_.getOrigin().y();
    tf::quaternionTFToMsg(transform_.getRotation(), start_.pose.pose.orientation);

    if (grid_->info.height >= start_.pose.pose.position.y && start_.pose.pose.position.y >= 0 &&
        grid_->info.width >= start_.pose.pose.position.x && start_.pose.pose.position.x >= 0)
    {
      // set the start as valid and plan
      valid_start_ = true;
    }
    else
    {
      valid_start_ = false;
    }

    MakePlan();
  }
  if (params_.fix_start_goal)
  {
    DLOG(INFO) << "in set map, start to make plan.";
    MakePlan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::SetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial)
{
  float x = initial->pose.pose.position.x / params_.cell_size;
  float y = initial->pose.pose.position.y / params_.cell_size;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  DLOG(INFO) << "I am seeing a new start x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t) << " deg";

  if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0)
  {
    valid_start_ = true;
    start_ = *initial;

    if (params_.manual)
    {
      DLOG(INFO) << "in SetStart, start to make plan.";
      MakePlan();
    }

    // publish start for RViz
    pub_start_.publish(startN);
  }
  else
  {
    DLOG(INFO) << "invalid start x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::SetGoal(const geometry_msgs::PoseStamped::ConstPtr &end)
{
  // retrieving goal position
  float x = end->pose.position.x / params_.cell_size;
  float y = end->pose.position.y / params_.cell_size;
  float t = tf::getYaw(end->pose.orientation);

  DLOG(INFO) << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t) << " deg";

  if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0)
  {
    valid_goal_ = true;
    goal_ = *end;

    if (params_.manual)
    {
      DLOG(INFO) << "in SetGoal, start to make plan.";
      MakePlan();
    }
  }
  else
  {
    DLOG(INFO) << "invalid goal x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::MakePlan()
{
  if (grid_ == nullptr)
  {
    DLOG(WARNING) << "map not initialized";
    return;
  }

  // if a start as well as goal are defined go ahead and plan
  if ((valid_start_ && valid_goal_) || params_.fix_start_goal)
  {
    DLOG(INFO) << "valid start and valid goal, start to make plan!";

    // LISTS ALLOCATED ROW MAJOR ORDER
    int width = grid_->info.width;
    int height = grid_->info.height;
    int depth = params_.headings;
    int length = width * height * depth + height * width + width;
    DLOG(INFO) << "map size is " << width << " " << height << " length is " << length;
    // define list pointers and initialize lists
    Node3D *nodes3D = new Node3D[length]();
    Node2D *nodes2D = new Node2D[width * height]();

    float x, y, t;
    Node3D nStart, nGoal;
    // set theta to a value (0,2PI]
    if (params_.fix_start_goal)
    {
      x = 2;
      y = height - 2;
      t = Utility::ConvertDegToRad(0);
      t = Utility::RadToZeroTo2P(t);
      nStart.setX(x);
      nStart.setY(y);
      nStart.setT(t);
      // set theta to a value (0,2PI]
      t = Utility::RadToZeroTo2P(t);
      x = width - 2;
      y = 2;
      t = Utility::ConvertDegToRad(0);
      nGoal.setX(x);
      nGoal.setY(y);
      nGoal.setT(t);
    }
    else
    {
      x = start_.pose.pose.position.x / params_.cell_size;
      y = start_.pose.pose.position.y / params_.cell_size;
      t = tf::getYaw(start_.pose.pose.orientation);
      nStart.setX(x);
      nStart.setY(y);
      nStart.setT(t);
      // retrieving goal position
      x = goal_.pose.position.x / params_.cell_size;
      y = goal_.pose.position.y / params_.cell_size;
      t = tf::getYaw(goal_.pose.orientation);
      nGoal.setX(x);
      nGoal.setY(y);
      nGoal.setT(t);
    }

    std::srand(0);
    Clear();
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();
    Utility::Path3D path, temp;
    if (params_.use_rrt)
    {
      LOG(INFO) << "Use RRT!";
      path = rrt_planner_ptr_->GetPath(nStart, nGoal);
      // if (true)
      // {
      //   // temp = rrt_planner_ptr_->ShortCut(path,false);
      //   // path = rrt_planner_ptr_->PiecewiseCubicBezier(temp);
      //   // path = rrt_planner_ptr_->ShortCut(path, false);
      //   // path = rrt_planner_ptr_->ShortCut(path,false);
      // }
    }
    else if (params_.use_a_star)
    {
      LOG(INFO) << "Use A star!";
      path = a_star_planner_ptr_->GetPath(nStart, nGoal, nodes2D);
    }
    else
    {
      // FIND THE PATH
      LOG(INFO) << "Use hybrid a star!";
      path = hybrid_a_star_ptr_->GetPath(nStart, nGoal, nodes3D, nodes2D);
    }
    LOG(INFO) << "path length is " << Utility::GetLength(path);
    //  for (const auto &node : path)
    //  {
    //    DLOG(INFO) << "node in path is " << node.getX() << " " << node.getY() << " " << node.getT();
    //  }
    //  set path
    smoother_ptr_->SetPath(path);
    // CREATE THE UPDATED PATH
    path_publisher_ptr_->UpdatePath(smoother_ptr_->GetPath());
    // DLOG(INFO) << "path before smooth size is " << smoother_ptr_->GetPath().size();
    // SMOOTH THE PATH
    if (params_.smooth)
    {
      smoother_ptr_->SmoothPath(voronoi_diagram_);
      // CREATE THE UPDATED PATH
      // DLOG(INFO) << "smoothed path size is " << smoother_ptr_->GetPath().size();
      smoothed_path_publisher_ptr_->UpdatePath(smoother_ptr_->GetPath());
    }
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    // LOG(INFO) << "TIME in ms: " << d * 1000;
    LOG(INFO) << "TIME in second: " << d;
    Publish(nodes3D, nodes2D, width, height, depth);

    delete[] nodes3D;
    delete[] nodes2D;
    // set these two flag to false when finished planning to avoid unwanted planning
    valid_start_ = false;
    valid_goal_ = false;
  }
  else if (valid_start_ && !valid_goal_)
  {
    DLOG(INFO) << "missing goal!!!";
  }
  else if (!valid_start_ && valid_goal_)
  {
    DLOG(INFO) << "missing start!!!";
  }
  else
  {
    DLOG(INFO) << "missing start and goal!!!";
  }
}

void Planner::MakePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
  // if a start as well as goal are defined go ahead and plan

  DLOG(INFO) << "valid start and valid goal, start to make plan!";
  // LISTS ALLOCATED ROW MAJOR ORDER
  int width = grid_->info.width;
  int height = grid_->info.height;
  int depth = params_.headings;
  int length = width * height * depth + height * width + width;
  DLOG(INFO) << "map size is " << width << " " << height << " length is " << length;
  // define list pointers and initialize lists
  Node3D *nodes3D = new Node3D[length]();
  Node2D *nodes2D = new Node2D[width * height]();

  float x, y, t;
  Node3D nStart, nGoal;
  // set theta to a value (0,2PI]
  if (params_.fix_start_goal)
  {
    x = 2;
    y = height - 2;
    t = Utility::ConvertDegToRad(0);
    t = Utility::RadToZeroTo2P(t);
    nStart.setX(x);
    nStart.setY(y);
    nStart.setT(t);
    // set theta to a value (0,2PI]
    t = Utility::RadToZeroTo2P(t);
    x = width - 2;
    y = 2;
    t = Utility::ConvertDegToRad(0);
    nGoal.setX(x);
    nGoal.setY(y);
    nGoal.setT(t);
  }
  else
  {
    Utility::TypeConversion(start, nStart);
    Utility::TypeConversion(goal, nGoal);
    // x = start_.pose.pose.position.x / params_.cell_size;
    // y = start_.pose.pose.position.y / params_.cell_size;
    // t = tf::getYaw(start_.pose.pose.orientation);
    // nStart.setX(x);
    // nStart.setY(y);
    // nStart.setT(t);
    // // retrieving goal position
    // x = goal_.pose.position.x / params_.cell_size;
    // y = goal_.pose.position.y / params_.cell_size;
    // t = tf::getYaw(goal_.pose.orientation);
    // nGoal.setX(x);
    // nGoal.setY(y);
    // nGoal.setT(t);
  }

  std::srand(0);
  Clear();
  // START AND TIME THE PLANNING
  ros::Time t0 = ros::Time::now();
  Utility::Path3D path, temp;
  if (params_.use_rrt)
  {
    LOG(INFO) << "Use RRT!";
    path = rrt_planner_ptr_->GetPath(nStart, nGoal);
  }
  else if (params_.use_a_star)
  {
    LOG(INFO) << "Use A star!";
    path = a_star_planner_ptr_->GetPath(nStart, nGoal, nodes2D);
  }
  else
  {
    // FIND THE PATH
    LOG(INFO) << "Use hybrid a star!";
    path = hybrid_a_star_ptr_->GetPath(nStart, nGoal, nodes3D, nodes2D);
  }
  LOG(INFO) << "path length is " << Utility::GetLength(path);
  //  set path
  smoother_ptr_->SetPath(path);
  // CREATE THE UPDATED PATH
  path_publisher_ptr_->UpdatePath(smoother_ptr_->GetPath());
  // DLOG(INFO) << "path before smooth size is " << smoother_ptr_->GetPath().size();
  // SMOOTH THE PATH
  if (params_.smooth)
  {
    smoother_ptr_->SmoothPath(voronoi_diagram_);
    // CREATE THE UPDATED PATH
    // DLOG(INFO) << "smoothed path size is " << smoother_ptr_->GetPath().size();
    smoothed_path_publisher_ptr_->UpdatePath(smoother_ptr_->GetPath());
  }
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  // LOG(INFO) << "TIME in ms: " << d * 1000;
  LOG(INFO) << "TIME in second: " << d;
  Publish(nodes3D, nodes2D, width, height, depth);
  Utility::TypeConversion(path, plan);
  delete[] nodes3D;
  delete[] nodes2D;
  // set these two flag to false when finished planning to avoid unwanted planning
}
void Planner::Clear()
{
  // CLEAR THE VISUALIZATION
  visualization_ptr_->clear();
  // CLEAR THE PATH
  path_publisher_ptr_->Clear();
  smoothed_path_publisher_ptr_->Clear();
  smoother_ptr_->Clear();
}

void Planner::Publish(Node3D *nodes3D, Node2D *nodes2D, const int &width, const int &height, const int &depth)
{
  // _________________________________
  // PUBLISH THE RESULTS OF THE SEARCH
  path_publisher_ptr_->PublishPath();
  path_publisher_ptr_->PublishPathNodes();
  path_publisher_ptr_->PublishPathVehicles();
  smoothed_path_publisher_ptr_->PublishPath();
  smoothed_path_publisher_ptr_->PublishPathNodes();
  smoothed_path_publisher_ptr_->PublishPathVehicles();
  visualization_ptr_->publishNode3DCosts(nodes3D, width, height, depth);
  visualization_ptr_->publishNode2DCosts(nodes2D, width, height);
}