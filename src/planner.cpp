#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  //    InitializeLookups();
  // Lookup::collisionLookup(collision_lookup_table);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configuration_space_;
  // _________________
  // TOPICS TO PUBLISH
  pub_start_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    sub_map_ = nh_.subscribe("/map", 1, &Planner::SetMap, this);
  } else {
    sub_map_ = nh_.subscribe("/occ_map", 1, &Planner::SetMap, this);
  }

  sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &Planner::SetGoal, this);
  sub_start_ = nh_.subscribe("/initialpose", 1, &Planner::SetStart, this);
  smoother_ptr_.reset(new Smoother(nh_));
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::InitializeLookups()
{
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubins_lookup_table);
  }

  Lookup::collisionLookup(collision_lookup_table);
}

//###################################################
//                                                MAP
//###################################################
void Planner::SetMap(const nav_msgs::OccupancyGrid::Ptr map)
{
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid_ = map;
  //update the configuration space with the current map
  configuration_space_.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoi_diagram_.initializeMap(width, height, binMap);
  voronoi_diagram_.update();
  voronoi_diagram_.visualize();
  //  ros::Time t1 = ros::Time::now();
  //  ros::Duration d(t1 - t0);
  //  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener_.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr))
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
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::SetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial)
{
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0)
  {
    valid_start_ = true;
    start_ = *initial;

    if (Constants::manual)
    {
      MakePlan();
    }

    // publish start for RViz
    pub_start_.publish(startN);
  }
  else
  {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::SetGoal(const geometry_msgs::PoseStamped::ConstPtr &end)
{
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0)
  {
    valid_goal_ = true;
    goal_ = *end;

    if (Constants::manual)
    {
      MakePlan();
    }
  }
  else
  {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::MakePlan()
{
  // if a start as well as goal are defined go ahead and plan
  if (valid_start_ && valid_goal_)
  {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid_->info.width;
    int height = grid_->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal_.pose.position.x / Constants::cellSize;
    float y = goal_.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal_.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start_.pose.pose.position.x / Constants::cellSize;
    y = start_.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start_.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization_.clear();
    // CLEAR THE PATH
    path_.clear();
    smoothed_path_.clear();
    // FIND THE PATH
    Node3D *nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configuration_space_, dubins_lookup_table, visualization_);
    // TRACE THE PATH
    smoother_ptr_->TracePath(nSolution);
    // CREATE THE UPDATED PATH
    path_.updatePath(smoother_ptr_->GetPath());
    // SMOOTH THE PATH
    smoother_ptr_->SmoothPath(voronoi_diagram_);
    // CREATE THE UPDATED PATH
    smoothed_path_.updatePath(smoother_ptr_->GetPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path_.publishPath();
    path_.publishPathNodes();
    path_.publishPathVehicles();
    smoothed_path_.publishPath();
    smoothed_path_.publishPathNodes();
    smoothed_path_.publishPathVehicles();
    visualization_.publishNode3DCosts(nodes3D, width, height, depth);
    visualization_.publishNode2DCosts(nodes2D, width, height);

    delete [] nodes3D;
    delete [] nodes2D;
  }
  else
  {
    std::cout << "missing goal or start" << std::endl;
  }
}
