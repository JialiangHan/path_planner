#pragma once

#include <ctime>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "hybrid_a_star.h"
#include "path_publisher.h"
#include "smoother.h"
#include "lookup.h"
#include "path_evaluator.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void SetMap(const nav_msgs::OccupancyGrid::Ptr map);

  /*!
     \brief SetStart
     \param start the start pose
  */
  void SetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start);

  /*!
     \brief SetGoal
     \param goal the goal pose
  */
  void SetGoal(const geometry_msgs::PoseStamped::ConstPtr &goal);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void MakePlan();

  void Clear();
  void Publish(Node3D *nodes3D, Node2D *nodes2D, const int &width, const int &height, const int &depth);

  void SetPlannerParams(const ParameterPlanner &params);

  private:
  /// The node handle
  ros::NodeHandle nh_;
  /// A publisher publishing the start position for RViz
  ros::Publisher pub_start_;
  /// A subscriber for receiving map updates
  ros::Subscriber sub_map_;
  /// A subscriber for receiving goal updates
  ros::Subscriber sub_goal_;
  /// A subscriber for receiving start updates
  ros::Subscriber sub_start_;
  /// A listener that awaits transforms
  tf::TransformListener listener_;
  /// A transform for moving start positions
  tf::StampedTransform transform_;
  /// The path produced by the hybrid A* algorithm
  std::shared_ptr<PathPublisher> path_publisher_ptr_;
  /// The smoother used for optimizing the path
  std::shared_ptr<Smoother> smoother_ptr_;
  /// The path smoothed and ready for the controller
  std::shared_ptr<PathPublisher> smoothed_path_publisher_ptr_;
  /// The visualization used for search visualization
  std::shared_ptr<Visualize> visualization_ptr_;

  /// The voronoi diagram
  DynamicVoronoi voronoi_diagram_;
  /// A pointer to the grid_ the planner runs on
  nav_msgs::OccupancyGrid::ConstPtr grid_;
  /// The start pose set through RViz
  geometry_msgs::PoseWithCovarianceStamped start_;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal_;
  /// Flags for allowing the planner to plan
  bool valid_start_ = false;
  /// Flags for allowing the planner to plan
  bool valid_goal_ = false;

  PathEvaluator::PathEvaluator path_evaluator_;
  //parameter manager, load param from *.yaml file
  std::shared_ptr<ParameterManager> param_manager_;
  ParameterPlanner params_;
  std::shared_ptr<HybridAStar> hybrid_a_star_ptr_;
};
}
