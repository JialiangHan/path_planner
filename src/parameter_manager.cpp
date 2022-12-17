
#include "parameter_manager.h"

namespace HybridAStar
{
  void ParameterManager::LoadParams()
  {
    LoadSmootherParams();
    LoadHybridAStarParams();
    LoadPathParams();
    LoadPlannerParams();
    LoadPathParams();
    LoadVisualizeParams();
    LoadRRTPlannerParams();
    LoadAStarPlannerParams();
  }
  void ParameterManager::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
  {
    nh_ = nh;
    nh_.param("odom_topic", odom_topic, odom_topic);
    nh_.param("map_frame", map_frame, map_frame);
    LoadParams();
    // checkParameters();
    // checkDeprecated(nh);
  }
  void ParameterManager::LoadRRTPlannerParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "visualization";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.visualization);
    ros_param_name = "max_iterations";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.max_iterations);

    ros_param_name = "visualization2D";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.visualization2D);
    ros_param_name = "goal_range";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.goal_range);

    ros_param_name = "headings";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.headings);

    ros_param_name = "possibility_to_goal";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.possibility_to_goal);

    ros_param_name = "consider_orientation";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.consider_orientation);
    ros_param_name = "curve_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.curve_step_size);

    ros_param_name = "analytical_expansion";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.analytical_expansion);

    ros_param_name = "consider_steering_angle_range";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.consider_steering_angle_range);

    ros_param_name = "steering_angle_resolution";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.steering_angle_resolution);

    ros_param_name = "step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.step_size);

    ros_param_name = "adaptive_possibility_to_goal";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.adaptive_possibility_to_goal);

    ros_param_name = "number_of_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.number_of_step_size);

    ros_param_name = "rewire";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.rewire);

    ros_param_name = "neighbor_detection_radius";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.neighbor_detection_radius);

    ros_param_name = "use_rrt_connect";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.use_rrt_connect);

    ros_param_name = "twoD_rrt";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.twoD_rrt);

    ros_param_name = "use_AEB_rrt";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->rrt_planner_params.use_AEB_rrt);

    LoadCollisionDetectionParams();
  }
  void ParameterManager::LoadSmootherParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "max_iterations";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.max_iterations);

    ros_param_name = "epsilon";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.epsilon);

    ros_param_name = "min_turning_radius";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.min_turning_radius);

    ros_param_name = "obsd_max";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.obsd_max);

    ros_param_name = "vor_obs_dmax";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.vor_obs_dmax);

    ros_param_name = "alpha";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.alpha);

    ros_param_name = "weight_obstacle";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.weight_obstacle);

    ros_param_name = "weight_voronoi";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.weight_voronoi);

    ros_param_name = "weight_curvature";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.weight_curvature);

    ros_param_name = "weight_smoothness";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.weight_smoothness);

    ros_param_name = "weight_length";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->smoother_params.weight_length);
  }

  void ParameterManager::LoadCollisionDetectionParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "obstacle_detection_range";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.obstacle_detection_range);

    ros_param_name = "enable_collision_lookup";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.enable_collision_lookup);

    ros_param_name = "map_boundary_obstacle";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.map_boundary_obstacle);

    ros_param_name = "consider_steering_angle_range_for_obstacle_density";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.consider_steering_angle_range_for_obstacle_density);

    ros_param_name = "add_one_more_successor_only_in_free_angle_range";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.add_one_more_successor_only_in_free_angle_range);

    ros_param_name = "steering_angle_towards_free_angle_range_for_obstacle_angle_range";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.steering_angle_towards_free_angle_range_for_obstacle_angle_range);

    ros_param_name = "fixed_number_of_steering_angle_in_free_angle_range";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.fixed_number_of_steering_angle_in_free_angle_range);

    ros_param_name = "add_one_more_successor";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.add_one_more_successor);

    ros_param_name = "position_resolution";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.position_resolution);

    ros_param_name = "headings";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.headings);

    ros_param_name = "min_turning_radius";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.min_turning_radius);

    ros_param_name = "reverse";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.reverse);
    ros_param_name = "curve_type";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.curve_type);

    ros_param_name = "vehicle_width";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.vehicle_width);

    ros_param_name = "vehicle_length";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.vehicle_length);
  }
  void ParameterManager::LoadPlannerParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "smooth";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.smooth);

    ros_param_name = "headings";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.headings);

    ros_param_name = "cell_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.cell_size);

    ros_param_name = "manual";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.manual);

    ros_param_name = "use_rrt";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.use_rrt);

    ros_param_name = "use_a_star";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.use_a_star);

    ros_param_name = "fix_start_goal";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.fix_start_goal);
  }
  void ParameterManager::LoadHybridAStarParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "number_of_successors";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.number_of_successors);

    ros_param_name = "curve_type_analytical_expansion";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.curve_type_analytical_expansion);

    ros_param_name = "adaptive_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.adaptive_step_size);

    ros_param_name = "piecewise_cubic_bezier_interpolation";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.piecewise_cubic_bezier_interpolation);

    ros_param_name = "adaptive_steering_angle_and_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.adaptive_steering_angle_and_step_size);

    ros_param_name = "step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.step_size);
    ros_param_name = "adaptive_steering_angle";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.adaptive_steering_angle);

    ros_param_name = "analytical_expansion_every_point";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.analytical_expansion_every_point);

    ros_param_name = "analytical_expansion";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.analytical_expansion);

    ros_param_name = "penalty_turning";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.penalty_turning);
    ros_param_name = "penalty_reverse";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.penalty_reverse);
    ros_param_name = "penalty_change_of_direction";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.penalty_change_of_direction);
    ros_param_name = "steering_angle";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.steering_angle);
    ros_param_name = "reverse";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.reverse);
    ros_param_name = "visualization";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.visualization);
    ros_param_name = "max_iterations";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.max_iterations);

    ros_param_name = "tie_breaker";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.tie_breaker);
    ros_param_name = "visualization2D";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.visualization2D);
    ros_param_name = "goal_range";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.goal_range);

    ros_param_name = "curve_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.curve_step_size);
    ros_param_name = "min_turning_radius";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.min_turning_radius);

    ros_param_name = "headings";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.headings);

    LoadCollisionDetectionParams();

    LoadAStarPlannerParams();

    ros_param_name = "constant_density";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.constant_density);
  }
  void ParameterManager::LoadPathParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "cell_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.cell_size);

    ros_param_name = "bloating";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.bloating);

    ros_param_name = "vehicle_width";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.vehicle_width);

    ros_param_name = "vehicle_length";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.vehicle_length);
  }

  void ParameterManager::LoadVisualizeParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "cell_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->visualize_params.cell_size);
  }

  void ParameterManager::LoadAStarPlannerParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "possible_direction";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->a_star_planner_params.possible_direction);

    ros_param_name = "use_adaptive_step_size_in_a_star";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->a_star_planner_params.use_adaptive_step_size_in_a_star);

    LoadCollisionDetectionParams();
  }

  template <typename T>
  void ParameterManager::GetSingleParam(const std::string &param_name, T &param_data)
  {
    nh_.getParam(param_name, param_data);
    // DLOG(INFO) << "Load param: " << param_name << " value is :" << param_data;
  }

  // std::shared_ptr<ParameterContainer> ParameterManager::GetAllParams()
  // {
  //   return param_container_ptr_;
  // }

  ParameterHybridAStar ParameterManager::GetHybridAStarParams()
  {
    return param_container_ptr_->hybrid_a_star_params;
  }

  ParameterSmoother ParameterManager::GetSmootherParams()
  {
    return param_container_ptr_->smoother_params;
  }

  ParameterPlanner ParameterManager::GetPlannerParams()
  {
    return param_container_ptr_->planner_params;
  }

  ParameterPathPublisher ParameterManager::GetPathPublisherParams()
  {
    return param_container_ptr_->path_publisher_params;
  }

  ParameterVisualize ParameterManager::GetVisualizeParams()
  {
    return param_container_ptr_->visualize_params;
  }

  ParameterCollisionDetection ParameterManager::GetCollisionDetectionParams()
  {
    return param_container_ptr_->collision_detection_params;
  }

  ParameterRRTPlanner ParameterManager::GetRRTPlannerParams()
  {
    return param_container_ptr_->rrt_planner_params;
  }

  ParameterAStar ParameterManager::GetAStarPlannerParams()
  {
    return param_container_ptr_->a_star_planner_params;
  }

  // void ParameterManager::checkParameters() const
  // {
  //   // positive backward velocity?
  //   if (robot.max_vel_x_backwards <= 0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");

  //   // bounds smaller than penalty epsilon
  //   if (robot.max_vel_x <= optim.penalty_epsilon)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  //   if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  //   if (robot.max_vel_theta <= optim.penalty_epsilon)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  //   if (robot.acc_lim_x <= optim.penalty_epsilon)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  //   if (robot.acc_lim_theta <= optim.penalty_epsilon)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  //   // dt_ref and dt_hyst
  //   if (trajectory.dt_ref <= trajectory.dt_hysteresis)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");

  //   // min number of samples
  //   if (trajectory.min_samples < 3)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");

  //   // costmap obstacle behind robot
  //   if (obstacles.costmap_obstacles_behind_robot_dist < 0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");

  //   // hcp: obstacle heading threshold
  //   if (hcp.obstacle_keypoint_offset >= 1 || hcp.obstacle_keypoint_offset <= 0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");

  //   // carlike
  //   if (robot.cmd_angle_instead_rotvel && robot.wheelbase == 0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");

  //   if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius == 0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");

  //   // positive weight_adapt_factor
  //   if (optim.weight_adapt_factor < 1.0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");

  //   if (recovery.oscillation_filter_duration < 0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");

  //   // weights
  //   if (optim.weight_optimaltime <= 0)
  //     LOG(WARNING)<<("TebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");
  // }
}
