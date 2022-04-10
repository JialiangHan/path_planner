
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

    ros_param_name = "enable_collision_lookup";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.enable_collision_lookup);

    ros_param_name = "position_resolution";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.position_resolution);

    ros_param_name = "headings";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.headings);

    ros_param_name = "weight_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.weight_step_size);

    ros_param_name = "min_turning_radius";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.min_turning_radius);

    ros_param_name = "reverse";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.reverse);
    ros_param_name = "curve_type";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.curve_type);
    // ros_param_name = "obstacle_detection_range";
    // GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.collision_detection_params.obstacle_detection_range);

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
  }
  void ParameterManager::LoadHybridAStarParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "number_of_successors";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.number_of_successors);

    ros_param_name = "piecewise_cubic_bezier_interpolation";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.piecewise_cubic_bezier_interpolation);

    ros_param_name = "adaptive_steering_angle_and_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.adaptive_steering_angle_and_step_size);
    ros_param_name = "adaptive_step_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->hybrid_a_star_params.adaptive_step_size);
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

  template <typename T>
  void ParameterManager::GetSingleParam(const std::string &param_name, T &param_data)
  {
    nh_.getParam(param_name, param_data);
    // DLOG(INFO) << "Load param: " << param_name << " value is :" << param_data;
  }

  std::shared_ptr<ParameterContainer> ParameterManager::GetAllParams()
  {
    return param_container_ptr_;
  }

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
}
