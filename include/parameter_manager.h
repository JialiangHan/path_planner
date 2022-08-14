#pragma once

#include <ros/ros.h>
#include "glog/logging.h"
#include "gflags/gflags.h"
namespace HybridAStar
{

  //this struct contains some used parameters in collisiondetection class
  struct ParameterCollisionDetection
  {
    /// [#] --- The sqrt of the number of discrete positions per cell
    int position_resolution = 10;
    /// [m] --- The number of discretizations in heading,used in planner.cpp
    int headings = 72;

    float min_turning_radius = 6;

    int curve_type = 0;

    bool reverse = true;

    /// [m] --- The width of the vehicle
    float vehicle_width = 1.75;
    /// [m] --- The length of the vehicle
    float vehicle_length = 2.65;

    bool enable_collision_lookup = false;
    // parameter which control the step size when has an obstacle
    // float weight_step_size = 0.5;

    bool consider_steering_angle_range_for_obstacle_density = false;

    bool add_one_more_successor_only_in_free_angle_range = false;

    bool add_one_more_successor = true;

    bool map_boundary_obstacle = false;

    bool steering_angle_towards_free_angle_range_for_obstacle_angle_range = true;

    bool fixed_number_of_steering_angle_in_free_angle_range = true;

    bool make_step_size_larger_than_one = true;
  };

  //this struct contains some used parameters in visualize class
  struct ParameterVisualize
  {
    /// [m] --- The cell size of the 2D grid of the world,used in planner.cpp
    float cell_size = 1;
  };

  //this struct contains some used parameters in algorithm class
  struct ParameterHybridAStar
  {

    int number_of_successors = 10;
    bool piecewise_cubic_bezier_interpolation = false;
    bool adaptive_steering_angle_and_step_size = false;
    // this parameter determine the frequency of analytical expansion, false means the way in the original paper
    bool analytical_expansion_every_point = true;
    //for create successor
    float step_size = 1;
    bool adaptive_steering_angle = false;

    ParameterCollisionDetection collision_detection_params;
    // ParameterAStar a_star_params;
    bool analytical_expansion = true;

     /// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
    float penalty_turning = 1.05;
    /// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
    float penalty_reverse = 1;
    /// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
    float penalty_change_of_direction = 2;
    /// A flag to toggle reversing (true = on; false = off)
    bool reverse = true;
    /// A flag for the visualization of 3D nodes (true = on; false = off)
    bool visualization = false;
    /// A flag for the visualization of 2D nodes (true = on; false = off)
    bool visualization2D = true;
    // max iterations for smoother
    int max_iterations = 10000;
    float steering_angle = 5;
    // the small number which will terminate loop if path difference smaller than this number.
    float goal_range = 1e-3;
    /*!
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell

  As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
  This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
  to allow the successor being placed in the same cell.
*/
    float tie_breaker = 0.01;
    /// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
    float curve_step_size = 1;
    /// maximum possible curvature of the non-holonomic vehicle
    float min_turning_radius = 6;
    ///  --- The number of discretizations in heading,used in planner.cpp
    int headings = 72;
    // nubmer of direction to create successor for A-star algorithm
    int possible_direction = 8;
  };

  struct ParameterRRTPlanner
  {
    ParameterCollisionDetection collision_detection_params;
    /// A flag for the visualization of 3D nodes (true = on; false = off)
    bool visualization = false;
    /// A flag for the visualization of 2D nodes (true = on; false = off)
    bool visualization2D = true;
    // max iterations for smoother
    int max_iterations = 10000;
    // probability to choose goal rather than a random node
    float possibility_to_goal = 0.5;
    // the small number which will terminate loop if path difference smaller than this number.
    float goal_range = 1e-3;
    ///  --- The number of discretizations in heading,used in planner.cpp
    int headings = 72;
    // use adaptive step size or not
    bool adaptive_step_size = false;
    // parameter used in goal check, false means just consider coordinate not orientation
    bool consider_orientation = false;
  };
  //this struct contains some used parameters in path class
  struct ParameterPathPublisher
  {
    /// [m] --- The cell size of the 2D grid of the world,used in planner.cpp
    float cell_size = 1;
    // #********************** vehicle parameters *****************

    /// [m] --- Uniformly adds a padding around the vehicle
    float bloating = 0;
    /// [m] --- The width of the vehicle
    float vehicle_width = 1.75;
    /// [m] --- The length of the vehicle
    float vehicle_length = 2.65;
  };
  //this struct contains some used parameter in planner.cpp
  struct ParameterPlanner
  {
    /// A flag for the mode (true = manual; false = dynamic). Manual for  map or dynamic for dynamic map. used in planner.cpp
    bool manual = true;
    /// [m] --- The cell size of the 2D grid of the world,used in planner.cpp
    float cell_size = 1;
    /// [m] --- The number of discretizations in heading,used in planner.cpp
    int headings = 72;

    bool smooth = false;

    bool use_rrt = false;
  };

  struct ParameterSmoother
  {
    //****************************** smoother parameter************************************
    // max iterations for smoother
    int max_iterations = 1000;
    // the small number which will terminate loop if path difference smaller than this number.
    float epsilon = 1e-6;
    /// maximum possible curvature of the non-holonomic vehicle
    float min_turning_radius = 6;
    /// maximum distance to obstacles that is penalized
    float obsd_max = 4;
    /// maximum distance for obstacles to influence the voronoi field
    float vor_obs_dmax = 3;
    /// falloff rate for the voronoi field
    float alpha = 0.1;
    /// weight for the obstacle term
    float weight_obstacle = 0;
    /// weight for the voronoi term
    float weight_voronoi = 0;
    /// weight for the curvature term
    float weight_curvature = 0;
    /// weight for the smoothness term
    float weight_smoothness = 0;
    //weight for path length
    float weight_length = 0;
  };
  struct ParameterContainer
  {
    ParameterHybridAStar hybrid_a_star_params;
    ParameterPathPublisher path_publisher_params;
    ParameterSmoother smoother_params;
    ParameterPlanner planner_params;
    ParameterVisualize visualize_params;
    ParameterCollisionDetection collision_detection_params;
    ParameterRRTPlanner rrt_planner_params;
  };

  class ParameterManager
  {

  public:
    ParameterManager() {}

    ParameterManager(const ros::NodeHandle &in)
    {
      nh_ = in;
      param_container_ptr_.reset(new ParameterContainer);
    }

    virtual ~ParameterManager() {}

    virtual void Initialization() {}
    virtual void LoadParams();
    virtual void LoadHybridAStarParams();
    virtual void LoadPathParams();
    //load param for smoother
    virtual void LoadSmootherParams();
    virtual void LoadPlannerParams();
    virtual void LoadVisualizeParams();
    virtual void LoadCollisionDetectionParams();

    virtual void LoadRRTPlannerParams();

    // virtual void LoadAStarParams();

    //load param by param name, member variable, param type
    template <typename T>
    void GetSingleParam(const std::string &param_name, T &param_data);

    //get the ptr of all param
    std::shared_ptr<ParameterContainer> GetAllParams();

    ParameterSmoother GetSmootherParams();
    ParameterPathPublisher GetPathPublisherParams();
    ParameterPlanner GetPlannerParams();
    ParameterHybridAStar GetHybridAStarParams();
    ParameterVisualize GetVisualizeParams();
    ParameterCollisionDetection GetCollisionDetectionParams();
    ParameterRRTPlanner GetRRTPlannerParams();
    // ParameterAStar GetAStarParams();

  private:
    std::shared_ptr<ParameterContainer> param_container_ptr_;
    ros::NodeHandle nh_;
  };
}
