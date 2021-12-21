#ifndef PARAMETER_MANAGER_H
#define PARAMETER_MANAGER_H

#include <ros/ros.h>

namespace HybridAStar
{
  struct ParameterContainer
  {
    //****************************** smoother parameter************************************
    // max iterations for smoother
    int max_iterations = 10000;
    // the small number which will terminate loop if path difference smaller than this number.
    float epsilon = 1e-3;
    /// maximum possible curvature of the non-holonomic vehicle
    float min_turning_radius = 6;
    /// maximum distance to obstacles that is penalized
    float obsd_max = 6;
    /// maximum distance for obstacles to influence the voronoi field
    float vor_obs_dmax = 3;
    /// falloff rate for the voronoi field
    float alpha = 0.1;
    /// weight for the obstacle term
    float weight_obstacle = 2.0;
    /// weight for the voronoi term
    float weight_voronoi = 2.0;
    /// weight for the curvature term
    float weight_curvature = 0.0;
    /// weight for the smoothness term
    float weight_smoothness = 2.0;
    //weight for path length
    float weight_length = 0.5;
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

    //load param for smoother
    virtual void LoadSmootherParams();

    //load param by param name, member variable, param type
    template <typename T>
    void GetSingleParam(const std::string &param_name, T &param_data);

    //get the ptr of all param
    std::shared_ptr<ParameterContainer> GetAllParam();

  private:
    std::shared_ptr<ParameterContainer> param_container_ptr_;
    ros::NodeHandle nh_;
  };

}
#endif //
