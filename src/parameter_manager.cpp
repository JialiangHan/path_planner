
#include "parameter_manager.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

namespace HybridAStar
{

  void ParameterManager::LoadSmootherParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "max_iterations";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->max_iterations);

    ros_param_name = "epsilon";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->epsilon);

    ros_param_name = "min_turning_radius";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->min_turning_radius);

    ros_param_name = "obsd_max";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->obsd_max);

    ros_param_name = "vor_obs_dmax";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->vor_obs_dmax);

    ros_param_name = "alpha";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->alpha);

    ros_param_name = "weight_obstacle";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->weight_obstacle);

    ros_param_name = "weight_voronoi";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->weight_voronoi);

    ros_param_name = "weight_curvature";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->weight_curvature);

    ros_param_name = "weight_smoothness";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->weight_smoothness);

    ros_param_name = "weight_length";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->weight_length);
  }

  template <typename T>
  void ParameterManager::GetSingleParam(const std::string &param_name, T &param_data)
  {
    nh_.getParam(param_name, param_data);
    DLOG(INFO) << "Load param: " << param_name << " value is :" << param_data;
  }

  std::shared_ptr<ParameterContainer> ParameterManager::GetAllParam()
  {
    return param_container_ptr_;
  }

}
