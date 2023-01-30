#pragma once
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <boost/heap/binomial_heap.hpp>
#include "visualize.h"
#include "collisiondetection.h"
#include "a_star.h"
#include "cubic_bezier.h"
#include "lookup_table.h"
#include "piecewise_cubic_bezier.h"
#include <costmap_2d/costmap_2d.h>
#include "expander.h"
#include <ros/publisher.h>
#include "dubins.h"
#include "ReedsShepp.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
typedef ompl::base::SE2StateSpace::StateType State;
#define point_accuracy 0.5
#define theta_accuracy 2
namespace HybridAStar
{
   //###################################################
   //                                    NODE COMPARISON
   //###################################################
   /*!
    \brief A structure to sort nodes in a heap structure
 */
   struct CompareNodes
   {
      /// Sorting 3D nodes by increasing C value - the total estimated cost
      bool operator()(const std::shared_ptr<Node3D> lhs, const std::shared_ptr<Node3D> rhs) const
      {
         return lhs->getTotalCost() > rhs->getTotalCost();
      }
      // Sorting 3D nodes by increasing C value - the total estimated cost
      bool operator()(const Node3D *lhs, const Node3D *rhs) const
      {
         return lhs->getTotalCost() > rhs->getTotalCost();
      }
   };

   struct cmp
   {
      // Sorting 3D nodes by increasing C value - the total estimated cost
      bool operator()(const std::pair<int, double> &left,
                      const std::pair<int, double> &right) const
      {
         return left.second >= right.second;
      }
   };

   typedef boost::heap::binomial_heap<std::shared_ptr<Node3D>, boost::heap::compare<CompareNodes>> priorityQueue;
   /*!
    * \brief A class that encompasses the functions central to the search.
    */
   class HybridAStar : public Expander
   {
   public:
      /// The default constructor
      HybridAStar(){};
      HybridAStar(const ParameterHybridAStar &params,
                  const std::shared_ptr<Visualize> &visualization_ptr);

      /**
       * @brief  Default constructor for the HybridAStarPlanner object
       */
      HybridAStar(std::string frame_id, costmap_2d::Costmap2D *_costmap, const ParameterHybridAStar &params,
                  const std::shared_ptr<Visualize> &visualization_ptr, const std::shared_ptr<CollisionDetection> &_configuration_space_ptr)
          : Expander(frame_id, _costmap)
      {
         // LOG(INFO) << "constructing HybridAStar.";

         params_ = params;
         lookup_table_ptr_.reset(new LookupTable(params_.collision_detection_params));
         configuration_space_ptr_ = _configuration_space_ptr;

         resolution_ = costmap->getResolution();
         origin_y_ = costmap->getOriginY();
         origin_x_ = costmap->getOriginX();

         visualization_ptr_ = visualization_ptr;

         a_star_ptr_.reset(new AStar(frame_id, _costmap, params_.a_star_params, visualization_ptr_, _configuration_space_ptr));

         nav_msgs::OccupancyGrid::Ptr map;
         map.reset(new nav_msgs::OccupancyGrid());
         Utility::TypeConversion(_costmap, frame_id, map);

         map_width_ = configuration_space_ptr_->GetMap()->info.width;
         map_height_ = configuration_space_ptr_->GetMap()->info.height;
         lookup_table_ptr_->Initialize(map_width_, map_height_);
      }
      /**
       * @brief Default deconstructor for the HybridAStarPlanner object
       */
      ~HybridAStar() {}

      /**
       * @brief set map and calculate lookup table
       *
       * @param map
       */
      void Initialize(nav_msgs::OccupancyGrid::Ptr map);

      // HYBRID A* ALGORITHM
      /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
            \return the pointer to the node satisfying the goal condition
  */
      Utility::Path3D GetPath(Node3D &start, Node3D &goal,
                              Node3D *nodes3D, Node2D *nodes2D);

      /**
       * @brief Find the path between the start pose and goal pose
       * @param start the reference of start pose
       * @param goal the reference of goal pose
       * @param cells_x the number of the cells of the costmap in x axis
       * @param cells_y the number of the cells of the costmap in y axis
       * @param plan the reference of plan;
       * @return true if a valid plan was found.
       */
      bool calculatePath(
          const geometry_msgs::PoseStamped &start,
          const geometry_msgs::PoseStamped &goal,
          int cellsX, int cellsY, std::vector<geometry_msgs::PoseStamped> &plan,
          ros::Publisher &pub, visualization_msgs::MarkerArray &pathNodes);

   private:
      /**
       * @brief update heuristic for hybrid a star
       *
       * @param start
       * @param goal
       * @param nodes2D
       */
      void UpdateHeuristic(Node3D &start, const Node3D &goal, Node2D *nodes2D);
      /**
       * @brief analytical expansion in paper, here use dubins curve.
       *
       * @param start
       * @param goal
       * @param configurationSpace
       * @return Node3D*
       */
      Utility::Path3D AnalyticExpansions(const Node3D &start, Node3D &goal);
      /**
       * @brief Create Successor for node 3d
       *
       * @param pred
       * @return std::vector<std::shared_ptr<Node3D>>
       */
      std::vector<std::shared_ptr<Node3D>> CreateSuccessor(const Node3D &pred);
      /**
       * @brief Create successor for node3d according to step size and steering angle vector
       *
       * @param pred
       * @param step_size_steering_angle_vec
       * @return std::vector<std::shared_ptr<Node3D>>
       */
      std::vector<std::shared_ptr<Node3D>> CreateSuccessor(const Node3D &pred, const std::vector<std::pair<float, float>> &step_size_steering_angle_vec);
      /**
       * @brief find step size and steering angle for current node
       *
       * @param pred
       * @return std::vector<std::pair<float, float>> first is step size, second is steering angle
       */
      std::vector<std::pair<float, float>> FindStepSizeAndSteeringAngle(const Node3D &pred);

      void UpdateCostSoFar(Node3D &node, const float &weight_turning, const float &weight_change_of_direction, const float &weight_reverse);

      void TracePath(std::shared_ptr<Node3D> node3d_ptr);

      void ConvertToPiecewiseCubicBezierPath();

      void AddOneMoreStepSizeAndSteeringAngle(const Node3D &pred, std::vector<std::pair<float, float>> &step_size_steering_angle_pair);
      /**
       * @brief Convert a map into a obstacle free map.
       *
       * @param map
       */
      nav_msgs::OccupancyGrid::Ptr TreatAstarMap(nav_msgs::OccupancyGrid::Ptr map);
      /**
       * @brief check if node_3d_ptr is already in the open list
       *
       * @param open_list
       * @param node_3d_ptr
       * @return true
       * @return false
       */
      bool DuplicateCheck(priorityQueue &open_list, std::shared_ptr<Node3D> node_3d_ptr);
      /**
       * @brief build astar cost map for current goal node.
       *
       * @param start
       * @return std::unordered_map<int, float> first is node index, second is cost so far
       */
      std::unordered_map<int, float> BuildAStarCostMap(const Node3D &start);

   private:
      ParameterHybridAStar params_;
      Utility::Path3D piecewise_cubic_bezier_path_;
      Utility::Path3D path_;
      Node3D start_;
      Node3D goal_;
      std::shared_ptr<CollisionDetection> configuration_space_ptr_;
      std::shared_ptr<LookupTable> lookup_table_ptr_;
      std::shared_ptr<Visualize> visualization_ptr_;
      std::shared_ptr<AStar> a_star_ptr_;
      // map related info
      uint map_width_;
      uint map_height_;
      float resolution_;
      float origin_x_;
      float origin_y_;
      /**
       * @brief index of path_ which analytic expansion start.
       *
       */
      uint analytical_expansion_index_;

      std::unordered_map<int, float> astar_cost_map_;
   };
}
