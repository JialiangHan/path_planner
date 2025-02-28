#pragma once

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "parameter_manager.h"
#include "gradient.h"
#include "node3d.h"
#include "node2d.h"
namespace HybridAStar {

/*!
   \brief A class for visualizing the hybrid A* search.

  Depending on the settings in constants.h the visualization will send different amounts of detail.
  It can show the 3D search as well as the underlying 2D search used for the holonomic with obstacles heuristic.
*/
class Visualize {
 public:
  // ___________
  // CONSTRUCTOR

   Visualize()
   {

     // _________________
     // TOPICS TO PUBLISH
     pubNode3D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes3DPose", 100);
     pubNodes3D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);
     pubNodes3Dreverse = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPosesReverse", 100);
     pubNodes3DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes3DCosts", 100);
     pubNode2D = n.advertise<visualization_msgs::Marker>("/visualizeNodes2DPose", 100);
     pubNodes2D = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes2DPoses", 10000);
     pubNodes2DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes2DCosts", 100);

     // CONFIGURE THE CONTAINER
     poses3D.header.frame_id = "map";
     poses3Dreverse.header.frame_id = "map";
   }

  // CLEAR VISUALIZATION
  /// Clears the entire visualization
   void clear();

   // PUBLISH A SINGLE/ARRAY 3D NODE TO RViz
   /// Publishes a single node to RViz, usually the one currently being expanded
   void publishNode3DPose(const Node3D &node);
   /// Publishes all expanded nodes to RViz
   void publishNode3DPoses(const Node3D &node);

   // PUBLISH A SINGEL/ARRAY 2D NODE TO RViz
   /// Publishes a single node to RViz, usually the one currently being expanded
   void publishNode2DPose(const Node2D &node);
   /// Publishes all expanded nodes to RViz
   void publishNode2DPoses(const Node2D &node);
   void publishNode2DPose(const Node3D &node);
   /// Publishes all expanded nodes to RViz
   void publishNode2DPoses(const Node3D &node);
   // PUBLISH THE COST FOR A 2D NODE TO RViz

   void publishSearchNodes(Node3D node, ros::Publisher &pub, visualization_msgs::MarkerArray &pathNodes, int i);

 private:
  /// A handle to the ROS node
  ros::NodeHandle n;
  /// Publisher for a single 3D node
  ros::Publisher pubNode3D;
  /// Publisher for an array of 3D forward nodes
  ros::Publisher pubNodes3D;
  /// Publisher for an array of 3D backward nodes
  ros::Publisher pubNodes3Dreverse;
  /// Publisher for an array of 3D cost with color gradient
  ros::Publisher pubNodes3DCosts;
  /// Publisher for a single 2D node
  ros::Publisher pubNode2D;
  /// Publisher for an array of 2D nodes
  ros::Publisher pubNodes2D;
  /// Publisher for an array of 2D cost with color gradient
  ros::Publisher pubNodes2DCosts;
  /// Array of poses describing forward nodes
  geometry_msgs::PoseArray poses3D;
  /// Array of poses describing backward nodes
  geometry_msgs::PoseArray poses3Dreverse;
  /// Array of poses describing 2D heuristic nodes
  visualization_msgs::MarkerArray poses2D;
};
}
