#include "path_publisher.h"

using namespace HybridAStar;

//###################################################
//                                         CLEAR PATH
//###################################################

void PathPublisher::Clear()
{
  Node3D node;
  path_.poses.clear();
  path_nodes_.markers.clear();
  path_vehicles_.markers.clear();
  AddNode(node, 0);
  AddVehicle(node, 1);
  PublishPath();
  PublishPathNodes();
  PublishPathVehicles();
}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void PathPublisher::UpdatePath(const std::vector<Node3D> &nodePath)
{
  path_.header.stamp = ros::Time();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i)
  {
    AddSegment(nodePath[i]);
    AddNode(nodePath[i], k);
    k++;
    AddVehicle(nodePath[i], k);
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT
void PathPublisher::AddSegment(const Node3D &node)
{
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX();
  vertex.pose.position.y = node.getY();
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path_.poses.emplace_back(vertex);
}

// ________
// ADD NODE
void PathPublisher::AddNode(const Node3D &node, int i)
{
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0)
  {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  pathNode.pose.position.x = node.getX();
  pathNode.pose.position.y = node.getY();
  path_nodes_.markers.emplace_back(pathNode);
}

void PathPublisher::AddVehicle(const Node3D &node, int i)
{
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1)
  {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = params_.vehicle_length - params_.bloating * 2;
  pathVehicle.scale.y = params_.vehicle_width - params_.bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed_)
  {
    pathVehicle.color.r = 0;
    pathVehicle.color.g = 1;
    pathVehicle.color.b = 0;
  }
  else
  {
    pathVehicle.color.r = 1;
    pathVehicle.color.g = 0;
    pathVehicle.color.b = 0;
  }
  //need to figure what pose stand for a cube, should be center of cube
  pathVehicle.pose.position.x = node.getX();
  pathVehicle.pose.position.y = node.getY();
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  path_vehicles_.markers.emplace_back(pathVehicle);
}
