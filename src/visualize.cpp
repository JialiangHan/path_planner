#include "visualize.h"
using namespace HybridAStar;
//###################################################
//                                CLEAR VISUALIZATION
//###################################################
void Visualize::clear() {
  poses3D.poses.clear();
  poses3Dreverse.poses.clear();
  poses2D.markers.clear();
  // 3D COSTS
  visualization_msgs::MarkerArray costCubes3D;
  visualization_msgs::Marker costCube3D;
  // CLEAR THE COST HEATMAP
  costCube3D.header.frame_id = "map";
  costCube3D.header.stamp = ros::Time();
  costCube3D.id = 0;
  costCube3D.action = 3;
  costCubes3D.markers.emplace_back(costCube3D);
  pubNodes3DCosts.publish(costCubes3D);

  // 2D COSTS
  visualization_msgs::MarkerArray costCubes2D;
  visualization_msgs::Marker costCube2D;
  // CLEAR THE COST HEATMAP
  costCube2D.header.frame_id = "map";
  costCube2D.header.stamp = ros::Time();
  costCube2D.id = 0;
  costCube2D.action = 3;
  costCubes2D.markers.emplace_back(costCube2D);
  pubNodes2DCosts.publish(costCubes2D);

  // 2D poses
  visualization_msgs::Marker pose2d;
  // CLEAR THE COST HEATMAP
  pose2d.header.frame_id = "map";
  pose2d.header.stamp = ros::Time();
  pose2d.id = 0;
  pose2d.action = 3;
  poses2D.markers.emplace_back(pose2d);
  pubNodes2D.publish(poses2D);
}

//###################################################
//                                    CURRENT 3D NODE
//###################################################
void Visualize::publishNode3DPose(const Node3D &node)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time();
  pose.header.seq = 0;
  pose.pose.position.x = node.getX();
  pose.pose.position.y = node.getY();

  //FORWARD
  if (node.getPrim() < 3)
  {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  }
  //REVERSE
  else {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
  }

  // PUBLISH THE POSE
  pubNode3D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 3D NODES
//###################################################
void Visualize::publishNode3DPoses(const Node3D &node)
{

  geometry_msgs::Pose pose;
  pose.position.x = node.getX();
  pose.position.y = node.getY();

  //FORWARD
  if (node.getPrim() < 3)
  {
    // LOG(INFO) << "publishing forward nodes";
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    poses3D.poses.emplace_back(pose);
    poses3D.header.stamp = ros::Time();
    // PUBLISH THE POSEARRAY
    pubNodes3D.publish(poses3D);
  }
  //REVERSE
  else
  {
    // LOG(INFO) << "publishing reverse nodes";
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
    poses3Dreverse.poses.emplace_back(pose);
    poses3Dreverse.header.stamp = ros::Time();
    // PUBLISH THE POSEARRAY
    pubNodes3Dreverse.publish(poses3Dreverse);
  }
}

//###################################################
//                                    CURRENT 2D NODE
//###################################################
void Visualize::publishNode2DPose(const Node2D &node)
{
  geometry_msgs::PoseStamped pose;
  // DLOG(INFO) << "publishing";
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time();
  pose.header.seq = 0;
  pose.pose.position.x = (node.getX() + 0.5);
  pose.pose.position.y = (node.getY() + 0.5);
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // PUBLISH THE POSE
  pubNode2D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 2D NODES
//###################################################
void Visualize::publishNode2DPoses(const Node2D &node)
{

  // DLOG(INFO) << "publishing node " << node.getX() << " " << node.getY();

  // _______________
  visualization_msgs::Marker pose2d;
  pose2d.action = visualization_msgs::Marker::ADD;

  pose2d.header.frame_id = "map";
  pose2d.header.stamp = ros::Time();
  pose2d.id = poses2D.markers.size();
  pose2d.type = visualization_msgs::Marker::CUBE;
  pose2d.scale.x = 0.2;
  pose2d.scale.y = 0.2;
  pose2d.scale.z = 0.1;

  pose2d.color.a = 0.5;
  pose2d.color.r = 1;
  pose2d.color.g = 0;
  pose2d.color.b = 0;
  // center in cell +0.5
  pose2d.pose.position.x = node.getX();
  pose2d.pose.position.y = node.getY();
  pose2d.pose.position.z = 1;
  pose2d.pose.orientation.x = 0.0;
  pose2d.pose.orientation.y = 0.0;
  pose2d.pose.orientation.z = 0.0;
  pose2d.pose.orientation.w = 1.0;
  poses2D.markers.emplace_back(pose2d);
  // PUBLISH THE POSEARRAY
  pubNodes2D.publish(poses2D);
}
// / ################################################## #
//                                    CURRENT 2D NODE
//###################################################
void Visualize::publishNode2DPose(const Node3D &node)
{
  geometry_msgs::PoseStamped pose;
  // DLOG(INFO) << "publishing";
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time();
  pose.header.seq = 0;
  pose.pose.position.x = (node.getX());
  pose.pose.position.y = (node.getY());
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // PUBLISH THE POSE
  pubNode2D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 2D NODES
//###################################################
void Visualize::publishNode2DPoses(const Node3D &node)
{

  // DLOG(INFO) << "publishing node " << node.getX() << " " << node.getY();

  // _______________
  visualization_msgs::Marker pose2d;
  pose2d.action = visualization_msgs::Marker::ADD;

  pose2d.header.frame_id = "map";
  pose2d.header.stamp = ros::Time();
  pose2d.id = poses2D.markers.size();
  pose2d.type = visualization_msgs::Marker::CUBE;
  pose2d.scale.x = 0.2;
  pose2d.scale.y = 0.2;
  pose2d.scale.z = 0.1;

  pose2d.color.a = 0.5;
  pose2d.color.r = 1;
  pose2d.color.g = 0;
  pose2d.color.b = 0;
  // center in cell +0.5
  pose2d.pose.position.x = node.getX();
  pose2d.pose.position.y = node.getY();
  pose2d.pose.position.z = 1;
  pose2d.pose.orientation.x = 0.0;
  pose2d.pose.orientation.y = 0.0;
  pose2d.pose.orientation.z = 0.0;
  pose2d.pose.orientation.w = 1.0;
  poses2D.markers.emplace_back(pose2d);
  // PUBLISH THE POSEARRAY
  pubNodes2D.publish(poses2D);
}

void Visualize::publishSearchNodes(Node3D node, ros::Publisher &pub,
                                   visualization_msgs::MarkerArray &pathNodes, int i)
{
  visualization_msgs::Marker pathVehicle;
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.color.r = 250.f / 255.f;
  pathVehicle.color.g = 250.f / 255.f;
  pathVehicle.color.b = 52.f / 255.f;
  pathVehicle.type = visualization_msgs::Marker::ARROW;
  pathVehicle.header.frame_id = "map";
  pathVehicle.scale.x = 0.22;
  pathVehicle.scale.y = 0.18;
  pathVehicle.scale.z = 0.12;
  pathVehicle.color.a = 0.1;
  // 转化节点，并同时加上时间戳等信息
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.pose.position.x = node.getX();
  pathVehicle.pose.position.y = node.getY();
  pathVehicle.pose.position.z = 0;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicle.id = i;
  pathNodes.markers.push_back(pathVehicle);

  // 发布这些车辆位置标记点
  pub.publish(pathNodes);

} // end of publishPathNodes