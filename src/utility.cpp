#include "utility.h"

namespace HybridAStar
{
    namespace Utility
    {
        void ConvertRosPathToVectorNode3D(const nav_msgs::Path::ConstPtr &path, std::vector<Node3D> &node_3d_vec)
        {
            node_3d_vec.clear();
            for (uint i = 0; i < path->poses.size(); ++i)
            {
                Node3D point;
                point.setX(path->poses[i].pose.position.x);
                point.setY(path->poses[i].pose.position.y);
                //not sure this is correct;
                point.setT(path->poses[i].pose.position.z);
                node_3d_vec.emplace_back(point);
            }
        }

        Node2D ConvertNode3DToNode2D(const Node3D node_3d)
        {
            Node2D out;
            out.setX(node_3d.getX());
            out.setY(node_3d.getY());
            return out;
        }

        Node2D ConvertIndexToNode2D(const int &index, const int &map_width)
        {
            Node2D node_2d;
            node_2d.setX(index % map_width);
            node_2d.setY(index / map_width);
            return node_2d;
        }

        float GetDistanceFromNode2DToNode3D(const Node2D &obstacle_2d, const Node3D &node_3d)
        {
            float distance;
            float delta_x = node_3d.getX() - obstacle_2d.getX();
            float delta_y = node_3d.getY() - obstacle_2d.getY();
            distance = sqrt(delta_x * delta_x + delta_y * delta_y);
            return distance;
        }
    }
}