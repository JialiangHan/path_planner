/**
 * @file lookup_table.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief a lookup table class for dubins, reedsheep curve and cubic bezier, unconstrained, without obstacle
 * @version 0.2
 * @date 2022-01-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <unordered_map>
#include "parameter_manager.h"
#include "node3d.h"
#include "cubic_bezier.h"
#include "bezier.h"
namespace HybridAStar
{
    typedef ompl::base::SE2StateSpace::StateType State;
    class LookupTable
    {
    public:
        LookupTable();
        LookupTable(const ParameterCollisionDetection &params);

        void Clear();

        void Initialize(const int &width, const int &height);

        float GetDubinsCost(const Node3D &node3d) const;
        float GetReedsSheppCost(const Node3D &node3d) const;
        float GetCubicBezierCost(const Node3D &node3d) const;
        // int CalculateNode3DIndex(const float &x, const float &y, const float &theta) const;

    private:
        int CalculateNode3DIndex(const Node3D &node3d) const;
        int CalculateNode3DIndex(const float &x, const float &y, const float &theta) const;
        void CalculateDubinsLookup();
        void CalculateReedsSheppLookup();
        void CalculateCubicBezierLookup();
        /**
         * @brief use bezier.h from github for cubic bezier
         * 
         */
        void CalculateCubicBezierLookupV1();

    private:
        ParameterCollisionDetection params_;
        int map_width_ = 0;
        int map_height_ = 0;
        /**
         * @brief key is node index, value is the curve length
         * 
         */
        std::unordered_map<int, float> dubins_lookup_;
        std::unordered_map<int, float> reeds_shepp_lookup_;
        std::unordered_map<int, float> cubic_bezier_lookup_;
    };
}
