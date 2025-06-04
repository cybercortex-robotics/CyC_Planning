// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef _CASTAR_PLANNER_H_
#define _CASTAR_PLANNER_H_

#include "CyC_TYPES.h"
#include "AStar.hpp"

class CAstarPlanner
{
public:
    CAstarPlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt);
    ~CAstarPlanner();

    /**
     * \brief Computes reference setpoints based on the environment model and the position of the vehicle relative to the global mission path
     *
     * \param _mission_path     Global mission path
     * \param _octree           Octree environment model
     * \param _vehicle_state    State of the vehicle (x, y, velocity, yaw)
     **/
    CycReferenceSetPoints perception2referenceAStar(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycState &_vehicle_state);

private:

    /**
    * \brief Represent the local path as a centripetal Catmull-Rom spline
    *
    * \param path				Point coordinates of the local path transformed from A* coordinate system to world coordinate system
    **/
    std::vector<Eigen::VectorXf> catmullRomChain(std::vector<Eigen::VectorXf> path);

    /**
    * \brief Return the points on the spline given by 4 control points
    * \param new_points			Point coordinates of the spline representation of the local path
    * \param p0					First control point
    * \param p1					Second control point
    * \param p2					Third control point
    * \param p3					Fourth control point
    **/
    void catmullRomSpline(std::vector<Eigen::VectorXf> &new_points, Eigen::VectorXf p0, Eigen::VectorXf p1, Eigen::VectorXf p2, Eigen::VectorXf p3);

    /**
     * \brief Computes the local reference path from a gridmap using A*
     *
     * \param _gridmap              Input gridmap
     * \param _destination          Input destination point (path to destionation point solved using A*)
     * \param _local_planner_path   Output local planner path
     * \param _fake_obstacles       Output fake obstacles added to the gridmap
     **/
    void gridmap2localpath(const std::vector<Eigen::Vector2i> &_local_points, const Eigen::MatrixXi& _gridmap, const Eigen::Vector2i& _destination, std::vector<Eigen::VectorXf>& _local_planner_path, std::vector<Eigen::Vector2f>& _fake_obstacles);

    /**
    * \brief Find the coordinates of the goal point in the A* coordinate system
    *
    * \param _goal_point        Point coordinates of the destination point in vehicle coordinate system
    * \param _vehicle_state     State of the vehicle (x, y, yaw)
    * \param _octree_resolution Input octree resolution (in meters)
    * \param _gridmap_size      Input gridmap size (in meters)
    **/
    static Eigen::Vector2i getGoalPointInAStarCoords(const Eigen::Vector4f &_goal_point, const CycState &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size);
    std::vector<Eigen::Vector2i> getLocalPointsInAStarCoords(const size_t &_index, const std::vector<Eigen::Vector4f> &_mission_planner, const int &_goal_point_idx, const CycState &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size);

    AStar::Generator m_AstarGenerator;
    Eigen::Vector2f m_gridMapSize = { 20.f, 20.f };
    size_t m_PreviousTrajectoryPointIndex = 0;
    std::string m_VehicleModelFile;
    float m_LookaheadDistance;
    float m_Dt;
};

#endif //_C_ASTAR_PLANNER_H_